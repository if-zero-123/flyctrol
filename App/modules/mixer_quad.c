#include "mixer_quad.h"

#if defined(__GNUC__)
#pragma GCC optimize ("Os")
#endif

#include "board_config.h"

static uint16_t s_motor_idle_permille = BOARD_MOTOR_IDLE_PERMILLE;
static uint16_t s_motor_max_permille = BOARD_MOTOR_MAX_PERMILLE;
static uint16_t s_throttle_ramped_permille;
static bool s_throttle_ramp_ready;

static float clampf_local(float v, float min_v, float max_v)
{
  if (v < min_v)
  {
    return min_v;
  }
  if (v > max_v)
  {
    return max_v;
  }
  return v;
}

static uint16_t duty_to_permille(float duty)
{
  if (duty < 0.0f)
  {
    duty = 0.0f;
  }
  if (duty > 1.0f)
  {
    duty = 1.0f;
  }
  return (uint16_t)(duty * 1000.0f);
}

static void normalize_to_range(float motor_out[4],
                               float idle,
                               float max_out,
                               float throttle,
                               mixer_feedback_t *feedback)
{
  float min_v = motor_out[0];
  float max_v = motor_out[0];
  float scale = 1.0f;
  bool attitude_scaled = false;
  bool saturated_low = false;
  bool saturated_high = false;
  for (uint8_t i = 1U; i < 4U; i++)
  {
    if (motor_out[i] < min_v)
    {
      min_v = motor_out[i];
    }
    if (motor_out[i] > max_v)
    {
      max_v = motor_out[i];
    }
  }

  float span = max_v - min_v;
  float available = max_out - idle;
  float max_spread = (float)BOARD_MOTOR_MAX_SPREAD_PERMILLE / 1000.0f;
  if (max_spread < available)
  {
    available = max_spread;
  }
  if ((span > available) && (span > 0.0001f))
  {
    scale = available / span;
    attitude_scaled = true;
    for (uint8_t i = 0U; i < 4U; i++)
    {
      motor_out[i] = throttle + ((motor_out[i] - throttle) * scale);
    }
    min_v = motor_out[0];
    max_v = motor_out[0];
    for (uint8_t i = 1U; i < 4U; i++)
    {
      if (motor_out[i] < min_v)
      {
        min_v = motor_out[i];
      }
      if (motor_out[i] > max_v)
      {
        max_v = motor_out[i];
      }
    }
  }
  float offset = 0.0f;
  if (min_v < idle)
  {
    saturated_low = true;
    offset = idle - min_v;
  }
  if ((max_v + offset) > max_out)
  {
    saturated_high = true;
    offset = max_out - max_v;
  }
  for (uint8_t i = 0U; i < 4U; i++)
  {
    motor_out[i] += offset;
  }

  for (uint8_t i = 0U; i < 4U; i++)
  {
    motor_out[i] = clampf_local(motor_out[i], idle, max_out);
  }

  if (feedback != 0)
  {
    float final_min = motor_out[0];
    float final_max = motor_out[0];
    for (uint8_t i = 1U; i < 4U; i++)
    {
      if (motor_out[i] < final_min)
      {
        final_min = motor_out[i];
      }
      if (motor_out[i] > final_max)
      {
        final_max = motor_out[i];
      }
    }

    feedback->motor_min_permille = duty_to_permille(final_min);
    feedback->motor_max_permille = duty_to_permille(final_max);
    feedback->motor_spread_permille = (uint16_t)(feedback->motor_max_permille -
                                                 feedback->motor_min_permille);
    feedback->attitude_scale_permille = duty_to_permille(scale);
    feedback->saturated_high = saturated_high || (feedback->motor_max_permille >= s_motor_max_permille);
    feedback->saturated_low = saturated_low || (feedback->motor_min_permille <= s_motor_idle_permille);
    feedback->attitude_scaled = attitude_scaled;
  }
}

void MixerQuad_SetMotorIdlePermille(uint16_t permille)
{
  if (permille > BOARD_MOTOR_IDLE_MAX_PERMILLE)
  {
    permille = BOARD_MOTOR_IDLE_MAX_PERMILLE;
  }
  s_motor_idle_permille = permille;
  if (s_motor_max_permille < s_motor_idle_permille)
  {
    s_motor_max_permille = s_motor_idle_permille;
  }
}

uint16_t MixerQuad_GetMotorIdlePermille(void)
{
  return s_motor_idle_permille;
}

void MixerQuad_SetMotorMaxPermille(uint16_t permille)
{
  if (permille > 1000U)
  {
    permille = 1000U;
  }
  if (permille < s_motor_idle_permille)
  {
    permille = s_motor_idle_permille;
  }
  s_motor_max_permille = permille;
}

uint16_t MixerQuad_GetMotorMaxPermille(void)
{
  return s_motor_max_permille;
}

void MixerQuad_ResetThrottleRamp(void)
{
  s_throttle_ramped_permille = 0U;
  s_throttle_ramp_ready = false;
}

void MixerQuad_PrimeThrottleRamp(uint16_t permille)
{
  if (permille < s_motor_idle_permille)
  {
    permille = s_motor_idle_permille;
  }
  if (permille > s_motor_max_permille)
  {
    permille = s_motor_max_permille;
  }
  s_throttle_ramped_permille = permille;
  s_throttle_ramp_ready = true;
}

static uint16_t slew_throttle(uint16_t throttle_permille)
{
  if (!s_throttle_ramp_ready)
  {
    s_throttle_ramped_permille = throttle_permille;
    s_throttle_ramp_ready = true;
    return s_throttle_ramped_permille;
  }

  if (throttle_permille <= s_throttle_ramped_permille)
  {
    s_throttle_ramped_permille = throttle_permille;
    return s_throttle_ramped_permille;
  }

  uint16_t delta = (uint16_t)(throttle_permille - s_throttle_ramped_permille);
  if (delta > BOARD_THROTTLE_SLEW_PER_LOOP)
  {
    delta = BOARD_THROTTLE_SLEW_PER_LOOP;
  }
  s_throttle_ramped_permille = (uint16_t)(s_throttle_ramped_permille + delta);
  return s_throttle_ramped_permille;
}

void MixerQuad_Mix(uint16_t throttle_permille, const control_output_t *control, float motor_out[4])
{
  MixerQuad_MixWithFeedback(throttle_permille, control, motor_out, 0);
}

void MixerQuad_MixWithFeedback(uint16_t throttle_permille,
                               const control_output_t *control,
                               float motor_out[4],
                               mixer_feedback_t *feedback)
{
  if ((control == 0) || (motor_out == 0))
  {
    return;
  }

  int32_t throttle = (int32_t)throttle_permille + control->altitude_permille;
  if (throttle < 0)
  {
    throttle = 0;
  }
  if (throttle > 1000)
  {
    throttle = 1000;
  }

  uint16_t idle_permille = s_motor_idle_permille;
  if (throttle < (int32_t)idle_permille)
  {
    throttle = (int32_t)idle_permille;
  }
  if (throttle > (int32_t)s_motor_max_permille)
  {
    throttle = (int32_t)s_motor_max_permille;
  }
  throttle = (int32_t)slew_throttle((uint16_t)throttle);
  if (feedback != 0)
  {
    feedback->throttle_permille = (uint16_t)throttle;
    feedback->motor_min_permille = (uint16_t)throttle;
    feedback->motor_max_permille = (uint16_t)throttle;
    feedback->motor_spread_permille = 0U;
    feedback->attitude_scale_permille = 1000U;
    feedback->saturated_high = false;
    feedback->saturated_low = false;
    feedback->attitude_scaled = false;
  }

  float idle = (float)idle_permille / 1000.0f;
  float max_out = (float)s_motor_max_permille / 1000.0f;
  float t = (float)throttle / 1000.0f;
  float r = control->roll;
  float p = control->pitch;
  float y = control->yaw;

  motor_out[0] = t - r + p - y; /* M1 rear-right, CW */
  motor_out[1] = t - r - p + y; /* M2 front-right, CCW */
  motor_out[2] = t + r + p + y; /* M3 rear-left, CCW */
  motor_out[3] = t + r - p - y; /* M4 front-left, CW */
  normalize_to_range(motor_out, idle, max_out, t, feedback);
}
