#include "mixer_quad.h"

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

static void normalize_to_range(float motor_out[4], float idle, float max_out, float throttle)
{
  float min_v = motor_out[0];
  float max_v = motor_out[0];
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
  if ((span > available) && (span > 0.0001f))
  {
    float scale = available / span;
    for (uint8_t i = 0U; i < 4U; i++)
    {
      motor_out[i] = throttle + ((motor_out[i] - throttle) * scale);
    }
  }
  else if (throttle >= 0.50f)
  {
    float offset = 0.0f;
    if (min_v < idle)
    {
      offset = idle - min_v;
    }
    if ((max_v + offset) > max_out)
    {
      offset = max_out - max_v;
    }
    for (uint8_t i = 0U; i < 4U; i++)
    {
      motor_out[i] += offset;
    }
  }

  for (uint8_t i = 0U; i < 4U; i++)
  {
    motor_out[i] = clampf_local(motor_out[i], idle, max_out);
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
  normalize_to_range(motor_out, idle, max_out, t);
}
