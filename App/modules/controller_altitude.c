#include "controller_altitude.h"

#include <string.h>

#include "board_config.h"

static bool s_active;
static bool s_velocity_control;
static int32_t s_hold_altitude_cm;
static int16_t s_stick_reference_permille;
static int16_t s_throttle_base_permille;
static float s_integrator_permille;
static int16_t s_last_output_permille;
static uint32_t s_last_baro_timestamp_ms;
static controller_altitude_debug_t s_debug;

static int32_t abs_i32(int32_t v)
{
  return (v < 0) ? -v : v;
}

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

static int16_t clamp_i16(int32_t v, int32_t min_v, int32_t max_v)
{
  if (v < min_v)
  {
    return (int16_t)min_v;
  }
  if (v > max_v)
  {
    return (int16_t)max_v;
  }
  return (int16_t)v;
}

static int16_t float_to_i16(float v)
{
  if (v > 32767.0f)
  {
    return 32767;
  }
  if (v < -32768.0f)
  {
    return -32768;
  }
  return (int16_t)v;
}

void ControllerAltitude_Init(void)
{
  ControllerAltitude_Reset();
}

void ControllerAltitude_Reset(void)
{
  s_active = false;
  s_velocity_control = false;
  s_hold_altitude_cm = 0;
  s_stick_reference_permille = 0;
  s_throttle_base_permille = 0;
  s_integrator_permille = 0.0f;
  s_last_output_permille = 0;
  s_last_baro_timestamp_ms = 0U;
  memset(&s_debug, 0, sizeof(s_debug));
}

int16_t ControllerAltitude_Update(const baro_sample_t *baro,
                                  const control_setpoint_t *setpoint,
                                  bool active,
                                  float dt_s)
{
  (void)dt_s;
  if ((baro == 0) || (setpoint == 0) || !active || !setpoint->baro_hold || !baro->healthy)
  {
    ControllerAltitude_Reset();
    return 0;
  }

  if (!s_active)
  {
    s_active = true;
    s_velocity_control = false;
    s_hold_altitude_cm = baro->altitude_cm;
    s_stick_reference_permille = (int16_t)BOARD_ALT_STICK_CENTER_PERMILLE;
    s_throttle_base_permille = clamp_i16(BOARD_ALT_HOVER_THRUST_PERMILLE,
                                         BOARD_ALT_BASE_MIN_PERMILLE,
                                         BOARD_ALT_BASE_MAX_PERMILLE);
    s_integrator_permille = 0.0f;
    s_last_output_permille = 0;
    s_last_baro_timestamp_ms = baro->timestamp_ms;
  }

  if (baro->timestamp_ms == s_last_baro_timestamp_ms)
  {
    return s_last_output_permille;
  }

  float baro_dt_s = 0.025f;
  if (baro->timestamp_ms > s_last_baro_timestamp_ms)
  {
    baro_dt_s = (float)(baro->timestamp_ms - s_last_baro_timestamp_ms) / 1000.0f;
    baro_dt_s = clampf_local(baro_dt_s, 0.010f, 0.150f);
  }
  s_last_baro_timestamp_ms = baro->timestamp_ms;

  int32_t throttle_delta = (int32_t)setpoint->throttle_permille - (int32_t)s_stick_reference_permille;
  int32_t abs_delta = abs_i32(throttle_delta);
  int32_t target_velocity_cms = 0;
  int32_t altitude_error_cm = 0;

  if (abs_delta > BOARD_ALT_HOLD_DB_PERMILLE)
  {
    int32_t usable = 1000 - BOARD_ALT_HOLD_DB_PERMILLE;
    int32_t mag = abs_delta - BOARD_ALT_HOLD_DB_PERMILLE;
    target_velocity_cms = (mag * BOARD_ALT_STICK_MAX_VEL_CMS) / usable;
    if (throttle_delta < 0)
    {
      target_velocity_cms = -target_velocity_cms;
    }
    s_hold_altitude_cm = baro->altitude_cm;
    s_velocity_control = true;
  }
  else
  {
    if (s_velocity_control)
    {
      s_hold_altitude_cm = baro->altitude_cm;
      s_velocity_control = false;
      s_integrator_permille = 0.0f;
    }

    altitude_error_cm = s_hold_altitude_cm - baro->altitude_cm;
    if (abs_i32(altitude_error_cm) <= BOARD_ALT_HOLD_DB_CM)
    {
      altitude_error_cm = 0;
    }
    target_velocity_cms = (int32_t)((float)altitude_error_cm * BOARD_ALT_HOLD_POS_P);
    target_velocity_cms = clamp_i16(target_velocity_cms,
                                    -BOARD_ALT_HOLD_MAX_VEL_CMS,
                                    BOARD_ALT_HOLD_MAX_VEL_CMS);
  }

  int32_t velocity_error = target_velocity_cms - baro->velocity_cms;
  float p_term = BOARD_ALT_VEL_P * (float)velocity_error;
  float output_before_i = p_term + s_integrator_permille;
  bool saturated_high = (output_before_i > ((float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE * 0.90f)) && (velocity_error > 0);
  bool saturated_low = (output_before_i < (-(float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE * 0.90f)) && (velocity_error < 0);

  if (!saturated_high && !saturated_low)
  {
    s_integrator_permille += BOARD_ALT_VEL_I * (float)velocity_error * baro_dt_s;
    s_integrator_permille = clampf_local(s_integrator_permille,
                                         -BOARD_ALT_I_LIMIT_PERMILLE,
                                         BOARD_ALT_I_LIMIT_PERMILLE);
  }

  float correction = p_term + s_integrator_permille;
  correction = clampf_local(correction,
                            -(float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE,
                            (float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE);

  int32_t desired_total = (int32_t)s_throttle_base_permille + (int32_t)correction;
  desired_total = clamp_i16(desired_total, 0, 1000);
  int16_t desired_output = clamp_i16(desired_total - (int32_t)setpoint->throttle_permille,
                                     -BOARD_ALT_OUTPUT_LIMIT_PERMILLE,
                                     BOARD_ALT_OUTPUT_LIMIT_PERMILLE);
  int32_t output_delta = (int32_t)desired_output - (int32_t)s_last_output_permille;
  if (output_delta > BOARD_ALT_OUTPUT_SLEW_PER_SAMPLE)
  {
    output_delta = BOARD_ALT_OUTPUT_SLEW_PER_SAMPLE;
  }
  else if (output_delta < -BOARD_ALT_OUTPUT_SLEW_PER_SAMPLE)
  {
    output_delta = -BOARD_ALT_OUTPUT_SLEW_PER_SAMPLE;
  }
  s_last_output_permille = (int16_t)((int32_t)s_last_output_permille + output_delta);

  s_debug.active = true;
  s_debug.velocity_control = s_velocity_control;
  s_debug.hold_altitude_cm = s_hold_altitude_cm;
  s_debug.altitude_error_cm = clamp_i16(altitude_error_cm, -32768, 32767);
  s_debug.velocity_cms = baro->velocity_cms;
  s_debug.target_velocity_cms = clamp_i16(target_velocity_cms, -32768, 32767);
  s_debug.throttle_base_permille = s_throttle_base_permille;
  s_debug.correction_permille = float_to_i16(correction);
  s_debug.output_permille = s_last_output_permille;
  return s_last_output_permille;
}

void ControllerAltitude_GetDebug(controller_altitude_debug_t *out)
{
  if (out != 0)
  {
    *out = s_debug;
  }
}

bool ControllerAltitude_IsActive(void)
{
  return s_active;
}
