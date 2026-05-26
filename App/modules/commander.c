#include "commander.h"

#if defined(__GNUC__)
#pragma GCC optimize ("Os")
#endif

#include <stddef.h>

#include "board_config.h"

static float s_baro_roll_deg;
static float s_baro_pitch_deg;
static bool s_baro_profile_ready;

static int16_t apply_expo_percent(int16_t value, uint8_t expo_percent)
{
  int32_t x = value;
  int32_t x3 = (x * x * x) / 1000000;
  int32_t expo = expo_percent;
  int32_t out = ((100 - expo) * x + expo * x3) / 100;
  if (out > 1000)
  {
    out = 1000;
  }
  if (out < -1000)
  {
    out = -1000;
  }
  return (int16_t)out;
}

static int16_t apply_expo(int16_t value)
{
  return apply_expo_percent(value, BOARD_RC_EXPO_PERCENT);
}

static int16_t apply_deadband_scaled(int16_t value, int16_t deadband)
{
  int32_t v = value;
  int32_t sign = 1;
  if (v < 0)
  {
    sign = -1;
    v = -v;
  }
  if (v <= deadband)
  {
    return 0;
  }
  v = ((v - deadband) * 1000) / (1000 - deadband);
  if (v > 1000)
  {
    v = 1000;
  }
  return (int16_t)(v * sign);
}

static float norm_to_angle(int16_t value)
{
  return ((float)apply_expo(value) / 1000.0f) * BOARD_MAX_ANGLE_DEG;
}

static float norm_to_baro_angle(int16_t value)
{
  int16_t shaped = apply_deadband_scaled(value, BOARD_BARO_RC_DEADBAND);
  shaped = apply_expo_percent(shaped, BOARD_BARO_RC_EXPO_PERCENT);
  return ((float)shaped / 1000.0f) * BOARD_BARO_MAX_ANGLE_DEG;
}

static float norm_to_yaw_rate(int16_t value)
{
  return ((float)apply_expo(value) / 1000.0f) * BOARD_MAX_YAW_RATE_DPS;
}

static float slew_float(float current, float target, float max_delta)
{
  float delta = target - current;
  if (delta > max_delta)
  {
    delta = max_delta;
  }
  else if (delta < -max_delta)
  {
    delta = -max_delta;
  }
  return current + delta;
}

static uint16_t apply_throttle_curve(uint16_t throttle)
{
  const int32_t hover = BOARD_THROTTLE_HOVER_PERMILLE;
  int32_t range = (throttle >= hover) ? (1000 - hover) : hover;
  int32_t x = (((int32_t)throttle - hover) * 1000) / range;
  int32_t x3 = (x * x * x) / 1000000;
  int32_t expo = BOARD_THROTTLE_EXPO_PERCENT;
  int32_t y = ((100 - expo) * x + expo * x3) / 100;
  int32_t out = hover + ((y * range) / 1000);
  if (out < 0)
  {
    out = 0;
  }
  if (out > 1000)
  {
    out = 1000;
  }
  return (uint16_t)out;
}

void Commander_Reset(void)
{
  s_baro_roll_deg = 0.0f;
  s_baro_pitch_deg = 0.0f;
  s_baro_profile_ready = false;
}

void Commander_BuildSetpoint(const app_rc_t *rc, control_setpoint_t *out)
{
  if ((rc == NULL) || (out == NULL))
  {
    return;
  }

  if (rc->baro_mode)
  {
    float roll_target_deg = norm_to_baro_angle(rc->roll);
    float pitch_target_deg = norm_to_baro_angle(rc->pitch);
    if (!s_baro_profile_ready)
    {
      s_baro_roll_deg = roll_target_deg;
      s_baro_pitch_deg = pitch_target_deg;
      s_baro_profile_ready = true;
    }
    else
    {
      s_baro_roll_deg = slew_float(s_baro_roll_deg,
                                   roll_target_deg,
                                   BOARD_BARO_SETPOINT_SLEW_DEG_PER_SAMPLE);
      s_baro_pitch_deg = slew_float(s_baro_pitch_deg,
                                    pitch_target_deg,
                                    BOARD_BARO_SETPOINT_SLEW_DEG_PER_SAMPLE);
    }
    out->roll_deg = s_baro_roll_deg;
    out->pitch_deg = s_baro_pitch_deg;
  }
  else
  {
    s_baro_profile_ready = false;
    out->roll_deg = norm_to_angle(rc->roll);
    out->pitch_deg = norm_to_angle(rc->pitch);
  }
  out->yaw_rate_dps = norm_to_yaw_rate(rc->yaw);
  out->throttle_permille = rc->baro_mode ? rc->throttle : apply_throttle_curve(rc->throttle);
  out->air_mode = false;
  out->baro_hold = rc->baro_mode;
}
