#include "commander.h"

#include <stddef.h>

#include "board_config.h"

static int16_t apply_expo(int16_t value)
{
  int32_t x = value;
  int32_t x3 = (x * x * x) / 1000000;
  int32_t expo = BOARD_RC_EXPO_PERCENT;
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

static float norm_to_angle(int16_t value)
{
  return ((float)apply_expo(value) / 1000.0f) * BOARD_MAX_ANGLE_DEG;
}

static float norm_to_yaw_rate(int16_t value)
{
  return ((float)apply_expo(value) / 1000.0f) * BOARD_MAX_YAW_RATE_DPS;
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

void Commander_BuildSetpoint(const app_rc_t *rc, control_setpoint_t *out)
{
  if ((rc == NULL) || (out == NULL))
  {
    return;
  }

  out->roll_deg = norm_to_angle(rc->roll);
  out->pitch_deg = norm_to_angle(rc->pitch);
  out->yaw_rate_dps = norm_to_yaw_rate(rc->yaw);
  out->throttle_permille = rc->baro_mode ? rc->throttle : apply_throttle_curve(rc->throttle);
  out->air_mode = false;
  out->baro_hold = rc->baro_mode;
}
