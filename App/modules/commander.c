#include "commander.h"

#include <stddef.h>

#include "board_config.h"

static float norm_to_angle(int16_t value)
{
  return ((float)value / 1000.0f) * BOARD_MAX_ANGLE_DEG;
}

static float norm_to_yaw_rate(int16_t value)
{
  return ((float)value / 1000.0f) * BOARD_MAX_YAW_RATE_DPS;
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
  out->throttle_permille = rc->throttle;
  out->baro_hold = rc->baro_mode;
}
