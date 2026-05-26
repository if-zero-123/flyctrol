#include "rc_calibration.h"

#if defined(__GNUC__)
#pragma GCC optimize ("Os")
#endif

#include "board_config.h"

static uint16_t s_center[3] = {
  BOARD_CRSF_RAW_MID,
  BOARD_CRSF_RAW_MID,
  BOARD_CRSF_RAW_MID
};

static uint16_t clamp_center(uint16_t raw)
{
  if (raw <= (BOARD_CRSF_RAW_MIN + 40U))
  {
    return BOARD_CRSF_RAW_MID;
  }
  if (raw >= (BOARD_CRSF_RAW_MAX - 40U))
  {
    return BOARD_CRSF_RAW_MID;
  }
  return raw;
}

void RcCalibration_Reset(void)
{
  s_center[RC_CAL_AXIS_ROLL] = BOARD_CRSF_RAW_MID;
  s_center[RC_CAL_AXIS_PITCH] = BOARD_CRSF_RAW_MID;
  s_center[RC_CAL_AXIS_YAW] = BOARD_CRSF_RAW_MID;
}

void RcCalibration_SetCenter(uint16_t roll_raw, uint16_t pitch_raw, uint16_t yaw_raw)
{
  s_center[RC_CAL_AXIS_ROLL] = clamp_center(roll_raw);
  s_center[RC_CAL_AXIS_PITCH] = clamp_center(pitch_raw);
  s_center[RC_CAL_AXIS_YAW] = clamp_center(yaw_raw);
}

uint16_t RcCalibration_GetCenter(rc_cal_axis_t axis)
{
  if ((axis < RC_CAL_AXIS_ROLL) || (axis > RC_CAL_AXIS_YAW))
  {
    return BOARD_CRSF_RAW_MID;
  }
  return s_center[(uint8_t)axis];
}

int16_t RcCalibration_NormalizeStick(rc_cal_axis_t axis, uint16_t raw)
{
  uint16_t center = RcCalibration_GetCenter(axis);
  int32_t span;
  int32_t v;

  if (raw >= center)
  {
    span = (int32_t)BOARD_CRSF_RAW_MAX - (int32_t)center;
  }
  else
  {
    span = (int32_t)center - (int32_t)BOARD_CRSF_RAW_MIN;
  }
  if (span < 1)
  {
    span = 1;
  }

  v = (((int32_t)raw - (int32_t)center) * 1000) / span;
  if ((v > -BOARD_RC_DEADBAND) && (v < BOARD_RC_DEADBAND))
  {
    v = 0;
  }
  if (v < -1000)
  {
    v = -1000;
  }
  if (v > 1000)
  {
    v = 1000;
  }
  return (int16_t)v;
}

uint16_t RcCalibration_NormalizeThrottle(uint16_t raw)
{
  int32_t v = ((int32_t)raw - (int32_t)BOARD_CRSF_RAW_MIN) * 1000 /
              ((int32_t)BOARD_CRSF_RAW_MAX - (int32_t)BOARD_CRSF_RAW_MIN);
  if (v < 0)
  {
    v = 0;
  }
  if (v > 1000)
  {
    v = 1000;
  }
  return (uint16_t)v;
}
