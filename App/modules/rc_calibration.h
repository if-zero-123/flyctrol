#ifndef APP_RC_CALIBRATION_H
#define APP_RC_CALIBRATION_H

#include <stdint.h>

typedef enum {
  RC_CAL_AXIS_ROLL = 0,
  RC_CAL_AXIS_PITCH = 1,
  RC_CAL_AXIS_YAW = 2
} rc_cal_axis_t;

void RcCalibration_Reset(void);
void RcCalibration_SetCenter(uint16_t roll_raw, uint16_t pitch_raw, uint16_t yaw_raw);
uint16_t RcCalibration_GetCenter(rc_cal_axis_t axis);
int16_t RcCalibration_NormalizeStick(rc_cal_axis_t axis, uint16_t raw);
uint16_t RcCalibration_NormalizeThrottle(uint16_t raw);

#endif /* APP_RC_CALIBRATION_H */
