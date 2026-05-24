#ifndef APP_CONTROLLER_ATTITUDE_H
#define APP_CONTROLLER_ATTITUDE_H

#include <stdbool.h>
#include "flight_types.h"
#include "pid.h"

typedef enum {
  PID_AXIS_ROLL = 0,
  PID_AXIS_PITCH = 1,
  PID_AXIS_YAW = 2
} pid_axis_t;

void ControllerAttitude_Init(void);
void ControllerAttitude_Update(const attitude_t *attitude,
                               const imu_sample_t *imu,
                               const control_setpoint_t *setpoint,
                               float dt_s,
                               control_output_t *out);
bool ControllerAttitude_SetPid(pid_axis_t axis, float kp, float ki, float kd);
bool ControllerAttitude_GetPid(pid_axis_t axis, app_pid_t *out);

#endif /* APP_CONTROLLER_ATTITUDE_H */
