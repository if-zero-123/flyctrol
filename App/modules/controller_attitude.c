#include "controller_attitude.h"

#include "board_config.h"

static app_pid_t s_roll;
static app_pid_t s_pitch;
static app_pid_t s_yaw_rate;
static int8_t s_yaw_gyro_direction = BOARD_YAW_GYRO_DIRECTION;

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

void ControllerAttitude_Init(void)
{
  Pid_Init(&s_roll, 0.012f, 0.000f, 0.001f, -0.35f, 0.35f);
  Pid_Init(&s_pitch, 0.012f, 0.000f, 0.001f, -0.35f, 0.35f);
  Pid_Init(&s_yaw_rate, 0.004f, 0.000f, 0.000f, -0.25f, 0.25f);
  ControllerAttitude_SetYawGyroDirection(BOARD_YAW_GYRO_DIRECTION);
}

void ControllerAttitude_Update(const attitude_t *attitude,
                               const imu_sample_t *imu,
                               const control_setpoint_t *setpoint,
                               float dt_s,
                               control_output_t *out)
{
  if (out == 0)
  {
    return;
  }

  out->roll = 0.0f;
  out->pitch = 0.0f;
  out->yaw = 0.0f;

  if ((attitude == 0) || (imu == 0) || (setpoint == 0) || !attitude->healthy || !imu->healthy)
  {
    Pid_Reset(&s_roll);
    Pid_Reset(&s_pitch);
    Pid_Reset(&s_yaw_rate);
    return;
  }

  out->roll = Pid_Update(&s_roll, setpoint->roll_deg, attitude->roll_deg, dt_s);
  out->pitch = Pid_Update(&s_pitch, setpoint->pitch_deg, attitude->pitch_deg, dt_s);
  float yaw_rate_dps = ((float)s_yaw_gyro_direction) * imu->gyro_dps[2];
  out->yaw = Pid_Update(&s_yaw_rate, setpoint->yaw_rate_dps, yaw_rate_dps, dt_s);
  out->roll = clampf_local(out->roll, -0.35f, 0.35f);
  out->pitch = clampf_local(out->pitch, -0.35f, 0.35f);
  out->yaw = clampf_local(out->yaw, -0.25f, 0.25f);
}

void ControllerAttitude_Reset(void)
{
  Pid_Reset(&s_roll);
  Pid_Reset(&s_pitch);
  Pid_Reset(&s_yaw_rate);
}

void ControllerAttitude_SetYawGyroDirection(int8_t direction)
{
  s_yaw_gyro_direction = (direction < 0) ? -1 : 1;
  Pid_Reset(&s_yaw_rate);
}

int8_t ControllerAttitude_GetYawGyroDirection(void)
{
  return s_yaw_gyro_direction;
}

bool ControllerAttitude_SetPid(pid_axis_t axis, float kp, float ki, float kd)
{
  app_pid_t *pid = 0;
  switch (axis)
  {
    case PID_AXIS_ROLL:
      pid = &s_roll;
      break;
    case PID_AXIS_PITCH:
      pid = &s_pitch;
      break;
    case PID_AXIS_YAW:
      pid = &s_yaw_rate;
      break;
    default:
      return false;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  Pid_Reset(pid);
  return true;
}

bool ControllerAttitude_GetPid(pid_axis_t axis, app_pid_t *out)
{
  if (out == 0)
  {
    return false;
  }
  switch (axis)
  {
    case PID_AXIS_ROLL:
      *out = s_roll;
      return true;
    case PID_AXIS_PITCH:
      *out = s_pitch;
      return true;
    case PID_AXIS_YAW:
      *out = s_yaw_rate;
      return true;
    default:
      return false;
  }
}
