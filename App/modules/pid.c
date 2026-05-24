#include "pid.h"

static float clampf(float v, float min_v, float max_v)
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

void Pid_Init(app_pid_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
  if (pid == 0)
  {
    return;
  }
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integrator = 0.0f;
  pid->previous_error = 0.0f;
  pid->out_min = out_min;
  pid->out_max = out_max;
  pid->i_min = out_min;
  pid->i_max = out_max;
}

void Pid_Reset(app_pid_t *pid)
{
  if (pid == 0)
  {
    return;
  }
  pid->integrator = 0.0f;
  pid->previous_error = 0.0f;
}

float Pid_Update(app_pid_t *pid, float setpoint, float measurement, float dt_s)
{
  if ((pid == 0) || (dt_s <= 0.0f))
  {
    return 0.0f;
  }

  float error = setpoint - measurement;
  pid->integrator += error * pid->ki * dt_s;
  pid->integrator = clampf(pid->integrator, pid->i_min, pid->i_max);
  float derivative = (error - pid->previous_error) / dt_s;
  pid->previous_error = error;

  return clampf((pid->kp * error) + pid->integrator + (pid->kd * derivative),
                pid->out_min,
                pid->out_max);
}
