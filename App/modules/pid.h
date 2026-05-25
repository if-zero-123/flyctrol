#ifndef APP_PID_H
#define APP_PID_H

typedef struct {
  float kp;
  float ki;
  float kd;
  float integrator;
  float previous_error;
  float previous_measurement;
  float out_min;
  float out_max;
  float i_min;
  float i_max;
} app_pid_t;

void Pid_Init(app_pid_t *pid, float kp, float ki, float kd, float out_min, float out_max);
void Pid_Reset(app_pid_t *pid);
float Pid_Update(app_pid_t *pid, float setpoint, float measurement, float dt_s);

#endif /* APP_PID_H */
