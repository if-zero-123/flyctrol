#include "controller_altitude.h"

#include <stdbool.h>

#include "pid.h"

static app_pid_t s_alt_pid;
static bool s_hold_valid;
static int32_t s_hold_cm;

void ControllerAltitude_Init(void)
{
  Pid_Init(&s_alt_pid, 0.002f, 0.000f, 0.000f, -150.0f, 150.0f);
  s_hold_valid = false;
  s_hold_cm = 0;
}

int16_t ControllerAltitude_Update(const baro_sample_t *baro,
                                  const control_setpoint_t *setpoint,
                                  bool active,
                                  float dt_s)
{
  if ((baro == 0) || (setpoint == 0) || !active || !setpoint->baro_hold || !baro->healthy)
  {
    s_hold_valid = false;
    Pid_Reset(&s_alt_pid);
    return 0;
  }

  if (!s_hold_valid)
  {
    s_hold_cm = baro->altitude_cm;
    s_hold_valid = true;
  }

  float correction = Pid_Update(&s_alt_pid, (float)s_hold_cm, (float)baro->altitude_cm, dt_s);
  if (correction > 150.0f)
  {
    correction = 150.0f;
  }
  if (correction < -150.0f)
  {
    correction = -150.0f;
  }
  return (int16_t)correction;
}
