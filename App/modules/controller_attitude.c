#include "controller_attitude.h"

#if defined(__GNUC__)
#pragma GCC optimize ("Os")
#endif

#include <string.h>

#include "board_config.h"

#define BF_PTERM_SCALE_NORM 0.000032029f
#define BF_ITERM_SCALE_NORM 0.000244381f
#define BF_DTERM_SCALE_NORM 0.000000529f
#define BF_DTERM_LPF_HZ     70.0f
#define BF_PID_LIMIT_RP     0.16f
#define BF_PID_LIMIT_YAW    0.10f
#define BF_ITERM_LIMIT_RP   0.05f
#define BF_ITERM_LIMIT_YAW  0.04f

typedef struct {
  app_pid_t pid;
  uint8_t bf_p;
  uint8_t bf_i;
  uint8_t bf_d;
  float filtered_rate_dps;
  bool filter_ready;
  float p_term;
  float i_term;
  float d_term;
  float rate_sp_dps;
  float gyro_dps;
  float error_dps;
  float output;
} bf_axis_t;

static bf_axis_t s_axis[3];
static int8_t s_yaw_gyro_direction = BOARD_YAW_GYRO_DIRECTION;
static controller_attitude_debug_t s_debug;

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

static float absf_local(float v)
{
  return (v < 0.0f) ? -v : v;
}

static int16_t float_to_i16(float v)
{
  if (v > 32767.0f)
  {
    return 32767;
  }
  if (v < -32768.0f)
  {
    return -32768;
  }
  return (int16_t)v;
}

static bf_axis_t *axis_state(pid_axis_t axis)
{
  if ((axis < PID_AXIS_ROLL) || (axis > PID_AXIS_YAW))
  {
    return 0;
  }
  return &s_axis[(uint8_t)axis];
}

static float pt1_alpha(float cutoff_hz, float dt_s)
{
  const float pi = 3.14159265f;
  if ((cutoff_hz <= 0.0f) || (dt_s <= 0.0f))
  {
    return 1.0f;
  }
  float rc = 1.0f / (2.0f * pi * cutoff_hz);
  return clampf_local(dt_s / (dt_s + rc), 0.0f, 1.0f);
}

static void set_axis_bf(pid_axis_t axis, uint8_t p, uint8_t i, uint8_t d)
{
  bf_axis_t *state = axis_state(axis);
  if (state == 0)
  {
    return;
  }

  float out_limit = (axis == PID_AXIS_YAW) ? BF_PID_LIMIT_YAW : BF_PID_LIMIT_RP;
  float i_limit = (axis == PID_AXIS_YAW) ? BF_ITERM_LIMIT_YAW : BF_ITERM_LIMIT_RP;

  state->bf_p = p;
  state->bf_i = i;
  state->bf_d = d;
  Pid_Init(&state->pid,
           BF_PTERM_SCALE_NORM * (float)p,
           BF_ITERM_SCALE_NORM * (float)i,
           BF_DTERM_SCALE_NORM * (float)d,
           -out_limit,
           out_limit);
  state->pid.i_min = -i_limit;
  state->pid.i_max = i_limit;
  state->filter_ready = false;
}

static void reset_axis(bf_axis_t *state)
{
  if (state == 0)
  {
    return;
  }
  Pid_Reset(&state->pid);
  state->filtered_rate_dps = 0.0f;
  state->filter_ready = false;
  state->p_term = 0.0f;
  state->i_term = 0.0f;
  state->d_term = 0.0f;
  state->rate_sp_dps = 0.0f;
  state->gyro_dps = 0.0f;
  state->error_dps = 0.0f;
  state->output = 0.0f;
}

static float update_rate_axis(bf_axis_t *state,
                              float rate_sp_dps,
                              float gyro_dps,
                              float dt_s,
                              bool allow_integrator,
                              float output_limit)
{
  if ((state == 0) || (dt_s <= 0.0f))
  {
    return 0.0f;
  }

  if (!state->filter_ready)
  {
    state->filtered_rate_dps = gyro_dps;
    state->pid.previous_measurement = gyro_dps;
    state->filter_ready = true;
  }

  float alpha = pt1_alpha(BF_DTERM_LPF_HZ, dt_s);
  state->filtered_rate_dps += alpha * (gyro_dps - state->filtered_rate_dps);

  float error = rate_sp_dps - gyro_dps;
  state->p_term = state->pid.kp * error;
  float rate_delta = (state->filtered_rate_dps - state->pid.previous_measurement) / dt_s;
  state->pid.previous_measurement = state->filtered_rate_dps;
  state->d_term = -state->pid.kd * rate_delta;

  float output_without_i = state->p_term + state->d_term;
  float output_with_i = output_without_i + state->pid.integrator;
  bool iterm_relaxed = absf_local(rate_sp_dps) > BOARD_ITERM_RELAX_RATE_DPS;
  bool error_too_large = absf_local(error) > BOARD_ITERM_ERROR_LIMIT_DPS;
  bool saturated_high = (output_with_i > (output_limit * 0.90f)) && (error > 0.0f);
  bool saturated_low = (output_with_i < (-output_limit * 0.90f)) && (error < 0.0f);

  if (allow_integrator && !iterm_relaxed && !error_too_large && !saturated_high && !saturated_low)
  {
    state->pid.integrator += state->pid.ki * error * dt_s;
    state->pid.integrator = clampf_local(state->pid.integrator, state->pid.i_min, state->pid.i_max);
  }
  else
  {
    state->pid.integrator *= BOARD_ITERM_DECAY;
  }

  state->i_term = state->pid.integrator;
  state->output = clampf_local(state->p_term + state->i_term + state->d_term,
                               -output_limit,
                               output_limit);
  state->rate_sp_dps = rate_sp_dps;
  state->gyro_dps = gyro_dps;
  state->error_dps = error;
  return state->output;
}

void ControllerAttitude_Init(void)
{
  ControllerAttitude_UseSafeDefaults();
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
    ControllerAttitude_Reset();
    return;
  }

  float roll_error_deg = clampf_local(setpoint->roll_deg - attitude->roll_deg,
                                      -BOARD_MAX_ANGLE_DEG,
                                      BOARD_MAX_ANGLE_DEG);
  float pitch_error_deg = clampf_local(setpoint->pitch_deg - attitude->pitch_deg,
                                       -BOARD_MAX_ANGLE_DEG,
                                       BOARD_MAX_ANGLE_DEG);
  float roll_rate_sp = clampf_local(roll_error_deg * BOARD_LEVEL_GAIN_DPS_PER_DEG,
                                    -BOARD_MAX_LEVEL_RATE_DPS,
                                    BOARD_MAX_LEVEL_RATE_DPS);
  float pitch_rate_sp = clampf_local(pitch_error_deg * BOARD_LEVEL_GAIN_DPS_PER_DEG,
                                     -BOARD_MAX_LEVEL_RATE_DPS,
                                     BOARD_MAX_LEVEL_RATE_DPS);
  float yaw_rate_sp = clampf_local(setpoint->yaw_rate_dps,
                                   -BOARD_MAX_YAW_RATE_DPS,
                                   BOARD_MAX_YAW_RATE_DPS);
  float yaw_rate_dps = ((float)s_yaw_gyro_direction) * imu->gyro_dps[2];
  bool allow_integrator = setpoint->air_mode || (setpoint->throttle_permille > BOARD_ARM_THROTTLE_MAX);

  out->roll = update_rate_axis(&s_axis[PID_AXIS_ROLL],
                               roll_rate_sp,
                               imu->gyro_dps[0],
                               dt_s,
                               allow_integrator,
                               BF_PID_LIMIT_RP);
  out->pitch = update_rate_axis(&s_axis[PID_AXIS_PITCH],
                                pitch_rate_sp,
                                imu->gyro_dps[1],
                                dt_s,
                                allow_integrator,
                                BF_PID_LIMIT_RP);
  out->yaw = update_rate_axis(&s_axis[PID_AXIS_YAW],
                              yaw_rate_sp,
                              yaw_rate_dps,
                              dt_s,
                              allow_integrator,
                              BF_PID_LIMIT_YAW);

  for (uint8_t i = 0U; i < 3U; i++)
  {
    s_debug.rate_setpoint_dps[i] = float_to_i16(s_axis[i].rate_sp_dps);
    s_debug.gyro_dps[i] = float_to_i16(s_axis[i].gyro_dps);
    s_debug.error_dps[i] = float_to_i16(s_axis[i].error_dps);
    s_debug.p_milli[i] = float_to_i16(s_axis[i].p_term * 1000.0f);
    s_debug.i_milli[i] = float_to_i16(s_axis[i].i_term * 1000.0f);
    s_debug.d_milli[i] = float_to_i16(s_axis[i].d_term * 1000.0f);
    s_debug.output_milli[i] = float_to_i16(s_axis[i].output * 1000.0f);
  }
}

void ControllerAttitude_Reset(void)
{
  for (uint8_t i = 0U; i < 3U; i++)
  {
    reset_axis(&s_axis[i]);
  }
  memset(&s_debug, 0, sizeof(s_debug));
}

void ControllerAttitude_SetYawGyroDirection(int8_t direction)
{
  s_yaw_gyro_direction = (direction < 0) ? -1 : 1;
  reset_axis(&s_axis[PID_AXIS_YAW]);
}

int8_t ControllerAttitude_GetYawGyroDirection(void)
{
  return s_yaw_gyro_direction;
}

bool ControllerAttitude_SetPid(pid_axis_t axis, float kp, float ki, float kd)
{
  bf_axis_t *state = axis_state(axis);
  if (state == 0)
  {
    return false;
  }

  state->pid.kp = kp;
  state->pid.ki = ki;
  state->pid.kd = kd;
  reset_axis(state);
  return true;
}

bool ControllerAttitude_GetPid(pid_axis_t axis, app_pid_t *out)
{
  if (out == 0)
  {
    return false;
  }
  bf_axis_t *state = axis_state(axis);
  if (state == 0)
  {
    return false;
  }
  *out = state->pid;
  return true;
}

bool ControllerAttitude_SetBfPid(pid_axis_t axis, uint8_t p, uint8_t i, uint8_t d)
{
  if (axis_state(axis) == 0)
  {
    return false;
  }
  set_axis_bf(axis, p, i, d);
  reset_axis(&s_axis[(uint8_t)axis]);
  return true;
}

bool ControllerAttitude_GetBfPid(pid_axis_t axis, uint8_t *p, uint8_t *i, uint8_t *d)
{
  bf_axis_t *state = axis_state(axis);
  if (state == 0)
  {
    return false;
  }
  if (p != 0)
  {
    *p = state->bf_p;
  }
  if (i != 0)
  {
    *i = state->bf_i;
  }
  if (d != 0)
  {
    *d = state->bf_d;
  }
  return true;
}

void ControllerAttitude_UseSafeDefaults(void)
{
  set_axis_bf(PID_AXIS_ROLL, 24U, 10U, 0U);
  set_axis_bf(PID_AXIS_PITCH, 26U, 12U, 0U);
  set_axis_bf(PID_AXIS_YAW, 35U, 6U, 0U);
  ControllerAttitude_Reset();
}

void ControllerAttitude_UseBfDefaults(void)
{
  set_axis_bf(PID_AXIS_ROLL, 60U, 70U, 17U);
  set_axis_bf(PID_AXIS_PITCH, 80U, 90U, 18U);
  set_axis_bf(PID_AXIS_YAW, 200U, 45U, 0U);
  ControllerAttitude_Reset();
}

void ControllerAttitude_GetDebug(controller_attitude_debug_t *out)
{
  if (out != 0)
  {
    *out = s_debug;
  }
}
