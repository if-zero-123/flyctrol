#include "estimator_attitude.h"

#include <stddef.h>

static attitude_t s_attitude;
static float s_roll_trim_deg;
static float s_pitch_trim_deg;

static float absf_local(float v)
{
  return (v < 0.0f) ? -v : v;
}

static float atan2_approx_rad(float y, float x)
{
  const float oneqtr_pi = 0.78539816339f;
  const float thrqtr_pi = 2.35619449019f;
  float abs_y = absf_local(y) + 0.0000001f;
  float r;
  float angle;

  if (x < 0.0f)
  {
    r = (x + abs_y) / (abs_y - x);
    angle = thrqtr_pi;
  }
  else
  {
    r = (x - abs_y) / (x + abs_y);
    angle = oneqtr_pi;
  }

  angle += (0.1963f * r * r - 0.9817f) * r;
  return (y < 0.0f) ? -angle : angle;
}

static float wrap_180(float v)
{
  while (v > 180.0f)
  {
    v -= 360.0f;
  }
  while (v < -180.0f)
  {
    v += 360.0f;
  }
  return v;
}

void EstimatorAttitude_Init(void)
{
  s_attitude.roll_deg = 0.0f;
  s_attitude.pitch_deg = 0.0f;
  s_attitude.yaw_deg = 0.0f;
  s_attitude.healthy = false;
  s_attitude.timestamp_ms = 0U;
}

void EstimatorAttitude_Update(const imu_sample_t *imu, float dt_s, attitude_t *out)
{
  if ((imu == NULL) || (out == NULL) || !imu->healthy || (dt_s <= 0.0f))
  {
    if (out != NULL)
    {
      *out = s_attitude;
      out->healthy = false;
    }
    return;
  }

  float acc_roll = atan2_approx_rad(imu->accel_g[1], imu->accel_g[2]) * 57.2957795f;
  float acc_pitch = -atan2_approx_rad(imu->accel_g[0], absf_local(imu->accel_g[2]) + (0.5f * absf_local(imu->accel_g[1]))) * 57.2957795f;

  s_attitude.roll_deg = (0.98f * (s_attitude.roll_deg + imu->gyro_dps[0] * dt_s)) + (0.02f * acc_roll);
  s_attitude.pitch_deg = (0.98f * (s_attitude.pitch_deg + imu->gyro_dps[1] * dt_s)) + (0.02f * acc_pitch);
  s_attitude.yaw_deg = wrap_180(s_attitude.yaw_deg + imu->gyro_dps[2] * dt_s);
  s_attitude.healthy = true;
  s_attitude.timestamp_ms = imu->timestamp_ms;
  *out = s_attitude;
  out->roll_deg -= s_roll_trim_deg;
  out->pitch_deg -= s_pitch_trim_deg;
}

void EstimatorAttitude_SetTrim(float roll_deg, float pitch_deg)
{
  s_roll_trim_deg = roll_deg;
  s_pitch_trim_deg = pitch_deg;
}

void EstimatorAttitude_GetTrim(float *roll_deg, float *pitch_deg)
{
  if (roll_deg != NULL)
  {
    *roll_deg = s_roll_trim_deg;
  }
  if (pitch_deg != NULL)
  {
    *pitch_deg = s_pitch_trim_deg;
  }
}
