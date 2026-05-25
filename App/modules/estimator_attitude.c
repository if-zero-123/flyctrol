#include "estimator_attitude.h"

#include <math.h>
#include <stddef.h>

#include "board_config.h"

static attitude_t s_attitude;
static float s_roll_trim_deg;
static float s_pitch_trim_deg;
static float s_q0;
static float s_q1;
static float s_q2;
static float s_q3;
static float s_integral_x;
static float s_integral_y;
static float s_integral_z;

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

static float inv_sqrt(float v)
{
  if (v <= 0.0f)
  {
    return 0.0f;
  }
  return 1.0f / sqrtf(v);
}

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

static void update_euler_from_quat(void)
{
  float q0q0 = s_q0 * s_q0;
  float q1q1 = s_q1 * s_q1;
  float q2q2 = s_q2 * s_q2;
  float q3q3 = s_q3 * s_q3;
  float roll_num = 2.0f * ((s_q0 * s_q1) + (s_q2 * s_q3));
  float roll_den = q0q0 - q1q1 - q2q2 + q3q3;
  float pitch_sin = 2.0f * ((s_q0 * s_q2) - (s_q3 * s_q1));
  float yaw_num = 2.0f * ((s_q0 * s_q3) + (s_q1 * s_q2));
  float yaw_den = q0q0 + q1q1 - q2q2 - q3q3;

  pitch_sin = clampf_local(pitch_sin, -1.0f, 1.0f);
  s_attitude.roll_deg = atan2_approx_rad(roll_num, roll_den) * 57.2957795f;
  s_attitude.pitch_deg = asinf(pitch_sin) * 57.2957795f;
  s_attitude.yaw_deg = wrap_180(atan2_approx_rad(yaw_num, yaw_den) * 57.2957795f);
}

void EstimatorAttitude_Init(void)
{
  s_q0 = 1.0f;
  s_q1 = 0.0f;
  s_q2 = 0.0f;
  s_q3 = 0.0f;
  s_integral_x = 0.0f;
  s_integral_y = 0.0f;
  s_integral_z = 0.0f;
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

  const float deg_to_rad = 0.0174532925f;
  float gx = imu->gyro_dps[0] * deg_to_rad;
  float gy = imu->gyro_dps[1] * deg_to_rad;
  float gz = imu->gyro_dps[2] * deg_to_rad;
  float ax = imu->accel_g[0];
  float ay = imu->accel_g[1];
  float az = imu->accel_g[2];
  float acc_mag_sq = (ax * ax) + (ay * ay) + (az * az);

  if ((acc_mag_sq > BOARD_IMU_ACC_MIN_G_SQ) && (acc_mag_sq < BOARD_IMU_ACC_MAX_G_SQ))
  {
    float recip_norm = inv_sqrt(acc_mag_sq);
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    float half_vx = (s_q1 * s_q3) - (s_q0 * s_q2);
    float half_vy = (s_q0 * s_q1) + (s_q2 * s_q3);
    float half_vz = (s_q0 * s_q0) - 0.5f + (s_q3 * s_q3);
    float half_ex = (ay * half_vz) - (az * half_vy);
    float half_ey = (az * half_vx) - (ax * half_vz);
    float half_ez = (ax * half_vy) - (ay * half_vx);

    if (BOARD_IMU_DCM_KI > 0.0f)
    {
      s_integral_x += BOARD_IMU_DCM_KI * half_ex * dt_s;
      s_integral_y += BOARD_IMU_DCM_KI * half_ey * dt_s;
      s_integral_z += BOARD_IMU_DCM_KI * half_ez * dt_s;
      gx += s_integral_x;
      gy += s_integral_y;
      gz += s_integral_z;
    }

    gx += BOARD_IMU_DCM_KP * half_ex;
    gy += BOARD_IMU_DCM_KP * half_ey;
    gz += BOARD_IMU_DCM_KP * half_ez;
  }
  else
  {
    s_integral_x *= 0.995f;
    s_integral_y *= 0.995f;
    s_integral_z *= 0.995f;
  }

  gx *= 0.5f * dt_s;
  gy *= 0.5f * dt_s;
  gz *= 0.5f * dt_s;

  float qa = s_q0;
  float qb = s_q1;
  float qc = s_q2;
  s_q0 += (-qb * gx - qc * gy - s_q3 * gz);
  s_q1 += (qa * gx + qc * gz - s_q3 * gy);
  s_q2 += (qa * gy - qb * gz + s_q3 * gx);
  s_q3 += (qa * gz + qb * gy - qc * gx);

  float recip_norm = inv_sqrt((s_q0 * s_q0) + (s_q1 * s_q1) + (s_q2 * s_q2) + (s_q3 * s_q3));
  if (recip_norm > 0.0f)
  {
    s_q0 *= recip_norm;
    s_q1 *= recip_norm;
    s_q2 *= recip_norm;
    s_q3 *= recip_norm;
  }

  update_euler_from_quat();
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

void EstimatorAttitude_GetGravityVector(float gravity_body[3])
{
  if (gravity_body == NULL)
  {
    return;
  }

  gravity_body[0] = 2.0f * ((s_q1 * s_q3) - (s_q0 * s_q2));
  gravity_body[1] = 2.0f * ((s_q0 * s_q1) + (s_q2 * s_q3));
  gravity_body[2] = (s_q0 * s_q0) - (s_q1 * s_q1) - (s_q2 * s_q2) + (s_q3 * s_q3);
}
