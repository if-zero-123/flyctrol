#include "estimator_altitude.h"

#include <stddef.h>

#include "board_config.h"
#include "estimator_attitude.h"
#include "topic.h"

static bool s_has_baseline;
static bool s_has_velocity;
static int32_t s_baseline_pa;
static int32_t s_baseline_accum_pa;
static uint8_t s_baseline_count;
static uint8_t s_spike_count;
static int32_t s_last_baseline_pressure_pa;
static float s_filtered_cm;
static float s_fused_altitude_cm;
static float s_velocity_cms;
static float s_accel_cms2;
static int32_t s_last_altitude_cm;
static uint32_t s_last_timestamp_ms;

static int32_t abs_i32(int32_t v)
{
  return (v < 0) ? -v : v;
}

static float absf_local(float v)
{
  return (v < 0.0f) ? -v : v;
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

static bool pressure_plausible(int32_t pressure_pa)
{
  return (pressure_pa >= BOARD_BARO_PRESSURE_MIN_PA) &&
         (pressure_pa <= BOARD_BARO_PRESSURE_MAX_PA);
}

static void restart_baseline(int32_t pressure_pa, uint32_t timestamp_ms)
{
  s_has_baseline = false;
  s_has_velocity = false;
  s_baseline_pa = pressure_pa;
  s_baseline_accum_pa = pressure_pa;
  s_baseline_count = 1U;
  s_spike_count = 0U;
  s_last_baseline_pressure_pa = pressure_pa;
  s_filtered_cm = 0.0f;
  s_fused_altitude_cm = 0.0f;
  s_velocity_cms = 0.0f;
  s_accel_cms2 = 0.0f;
  s_last_altitude_cm = 0;
  s_last_timestamp_ms = timestamp_ms;
}

void EstimatorAltitude_Init(void)
{
  s_has_baseline = false;
  s_has_velocity = false;
  s_baseline_pa = 101325;
  s_baseline_accum_pa = 0;
  s_baseline_count = 0U;
  s_spike_count = 0U;
  s_last_baseline_pressure_pa = 0;
  s_filtered_cm = 0.0f;
  s_fused_altitude_cm = 0.0f;
  s_velocity_cms = 0.0f;
  s_accel_cms2 = 0.0f;
  s_last_altitude_cm = 0;
  s_last_timestamp_ms = 0U;
}

void EstimatorAltitude_ResetDynamic(void)
{
  s_has_velocity = false;
  s_spike_count = 0U;
  s_fused_altitude_cm = s_filtered_cm;
  s_velocity_cms = 0.0f;
  s_accel_cms2 = 0.0f;
  s_last_altitude_cm = (int32_t)s_filtered_cm;
}

void EstimatorAltitude_PredictImu(const imu_sample_t *imu,
                                  const attitude_t *attitude,
                                  float dt_s)
{
#if BOARD_ALT_IMU_PREDICT_ENABLE
  if ((imu == NULL) || (attitude == NULL) || !s_has_baseline ||
      !imu->healthy || !attitude->healthy || (dt_s <= 0.0f))
  {
    return;
  }

  dt_s = clampf_local(dt_s, 0.001f, 0.010f);
  float gravity_body[3];
  EstimatorAttitude_GetGravityVector(gravity_body);
  float vertical_g = (imu->accel_g[0] * gravity_body[0]) +
                     (imu->accel_g[1] * gravity_body[1]) +
                     (imu->accel_g[2] * gravity_body[2]);
  float accel_cms2 = (vertical_g - 1.0f) * 980.665f;

  if (absf_local(accel_cms2) < BOARD_ALT_IMU_ACC_DEADBAND_CMS2)
  {
    accel_cms2 = 0.0f;
  }
  accel_cms2 = clampf_local(accel_cms2,
                            -BOARD_ALT_IMU_ACC_LIMIT_CMS2,
                            BOARD_ALT_IMU_ACC_LIMIT_CMS2);
  s_accel_cms2 += pt1_alpha(BOARD_ALT_IMU_ACC_LPF_HZ, dt_s) * (accel_cms2 - s_accel_cms2);
  s_velocity_cms += s_accel_cms2 * dt_s;
  s_velocity_cms = clampf_local(s_velocity_cms,
                                -(float)BOARD_BARO_VEL_LIMIT_CMS,
                                (float)BOARD_BARO_VEL_LIMIT_CMS);
  s_fused_altitude_cm += s_velocity_cms * dt_s;
#else
  (void)imu;
  (void)attitude;
  (void)dt_s;
#endif
}

void EstimatorAltitude_Update(const baro_sample_t *baro, baro_sample_t *out)
{
  if ((baro == NULL) || (out == NULL))
  {
    return;
  }

  *out = *baro;
  if (!baro->healthy || !pressure_plausible(baro->pressure_pa))
  {
    out->healthy = false;
    return;
  }

  if (!s_has_baseline)
  {
    if ((s_baseline_count == 0U) ||
        (abs_i32(baro->pressure_pa - s_last_baseline_pressure_pa) > BOARD_BARO_BASELINE_STEP_MAX_PA))
    {
      restart_baseline(baro->pressure_pa, baro->timestamp_ms);
    }
    else
    {
      s_baseline_accum_pa += baro->pressure_pa;
      s_baseline_count++;
      s_last_baseline_pressure_pa = baro->pressure_pa;
    }

    out->altitude_cm = 0;
    out->velocity_cms = 0;
    out->healthy = false;
    if (s_baseline_count < BOARD_BARO_BASELINE_SAMPLES)
    {
      return;
    }

    s_baseline_pa = s_baseline_accum_pa / (int32_t)s_baseline_count;
    s_filtered_cm = 0.0f;
    s_fused_altitude_cm = 0.0f;
    s_velocity_cms = 0.0f;
    s_accel_cms2 = 0.0f;
    s_last_altitude_cm = 0;
    s_last_timestamp_ms = baro->timestamp_ms;
    s_has_velocity = false;
    s_has_baseline = true;
    out->healthy = true;
  }

  int32_t raw_cm = ((s_baseline_pa - baro->pressure_pa) * 25) / 3;
  float dt_s = 0.025f;
  if ((s_last_timestamp_ms != 0U) && (baro->timestamp_ms > s_last_timestamp_ms))
  {
    dt_s = (float)(baro->timestamp_ms - s_last_timestamp_ms) / 1000.0f;
    dt_s = clampf_local(dt_s, 0.010f, 0.150f);
  }

  if (s_has_velocity && (absf_local((float)raw_cm - s_filtered_cm) > (float)BOARD_BARO_ALT_SPIKE_REJECT_CM))
  {
    if (s_spike_count < 8U)
    {
      s_spike_count++;
    }
    if ((s_spike_count >= 8U) && !Topic_GetStatus().armed)
    {
      restart_baseline(baro->pressure_pa, baro->timestamp_ms);
      out->altitude_cm = 0;
      out->velocity_cms = 0;
      out->healthy = false;
      return;
    }
    s_velocity_cms *= 0.80f;
    s_last_timestamp_ms = baro->timestamp_ms;
    s_fused_altitude_cm += BOARD_ALT_BARO_POS_BLEND * (s_filtered_cm - s_fused_altitude_cm);
    out->altitude_cm = (int32_t)s_fused_altitude_cm;
    out->velocity_cms = float_to_i16(s_velocity_cms);
    return;
  }
  s_spike_count = 0U;

  s_filtered_cm += pt1_alpha(BOARD_BARO_ALT_LPF_HZ, dt_s) * ((float)raw_cm - s_filtered_cm);
  int32_t alt_cm = (int32_t)s_filtered_cm;
  if (!s_has_velocity)
  {
    s_velocity_cms = 0.0f;
    s_has_velocity = true;
  }
  else
  {
    float raw_vel = (float)(alt_cm - s_last_altitude_cm) / dt_s;
    raw_vel = clampf_local(raw_vel,
                           -(float)BOARD_BARO_VEL_LIMIT_CMS,
                           (float)BOARD_BARO_VEL_LIMIT_CMS);
    if (absf_local(raw_vel) < (float)BOARD_BARO_VEL_DEADBAND_CMS)
    {
      raw_vel = 0.0f;
    }
    float baro_velocity = s_velocity_cms + pt1_alpha(BOARD_BARO_VEL_LPF_HZ, dt_s) * (raw_vel - s_velocity_cms);
    s_velocity_cms += BOARD_ALT_BARO_VEL_BLEND * (baro_velocity - s_velocity_cms);
  }

  s_fused_altitude_cm += BOARD_ALT_BARO_POS_BLEND * ((float)alt_cm - s_fused_altitude_cm);
  s_last_altitude_cm = alt_cm;
  s_last_timestamp_ms = baro->timestamp_ms;
  out->altitude_cm = (int32_t)s_fused_altitude_cm;
  out->velocity_cms = float_to_i16(s_velocity_cms);
}
