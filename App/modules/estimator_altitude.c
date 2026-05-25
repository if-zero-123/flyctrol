#include "estimator_altitude.h"

#include <stddef.h>

#include "board_config.h"

static bool s_has_baseline;
static bool s_has_velocity;
static int32_t s_baseline_pa;
static float s_filtered_cm;
static float s_velocity_cms;
static int32_t s_last_altitude_cm;
static uint32_t s_last_timestamp_ms;

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

void EstimatorAltitude_Init(void)
{
  s_has_baseline = false;
  s_has_velocity = false;
  s_baseline_pa = 101325;
  s_filtered_cm = 0.0f;
  s_velocity_cms = 0.0f;
  s_last_altitude_cm = 0;
  s_last_timestamp_ms = 0U;
}

void EstimatorAltitude_Update(const baro_sample_t *baro, baro_sample_t *out)
{
  if ((baro == NULL) || (out == NULL))
  {
    return;
  }

  *out = *baro;
  if (!baro->healthy || (baro->pressure_pa <= 0))
  {
    out->healthy = false;
    return;
  }

  if (!s_has_baseline)
  {
    s_baseline_pa = baro->pressure_pa;
    s_filtered_cm = 0.0f;
    s_velocity_cms = 0.0f;
    s_last_altitude_cm = 0;
    s_last_timestamp_ms = baro->timestamp_ms;
    s_has_velocity = false;
    s_has_baseline = true;
  }

  int32_t raw_cm = ((s_baseline_pa - baro->pressure_pa) * 25) / 3;
  float dt_s = 0.025f;
  if ((s_last_timestamp_ms != 0U) && (baro->timestamp_ms > s_last_timestamp_ms))
  {
    dt_s = (float)(baro->timestamp_ms - s_last_timestamp_ms) / 1000.0f;
    dt_s = clampf_local(dt_s, 0.010f, 0.150f);
  }

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
    s_velocity_cms += pt1_alpha(BOARD_BARO_VEL_LPF_HZ, dt_s) * (raw_vel - s_velocity_cms);
  }

  s_last_altitude_cm = alt_cm;
  s_last_timestamp_ms = baro->timestamp_ms;
  out->altitude_cm = alt_cm;
  out->velocity_cms = float_to_i16(s_velocity_cms);
}
