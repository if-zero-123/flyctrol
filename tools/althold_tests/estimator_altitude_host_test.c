#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "estimator_altitude.h"
#include "board_config.h"

static bool s_stub_armed;

flight_status_t Topic_GetStatus(void)
{
  flight_status_t status = {0};
  status.armed = s_stub_armed;
  return status;
}

void EstimatorAttitude_GetGravityVector(float gravity_body[3])
{
  gravity_body[0] = 0.0f;
  gravity_body[1] = 0.0f;
  gravity_body[2] = 1.0f;
}

static baro_sample_t make_baro(int32_t pressure_pa, uint32_t timestamp_ms)
{
  baro_sample_t baro = {0};
  baro.pressure_pa = pressure_pa;
  baro.temperature_centi_c = 2500;
  baro.healthy = true;
  baro.timestamp_ms = timestamp_ms;
  return baro;
}

static void prime_baseline(void)
{
  baro_sample_t filtered;
  for (uint8_t i = 0U; i < BOARD_BARO_BASELINE_SAMPLES; i++)
  {
    baro_sample_t baro = make_baro(101325, 25U * (uint32_t)(i + 1U));
    EstimatorAltitude_Update(&baro, &filtered);
  }
  assert(filtered.healthy);
}

static void test_imu_prediction_moves_fused_altitude_between_baro_samples(void)
{
  imu_sample_t imu = {0};
  attitude_t attitude = {0};
  baro_sample_t filtered;

  EstimatorAltitude_Init();
  prime_baseline();
  s_stub_armed = true;

  imu.accel_g[2] = 1.30f;
  imu.healthy = true;
  attitude.healthy = true;
  for (uint16_t i = 0U; i < 100U; i++)
  {
    EstimatorAltitude_PredictImu(&imu, &attitude, 0.002f);
  }

  baro_sample_t baro = make_baro(101325, 450U);
  EstimatorAltitude_Update(&baro, &filtered);
  assert(filtered.velocity_cms > 15);
  assert(filtered.altitude_cm > 3);
  s_stub_armed = false;
}

static void test_disarmed_pressure_warmup_does_not_walk_relative_altitude(void)
{
  baro_sample_t filtered;

  EstimatorAltitude_Init();
  s_stub_armed = false;
  prime_baseline();

  for (uint8_t i = 0U; i < 80U; i++)
  {
    int32_t pressure = 101325 + ((int32_t)i * 5);
    baro_sample_t baro = make_baro(pressure, 425U + (25U * (uint32_t)i));
    EstimatorAltitude_Update(&baro, &filtered);
  }

  assert(abs(filtered.altitude_cm) <= 20);
  assert(abs(filtered.velocity_cms) <= 20);
}

int main(void)
{
  test_imu_prediction_moves_fused_altitude_between_baro_samples();
  test_disarmed_pressure_warmup_does_not_walk_relative_altitude();
  puts("estimator_altitude_host_test: PASS");
  return 0;
}
