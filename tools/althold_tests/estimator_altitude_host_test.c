#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "estimator_altitude.h"
#include "board_config.h"

flight_status_t Topic_GetStatus(void)
{
  flight_status_t status = {0};
  status.armed = false;
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

static void test_imu_prediction_is_disabled_for_v1_baro_only_velocity(void)
{
  imu_sample_t imu = {0};
  attitude_t attitude = {0};
  baro_sample_t filtered;

  EstimatorAltitude_Init();
  prime_baseline();

  imu.accel_g[2] = 1.30f;
  imu.healthy = true;
  attitude.healthy = true;
  for (uint16_t i = 0U; i < 100U; i++)
  {
    EstimatorAltitude_PredictImu(&imu, &attitude, 0.002f);
  }

  baro_sample_t baro = make_baro(101325, 450U);
  EstimatorAltitude_Update(&baro, &filtered);
  assert(abs(filtered.velocity_cms) <= BOARD_BARO_VEL_DEADBAND_CMS);
  assert(abs((int)filtered.altitude_cm) <= BOARD_ALT_HOLD_DB_CM);
}

int main(void)
{
  test_imu_prediction_is_disabled_for_v1_baro_only_velocity();
  puts("estimator_altitude_host_test: PASS");
  return 0;
}
