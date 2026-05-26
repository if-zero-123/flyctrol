#include "mpu6050.h"

#if defined(__GNUC__)
#pragma GCC optimize ("Os")
#endif

#include <string.h>

#include "board_config.h"
#include "i2c_bus.h"
#include "main.h"

#define MPU6050_ADDR        (0x68U << 1)
#define MPU6050_WHO_AM_I    0x75U
#define MPU6050_PWR_MGMT_1  0x6BU
#define MPU6050_SMPLRT_DIV  0x19U
#define MPU6050_CONFIG      0x1AU
#define MPU6050_GYRO_CONFIG 0x1BU
#define MPU6050_ACCEL_CONFIG 0x1CU
#define MPU6050_ACCEL_XOUT_H 0x3BU

static bool s_healthy;
static float s_gyro_bias_dps[3];
static float s_accel_bias_g[3];
static uint32_t s_read_ok_count;
static uint32_t s_read_fail_count;
static bool s_filter_ready;
static float s_accel_filtered_g[3];
static float s_gyro_filtered_dps[3];

static float pt1_alpha(float cutoff_hz)
{
  const float pi = 3.14159265f;
  const float dt_s = 1.0f / (float)BOARD_CONTROL_LOOP_HZ;
  if (cutoff_hz <= 0.0f)
  {
    return 1.0f;
  }
  float rc = 1.0f / (2.0f * pi * cutoff_hz);
  return dt_s / (dt_s + rc);
}

static bool write_reg(uint8_t reg, uint8_t value)
{
  if (!I2cBus_Lock(5U))
  {
    return false;
  }
  bool ok = HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, 20U) == HAL_OK;
  I2cBus_Unlock();
  return ok;
}

static bool read_reg(uint8_t reg, uint8_t *value)
{
  if (!I2cBus_Lock(5U))
  {
    return false;
  }
  bool ok = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1U, 20U) == HAL_OK;
  I2cBus_Unlock();
  return ok;
}

static bool read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (!I2cBus_Lock(5U))
  {
    return false;
  }
  bool ok = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 20U) == HAL_OK;
  I2cBus_Unlock();
  return ok;
}

static int16_t be16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] << 8 | p[1]);
}

static bool read_raw(int16_t *ax,
                     int16_t *ay,
                     int16_t *az,
                     int16_t *temp,
                     int16_t *gx,
                     int16_t *gy,
                     int16_t *gz)
{
  uint8_t data[14];
  if (!read_regs(MPU6050_ACCEL_XOUT_H, data, sizeof(data)))
  {
    return false;
  }

  if (ax != NULL) { *ax = be16(&data[0]); }
  if (ay != NULL) { *ay = be16(&data[2]); }
  if (az != NULL) { *az = be16(&data[4]); }
  if (temp != NULL) { *temp = be16(&data[6]); }
  if (gx != NULL) { *gx = be16(&data[8]); }
  if (gy != NULL) { *gy = be16(&data[10]); }
  if (gz != NULL) { *gz = be16(&data[12]); }
  return true;
}

bool Mpu6050_Init(void)
{
  uint8_t who = 0U;
  s_healthy = false;

  if (!read_reg(MPU6050_WHO_AM_I, &who) || (who != 0x68U))
  {
    return false;
  }

  bool ok = true;
  ok = ok && write_reg(MPU6050_PWR_MGMT_1, 0x01U);
  HAL_Delay(10U);
  ok = ok && write_reg(MPU6050_SMPLRT_DIV, 0x01U);
  ok = ok && write_reg(MPU6050_CONFIG, 0x02U);
  ok = ok && write_reg(MPU6050_GYRO_CONFIG, 0x08U);
  ok = ok && write_reg(MPU6050_ACCEL_CONFIG, 0x08U);

  s_healthy = ok;
  return ok;
}

bool Mpu6050_CalibrateGyro(uint16_t samples)
{
  if (samples < 16U)
  {
    samples = 16U;
  }

  int64_t gx_sum = 0;
  int64_t gy_sum = 0;
  int64_t gz_sum = 0;
  uint16_t ok_count = 0U;

  for (uint16_t i = 0U; i < samples; i++)
  {
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    if (read_raw(NULL, NULL, NULL, NULL, &gx, &gy, &gz))
    {
      gx_sum += gx;
      gy_sum += gy;
      gz_sum += gz;
      ok_count++;
    }
    HAL_Delay(2U);
  }

  if (ok_count < (samples / 2U))
  {
    s_healthy = false;
    return false;
  }

  s_gyro_bias_dps[0] = ((float)gx_sum / (float)ok_count) / 65.5f;
  s_gyro_bias_dps[1] = ((float)gy_sum / (float)ok_count) / 65.5f;
  s_gyro_bias_dps[2] = ((float)gz_sum / (float)ok_count) / 65.5f;
  s_filter_ready = false;
  s_healthy = true;
  return true;
}

bool Mpu6050_CalibrateAccel(uint16_t samples)
{
  if (samples < 16U)
  {
    samples = 16U;
  }

  int64_t ax_sum = 0;
  int64_t ay_sum = 0;
  int64_t az_sum = 0;
  uint16_t ok_count = 0U;

  for (uint16_t i = 0U; i < samples; i++)
  {
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    if (read_raw(&ax, &ay, &az, NULL, NULL, NULL, NULL))
    {
      ax_sum += ax;
      ay_sum += ay;
      az_sum += az;
      ok_count++;
    }
    HAL_Delay(2U);
  }

  if (ok_count < (samples / 2U))
  {
    s_healthy = false;
    return false;
  }

  s_accel_bias_g[0] = ((float)ax_sum / (float)ok_count) / 8192.0f;
  s_accel_bias_g[1] = ((float)ay_sum / (float)ok_count) / 8192.0f;
  s_accel_bias_g[2] = (((float)az_sum / (float)ok_count) / 8192.0f) - 1.0f;
  s_filter_ready = false;
  s_healthy = true;
  return true;
}

bool Mpu6050_CalibrateImu(uint16_t samples)
{
  bool gyro_ok = Mpu6050_CalibrateGyro(samples);
  bool accel_ok = Mpu6050_CalibrateAccel(samples);
  return gyro_ok && accel_ok;
}

void Mpu6050_ResetFilters(void)
{
  s_filter_ready = false;
}

bool Mpu6050_Read(imu_sample_t *out)
{
  if (out == NULL)
  {
    return false;
  }
  memset(out, 0, sizeof(*out));

  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t temp = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
  if (!read_raw(&ax, &ay, &az, &temp, &gx, &gy, &gz))
  {
    s_healthy = false;
    s_read_fail_count++;
    out->healthy = false;
    out->timestamp_ms = HAL_GetTick();
    return false;
  }

  out->accel_g[0] = ((float)ax / 8192.0f) - s_accel_bias_g[0];
  out->accel_g[1] = ((float)ay / 8192.0f) - s_accel_bias_g[1];
  out->accel_g[2] = ((float)az / 8192.0f) - s_accel_bias_g[2];
  out->gyro_dps[0] = ((float)gx / 65.5f) - s_gyro_bias_dps[0];
  out->gyro_dps[1] = ((float)gy / 65.5f) - s_gyro_bias_dps[1];
  out->gyro_dps[2] = ((float)gz / 65.5f) - s_gyro_bias_dps[2];
  if (!s_filter_ready)
  {
    for (uint8_t i = 0U; i < 3U; i++)
    {
      s_accel_filtered_g[i] = out->accel_g[i];
      s_gyro_filtered_dps[i] = out->gyro_dps[i];
    }
    s_filter_ready = true;
  }
  else
  {
    float accel_alpha = pt1_alpha(BOARD_IMU_ACCEL_LPF_HZ);
    float gyro_alpha = pt1_alpha(BOARD_IMU_GYRO_LPF_HZ);
    for (uint8_t i = 0U; i < 3U; i++)
    {
      s_accel_filtered_g[i] += accel_alpha * (out->accel_g[i] - s_accel_filtered_g[i]);
      s_gyro_filtered_dps[i] += gyro_alpha * (out->gyro_dps[i] - s_gyro_filtered_dps[i]);
    }
  }
  for (uint8_t i = 0U; i < 3U; i++)
  {
    out->accel_g[i] = s_accel_filtered_g[i];
    out->gyro_dps[i] = s_gyro_filtered_dps[i];
  }
  out->temp_centi_c = (int16_t)((((int32_t)temp * 100) / 340) + 3653);
  out->timestamp_ms = HAL_GetTick();
  out->healthy = true;
  s_healthy = true;
  s_read_ok_count++;
  return true;
}

bool Mpu6050_IsHealthy(void)
{
  return s_healthy;
}

void Mpu6050_GetReadStats(uint32_t *ok_count, uint32_t *fail_count)
{
  if (ok_count != NULL)
  {
    *ok_count = s_read_ok_count;
  }
  if (fail_count != NULL)
  {
    *fail_count = s_read_fail_count;
  }
}
