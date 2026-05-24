#include "mpu6050.h"

#include <string.h>

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
  ok = ok && write_reg(MPU6050_SMPLRT_DIV, 0x00U);
  ok = ok && write_reg(MPU6050_CONFIG, 0x03U);
  ok = ok && write_reg(MPU6050_GYRO_CONFIG, 0x08U);
  ok = ok && write_reg(MPU6050_ACCEL_CONFIG, 0x08U);

  s_healthy = ok;
  return ok;
}

bool Mpu6050_Read(imu_sample_t *out)
{
  uint8_t data[14];
  if (out == NULL)
  {
    return false;
  }
  memset(out, 0, sizeof(*out));

  if (!read_regs(MPU6050_ACCEL_XOUT_H, data, sizeof(data)))
  {
    s_healthy = false;
    out->healthy = false;
    out->timestamp_ms = HAL_GetTick();
    return false;
  }

  int16_t ax = be16(&data[0]);
  int16_t ay = be16(&data[2]);
  int16_t az = be16(&data[4]);
  int16_t temp = be16(&data[6]);
  int16_t gx = be16(&data[8]);
  int16_t gy = be16(&data[10]);
  int16_t gz = be16(&data[12]);

  out->accel_g[0] = (float)ax / 8192.0f;
  out->accel_g[1] = (float)ay / 8192.0f;
  out->accel_g[2] = (float)az / 8192.0f;
  out->gyro_dps[0] = (float)gx / 65.5f;
  out->gyro_dps[1] = (float)gy / 65.5f;
  out->gyro_dps[2] = (float)gz / 65.5f;
  out->temp_centi_c = (int16_t)((((int32_t)temp * 100) / 340) + 3653);
  out->timestamp_ms = HAL_GetTick();
  out->healthy = true;
  s_healthy = true;
  return true;
}

bool Mpu6050_IsHealthy(void)
{
  return s_healthy;
}
