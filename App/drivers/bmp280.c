#include "bmp280.h"

#include <string.h>

#include "i2c_bus.h"
#include "main.h"

#define BMP280_ADDR       (0x76U << 1)
#define BMP280_ID_REG     0xD0U
#define BMP280_RESET_REG  0xE0U
#define BMP280_CTRL_MEAS  0xF4U
#define BMP280_CONFIG     0xF5U
#define BMP280_PRESS_MSB  0xF7U
#define BMP280_CALIB_REG  0x88U
#define BMP280_CONFIG_FAST_FILTER_X4 0x08U
#define BMP280_CTRL_TEMP_X1_PRESS_X8_NORMAL 0x33U

typedef struct {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  int32_t t_fine;
} bmp280_calib_t;

static bmp280_calib_t s_calib;
static bool s_healthy;

static uint16_t le_u16(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t le_s16(const uint8_t *p)
{
  return (int16_t)le_u16(p);
}

static bool write_reg(uint8_t reg, uint8_t value)
{
  if (!I2cBus_Lock(10U))
  {
    return false;
  }
  bool ok = HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, 20U) == HAL_OK;
  I2cBus_Unlock();
  return ok;
}

static bool read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (!I2cBus_Lock(10U))
  {
    return false;
  }
  bool ok = HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 30U) == HAL_OK;
  I2cBus_Unlock();
  return ok;
}

static int32_t compensate_t(int32_t adc_t)
{
  int32_t var1 = ((((adc_t >> 3) - ((int32_t)s_calib.dig_T1 << 1))) * (int32_t)s_calib.dig_T2) >> 11;
  int32_t var2 = (((((adc_t >> 4) - (int32_t)s_calib.dig_T1) * ((adc_t >> 4) - (int32_t)s_calib.dig_T1)) >> 12) *
                  (int32_t)s_calib.dig_T3) >> 14;
  s_calib.t_fine = var1 + var2;
  return (s_calib.t_fine * 5 + 128) >> 8;
}

static uint32_t compensate_p(int32_t adc_p)
{
  int64_t var1 = ((int64_t)s_calib.t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) + ((var1 * (int64_t)s_calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)s_calib.dig_P1) >> 33;

  if (var1 == 0)
  {
    return 0U;
  }

  int64_t p = 1048576 - adc_p;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);
  return (uint32_t)(p >> 8);
}

bool Bmp280_Init(void)
{
  uint8_t id = 0U;
  uint8_t calib[24];
  memset(&s_calib, 0, sizeof(s_calib));
  s_healthy = false;

  if (!read_regs(BMP280_ID_REG, &id, 1U) || (id != 0x58U))
  {
    return false;
  }

  if (!read_regs(BMP280_CALIB_REG, calib, sizeof(calib)))
  {
    return false;
  }

  s_calib.dig_T1 = le_u16(&calib[0]);
  s_calib.dig_T2 = le_s16(&calib[2]);
  s_calib.dig_T3 = le_s16(&calib[4]);
  s_calib.dig_P1 = le_u16(&calib[6]);
  s_calib.dig_P2 = le_s16(&calib[8]);
  s_calib.dig_P3 = le_s16(&calib[10]);
  s_calib.dig_P4 = le_s16(&calib[12]);
  s_calib.dig_P5 = le_s16(&calib[14]);
  s_calib.dig_P6 = le_s16(&calib[16]);
  s_calib.dig_P7 = le_s16(&calib[18]);
  s_calib.dig_P8 = le_s16(&calib[20]);
  s_calib.dig_P9 = le_s16(&calib[22]);

  bool ok = true;
  ok = ok && write_reg(BMP280_RESET_REG, 0xB6U);
  HAL_Delay(5U);
  ok = ok && write_reg(BMP280_CONFIG, BMP280_CONFIG_FAST_FILTER_X4);
  ok = ok && write_reg(BMP280_CTRL_MEAS, BMP280_CTRL_TEMP_X1_PRESS_X8_NORMAL);
  s_healthy = ok;
  return ok;
}

bool Bmp280_Read(baro_sample_t *out)
{
  uint8_t data[6];
  if (out == NULL)
  {
    return false;
  }
  memset(out, 0, sizeof(*out));

  if (!read_regs(BMP280_PRESS_MSB, data, sizeof(data)))
  {
    s_healthy = false;
    out->healthy = false;
    out->timestamp_ms = HAL_GetTick();
    return false;
  }

  int32_t adc_p = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
  int32_t adc_t = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
  int32_t temp = compensate_t(adc_t);
  uint32_t pressure = compensate_p(adc_p);

  out->temperature_centi_c = (int16_t)temp;
  out->pressure_pa = (int32_t)pressure;
  out->altitude_cm = ((101325 - out->pressure_pa) * 25) / 3;
  out->healthy = pressure > 0U;
  out->timestamp_ms = HAL_GetTick();
  s_healthy = out->healthy;
  return out->healthy;
}

bool Bmp280_IsHealthy(void)
{
  return s_healthy;
}
