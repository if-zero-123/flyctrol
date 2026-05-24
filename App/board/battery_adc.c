#include "battery_adc.h"

#include "board_config.h"
#include "main.h"

void BatteryAdc_Init(void)
{
  (void)HAL_ADCEx_Calibration_Start(&hadc1);
}

bool BatteryAdc_Read(battery_status_t *out)
{
  if (out == NULL)
  {
    return false;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return false;
  }
  if (HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return false;
  }

  uint32_t raw = HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  uint32_t vadc_mv = (raw * BOARD_ADC_REF_MV) / BOARD_ADC_MAX_COUNTS;
  uint32_t vbat_mv = (vadc_mv * BOARD_BATTERY_DIVIDER_NUM) / BOARD_BATTERY_DIVIDER_DEN;
  uint32_t percent = 0U;

  if (vbat_mv >= BOARD_BATT_FULL_MV)
  {
    percent = 100U;
  }
  else if (vbat_mv > BOARD_BATT_EMPTY_MV)
  {
    percent = ((vbat_mv - BOARD_BATT_EMPTY_MV) * 100U) / (BOARD_BATT_FULL_MV - BOARD_BATT_EMPTY_MV);
  }

  out->adc_raw = (uint16_t)raw;
  out->voltage_mv = (uint16_t)vbat_mv;
  out->percent = (uint8_t)percent;
  out->low = vbat_mv <= BOARD_BATT_LOW_MV;
  out->critical = vbat_mv <= BOARD_BATT_CRITICAL_MV;
  out->timestamp_ms = HAL_GetTick();
  return true;
}
