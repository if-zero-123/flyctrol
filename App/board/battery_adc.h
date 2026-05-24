#ifndef APP_BATTERY_ADC_H
#define APP_BATTERY_ADC_H

#include <stdbool.h>
#include "flight_types.h"

void BatteryAdc_Init(void);
bool BatteryAdc_Read(battery_status_t *out);

#endif /* APP_BATTERY_ADC_H */
