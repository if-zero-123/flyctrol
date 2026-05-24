#ifndef APP_BMP280_H
#define APP_BMP280_H

#include <stdbool.h>
#include "flight_types.h"

bool Bmp280_Init(void);
bool Bmp280_Read(baro_sample_t *out);
bool Bmp280_IsHealthy(void);

#endif /* APP_BMP280_H */
