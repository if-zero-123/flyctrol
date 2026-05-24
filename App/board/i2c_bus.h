#ifndef APP_I2C_BUS_H
#define APP_I2C_BUS_H

#include <stdbool.h>
#include <stdint.h>

void I2cBus_Init(void);
bool I2cBus_Lock(uint32_t timeout_ms);
void I2cBus_Unlock(void);

#endif /* APP_I2C_BUS_H */
