#include "i2c_bus.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static SemaphoreHandle_t s_i2c_mutex;

void I2cBus_Init(void)
{
  s_i2c_mutex = xSemaphoreCreateMutex();
}

bool I2cBus_Lock(uint32_t timeout_ms)
{
  if ((s_i2c_mutex == NULL) || (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED))
  {
    return true;
  }
  return xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void I2cBus_Unlock(void)
{
  if ((s_i2c_mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
  {
    (void)xSemaphoreGive(s_i2c_mutex);
  }
}
