#include "debug_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "main.h"

static SemaphoreHandle_t s_tx_mutex;

void DebugUart_Init(void)
{
  s_tx_mutex = xSemaphoreCreateMutex();
}

void DebugUart_Write(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  size_t len = strlen(text);
  if (len == 0U)
  {
    return;
  }

  bool lock = (s_tx_mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
  if (lock)
  {
    (void)xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(100));
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, 200U);

  if (lock)
  {
    (void)xSemaphoreGive(s_tx_mutex);
  }
}

void DebugUart_WriteLine(const char *text)
{
  DebugUart_Write(text);
  DebugUart_Write("\r\n");
}

void DebugUart_Printf(const char *fmt, ...)
{
  char buf[192];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (n <= 0)
  {
    return;
  }
  buf[sizeof(buf) - 1U] = '\0';
  DebugUart_Write(buf);
}

bool DebugUart_ReadByte(uint8_t *byte, uint32_t timeout_ms)
{
  if (byte == NULL)
  {
    return false;
  }
  return HAL_UART_Receive(&huart1, byte, 1U, timeout_ms) == HAL_OK;
}
