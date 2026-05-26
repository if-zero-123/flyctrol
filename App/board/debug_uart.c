#include "debug_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "main.h"

#define DEBUG_UART_RX_BUF_SIZE 128U

static SemaphoreHandle_t s_tx_mutex;
static char s_printf_buf[192];
static uint8_t s_rx_irq_byte;
static volatile uint8_t s_rx_buf[DEBUG_UART_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_dropped;

static void start_rx_it(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &s_rx_irq_byte, 1U);
}

void DebugUart_Init(void)
{
  s_tx_mutex = xSemaphoreCreateMutex();
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_dropped = 0U;
  start_rx_it();
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

  bool use_lock = (s_tx_mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
  bool locked = false;
  if (use_lock)
  {
    locked = xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(100)) == pdTRUE;
    if (!locked)
    {
      return;
    }
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, 200U);

  if (locked)
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
  bool use_lock = (s_tx_mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
  bool locked = false;
  if (use_lock)
  {
    locked = xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(100)) == pdTRUE;
    if (!locked)
    {
      return;
    }
  }

  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(s_printf_buf, sizeof(s_printf_buf), fmt, args);
  va_end(args);

  if (n > 0)
  {
    s_printf_buf[sizeof(s_printf_buf) - 1U] = '\0';
    (void)HAL_UART_Transmit(&huart1,
                            (uint8_t *)s_printf_buf,
                            (uint16_t)strlen(s_printf_buf),
                            200U);
  }

  if (locked)
  {
    (void)xSemaphoreGive(s_tx_mutex);
  }
}

bool DebugUart_ReadByte(uint8_t *byte, uint32_t timeout_ms)
{
  if (byte == NULL)
  {
    return false;
  }

  TickType_t start = xTaskGetTickCount();
  TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms);
  for (;;)
  {
    __disable_irq();
    if (s_rx_tail != s_rx_head)
    {
      *byte = s_rx_buf[s_rx_tail];
      s_rx_tail = (uint16_t)((s_rx_tail + 1U) % DEBUG_UART_RX_BUF_SIZE);
      __enable_irq();
      return true;
    }
    __enable_irq();

    if (timeout_ms == 0U)
    {
      return false;
    }
    if ((xTaskGetTickCount() - start) >= wait_ticks)
    {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1U));
  }
}

uint32_t DebugUart_RxDropped(void)
{
  return s_rx_dropped;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    uint16_t next = (uint16_t)((s_rx_head + 1U) % DEBUG_UART_RX_BUF_SIZE);
    if (next == s_rx_tail)
    {
      s_rx_dropped++;
    }
    else
    {
      s_rx_buf[s_rx_head] = s_rx_irq_byte;
      s_rx_head = next;
    }
    start_rx_it();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    start_rx_it();
  }
}
