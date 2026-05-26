#include "led.h"

#include "board_time.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

static uint32_t led_time_ms(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    return (uint32_t)xTaskGetTickCount() * (uint32_t)portTICK_PERIOD_MS;
  }
  return BoardTime_Millis();
}

void Led_Init(void)
{
  Led_Set(LED_GREEN, false);
  Led_Set(LED_BLUE, false);
}

void Led_Set(led_id_t led, bool on)
{
  GPIO_TypeDef *port = (led == LED_GREEN) ? LED1_GREEN_GPIO_Port : LED2_BLUE_GPIO_Port;
  uint16_t pin = (led == LED_GREEN) ? LED1_GREEN_Pin : LED2_BLUE_Pin;
  HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Led_Toggle(led_id_t led)
{
  GPIO_TypeDef *port = (led == LED_GREEN) ? LED1_GREEN_GPIO_Port : LED2_BLUE_GPIO_Port;
  uint16_t pin = (led == LED_GREEN) ? LED1_GREEN_Pin : LED2_BLUE_Pin;
  HAL_GPIO_TogglePin(port, pin);
}

void Led_UpdateStatus(const flight_status_t *status)
{
  uint32_t now = led_time_ms();
  bool fast_blink = ((now / 100U) & 1U) != 0U;
  bool slow_blink = ((now / 500U) & 1U) != 0U;

  if (status == NULL)
  {
    Led_Set(LED_GREEN, slow_blink);
    Led_Set(LED_BLUE, false);
    return;
  }

  Led_Set(LED_GREEN, status->armed ? fast_blink : slow_blink);
  Led_Set(LED_BLUE, status->failsafe ? fast_blink : status->rc_ok);
}
