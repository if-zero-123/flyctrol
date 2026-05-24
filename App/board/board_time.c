#include "board_time.h"

#include "stm32f1xx_hal.h"

uint32_t BoardTime_Millis(void)
{
  return HAL_GetTick();
}

uint32_t BoardTime_Micros(void)
{
  uint32_t ms = HAL_GetTick();
  uint32_t ticks = SysTick->VAL;
  uint32_t load = SysTick->LOAD + 1U;

  if (load == 0U)
  {
    return ms * 1000U;
  }

  return (ms * 1000U) + ((load - ticks) / (SystemCoreClock / 1000000U));
}
