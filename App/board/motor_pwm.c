#include "motor_pwm.h"

#include <stdbool.h>

#include "board_config.h"
#include "flight_types.h"
#include "main.h"

static float s_last[APP_MOTOR_COUNT];
static bool s_started;

static uint32_t duty_to_ccr(float duty)
{
  if (duty < 0.0f)
  {
    duty = 0.0f;
  }
  if (duty > 1.0f)
  {
    duty = 1.0f;
  }
  return (uint32_t)(duty * (float)BOARD_PWM_FULL_COUNTS);
}

static void write_motor(uint8_t index, uint32_t ccr)
{
  switch (index)
  {
    case 0:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
      break;
    case 1:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr);
      break;
    case 2:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
      break;
    case 3:
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr);
      break;
    default:
      break;
  }
}

void MotorPwm_Init(void)
{
  MotorPwm_SetAll(0.0f);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  s_started = true;
  MotorPwm_SetAll(0.0f);
}

void MotorPwm_Set(uint8_t index, float duty)
{
  if (index >= APP_MOTOR_COUNT)
  {
    return;
  }
  if (duty < 0.0f)
  {
    duty = 0.0f;
  }
  if (duty > 1.0f)
  {
    duty = 1.0f;
  }
  s_last[index] = duty;
  if (s_started)
  {
    write_motor(index, duty_to_ccr(duty));
  }
}

void MotorPwm_SetPermille(uint8_t index, uint16_t permille)
{
  if (permille > 1000U)
  {
    permille = 1000U;
  }
  MotorPwm_Set(index, (float)permille / 1000.0f);
}

void MotorPwm_SetAll(float duty)
{
  for (uint8_t i = 0; i < APP_MOTOR_COUNT; i++)
  {
    MotorPwm_Set(i, duty);
  }
}

void MotorPwm_SetAllPermille(uint16_t permille)
{
  for (uint8_t i = 0; i < APP_MOTOR_COUNT; i++)
  {
    MotorPwm_SetPermille(i, permille);
  }
}

void MotorPwm_Set4(const float duty[4])
{
  if (duty == NULL)
  {
    MotorPwm_SetAll(0.0f);
    return;
  }
  for (uint8_t i = 0; i < APP_MOTOR_COUNT; i++)
  {
    MotorPwm_Set(i, duty[i]);
  }
}

void MotorPwm_GetLast(float duty[4])
{
  if (duty == NULL)
  {
    return;
  }
  for (uint8_t i = 0; i < APP_MOTOR_COUNT; i++)
  {
    duty[i] = s_last[i];
  }
}
