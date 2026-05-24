#ifndef APP_MOTOR_PWM_H
#define APP_MOTOR_PWM_H

#include <stdint.h>

void MotorPwm_Init(void);
void MotorPwm_Set(uint8_t index, float duty);
void MotorPwm_SetPermille(uint8_t index, uint16_t permille);
void MotorPwm_SetAll(float duty);
void MotorPwm_SetAllPermille(uint16_t permille);
void MotorPwm_Set4(const float duty[4]);
void MotorPwm_GetLast(float duty[4]);

#endif /* APP_MOTOR_PWM_H */
