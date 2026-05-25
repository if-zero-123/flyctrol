#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include <stdbool.h>
#include <stdint.h>
#include "flight_types.h"

void Safety_Init(void);
void Safety_Update(void);
bool Safety_CanRunMotors(void);
bool Safety_CanMotorTest(void);
void Safety_RequestArm(bool enable);
void Safety_RequestDisarm(void);
void Safety_MotorTestUnlock(uint32_t window_ms);
void Safety_MotorTestBenchUnlock(uint32_t window_ms);
flight_status_t Safety_GetStatus(void);

#endif /* APP_SAFETY_H */
