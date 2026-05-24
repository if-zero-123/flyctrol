#ifndef APP_CRSF_H
#define APP_CRSF_H

#include <stdbool.h>
#include "stm32f1xx_hal.h"

bool Crsf_Init(void);
void Crsf_SetTaskHandle(void *task_handle);
void Crsf_ProcessRx(void);
void Crsf_UpdateLinkState(void);
void Crsf_OnUsartIdleIsr(UART_HandleTypeDef *huart);

#endif /* APP_CRSF_H */
