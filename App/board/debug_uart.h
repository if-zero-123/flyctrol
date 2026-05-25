#ifndef APP_DEBUG_UART_H
#define APP_DEBUG_UART_H

#include <stdbool.h>
#include <stdint.h>

void DebugUart_Init(void);
void DebugUart_Write(const char *text);
void DebugUart_WriteLine(const char *text);
void DebugUart_Printf(const char *fmt, ...);
bool DebugUart_ReadByte(uint8_t *byte, uint32_t timeout_ms);
uint32_t DebugUart_RxDropped(void);

#endif /* APP_DEBUG_UART_H */
