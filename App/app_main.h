#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdbool.h>

void App_Start(void);
void App_SetTelemetryLog(bool enabled);
bool App_GetTelemetryLog(void);
void App_SetHeartbeat(bool enabled);
bool App_GetHeartbeat(void);

#endif /* APP_MAIN_H */
