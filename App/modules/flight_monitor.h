#ifndef APP_FLIGHT_MONITOR_H
#define APP_FLIGHT_MONITOR_H

#include "flight_types.h"

void FlightMonitor_Init(void);
void FlightMonitor_Update(const flight_status_t *status);
void FlightMonitor_Print(void);

#endif /* APP_FLIGHT_MONITOR_H */
