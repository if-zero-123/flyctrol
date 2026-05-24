#ifndef APP_COMMANDER_H
#define APP_COMMANDER_H

#include "flight_types.h"

void Commander_BuildSetpoint(const app_rc_t *rc, control_setpoint_t *out);

#endif /* APP_COMMANDER_H */
