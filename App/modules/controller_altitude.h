#ifndef APP_CONTROLLER_ALTITUDE_H
#define APP_CONTROLLER_ALTITUDE_H

#include "flight_types.h"

void ControllerAltitude_Init(void);
int16_t ControllerAltitude_Update(const baro_sample_t *baro,
                                  const control_setpoint_t *setpoint,
                                  bool active,
                                  float dt_s);

#endif /* APP_CONTROLLER_ALTITUDE_H */
