#ifndef APP_CONTROLLER_ALTITUDE_H
#define APP_CONTROLLER_ALTITUDE_H

#include "flight_types.h"

void ControllerAltitude_Init(void);
void ControllerAltitude_Reset(void);
bool ControllerAltitude_IsActive(void);
int16_t ControllerAltitude_Update(const baro_sample_t *baro,
                                  const control_setpoint_t *setpoint,
                                  bool active,
                                  float dt_s);

typedef struct {
  bool active;
  bool velocity_control;
  int32_t hold_altitude_cm;
  int16_t altitude_error_cm;
  int16_t velocity_cms;
  int16_t target_velocity_cms;
  int16_t throttle_base_permille;
  int16_t correction_permille;
  int16_t output_permille;
} controller_altitude_debug_t;

void ControllerAltitude_GetDebug(controller_altitude_debug_t *out);

#endif /* APP_CONTROLLER_ALTITUDE_H */
