#ifndef APP_MIXER_QUAD_H
#define APP_MIXER_QUAD_H

#include "flight_types.h"

void MixerQuad_Mix(uint16_t throttle_permille, const control_output_t *control, float motor_out[4]);

#endif /* APP_MIXER_QUAD_H */
