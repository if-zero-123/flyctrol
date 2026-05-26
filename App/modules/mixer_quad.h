#ifndef APP_MIXER_QUAD_H
#define APP_MIXER_QUAD_H

#include "flight_types.h"

void MixerQuad_SetMotorIdlePermille(uint16_t permille);
uint16_t MixerQuad_GetMotorIdlePermille(void);
void MixerQuad_SetMotorMaxPermille(uint16_t permille);
uint16_t MixerQuad_GetMotorMaxPermille(void);
void MixerQuad_ResetThrottleRamp(void);
void MixerQuad_PrimeThrottleRamp(uint16_t permille);
void MixerQuad_Mix(uint16_t throttle_permille, const control_output_t *control, float motor_out[4]);

#endif /* APP_MIXER_QUAD_H */
