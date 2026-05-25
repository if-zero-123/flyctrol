#include "mixer_quad.h"

static float clamp_unit(float v)
{
  if (v < 0.0f)
  {
    return 0.0f;
  }
  if (v > 1.0f)
  {
    return 1.0f;
  }
  return v;
}

void MixerQuad_Mix(uint16_t throttle_permille, const control_output_t *control, float motor_out[4])
{
  if ((control == 0) || (motor_out == 0))
  {
    return;
  }

  int32_t throttle = (int32_t)throttle_permille + control->altitude_permille;
  if (throttle < 0)
  {
    throttle = 0;
  }
  if (throttle > 1000)
  {
    throttle = 1000;
  }

  float t = (float)throttle / 1000.0f;
  float r = control->roll;
  float p = control->pitch;
  float y = control->yaw;

  motor_out[0] = clamp_unit(t - r - p + y);
  motor_out[1] = clamp_unit(t - r + p - y);
  motor_out[2] = clamp_unit(t + r - p - y);
  motor_out[3] = clamp_unit(t + r + p + y);
}
