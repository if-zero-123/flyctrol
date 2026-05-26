#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "board_config.h"
#include "mixer_quad.h"

static void test_feedback_reports_motor_range_and_saturation(void)
{
  control_output_t control = {0};
  float motor[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  mixer_feedback_t feedback = {0};

  MixerQuad_SetMotorIdlePermille(BOARD_MOTOR_IDLE_PERMILLE);
  MixerQuad_SetMotorMaxPermille(BOARD_MOTOR_MAX_PERMILLE);
  MixerQuad_ResetThrottleRamp();

  control.roll = 0.30f;
  control.pitch = -0.25f;
  control.yaw = 0.10f;
  control.altitude_permille = 100;

  MixerQuad_MixWithFeedback(BOARD_MOTOR_MAX_PERMILLE - 30U, &control, motor, &feedback);

  assert(feedback.motor_max_permille <= BOARD_MOTOR_MAX_PERMILLE);
  assert(feedback.motor_min_permille >= BOARD_MOTOR_IDLE_PERMILLE);
  assert(feedback.motor_spread_permille > 0U);
  assert(feedback.saturated_high || feedback.attitude_scaled);
  assert(feedback.attitude_scale_permille <= 1000U);
}

int main(void)
{
  test_feedback_reports_motor_range_and_saturation();
  puts("mixer_quad_host_test: PASS");
  return 0;
}
