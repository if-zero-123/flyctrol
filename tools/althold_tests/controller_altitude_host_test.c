#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "controller_altitude.h"
#include "board_config.h"

static int16_t abs_i16_local(int16_t v)
{
  return (v < 0) ? (int16_t)-v : v;
}

static baro_sample_t make_baro(int32_t altitude_cm, int16_t velocity_cms, uint32_t timestamp_ms)
{
  baro_sample_t baro = {0};
  baro.altitude_cm = altitude_cm;
  baro.velocity_cms = velocity_cms;
  baro.pressure_pa = 101325 - ((altitude_cm * 3) / 25);
  baro.healthy = true;
  baro.timestamp_ms = timestamp_ms;
  return baro;
}

static control_setpoint_t make_setpoint(uint16_t throttle_permille)
{
  control_setpoint_t setpoint = {0};
  setpoint.throttle_permille = throttle_permille;
  setpoint.baro_hold = true;
  return setpoint;
}

static mixer_feedback_t make_feedback(void)
{
  mixer_feedback_t feedback = {0};
  feedback.motor_min_permille = BOARD_MOTOR_IDLE_PERMILLE;
  feedback.motor_max_permille = BOARD_ALT_HOVER_THRUST_PERMILLE;
  feedback.attitude_scale_permille = 1000U;
  return feedback;
}

static controller_altitude_debug_t update(uint16_t throttle_permille,
                                          int32_t altitude_cm,
                                          int16_t velocity_cms,
                                          bool ready,
                                          uint8_t freeze_reason,
                                          const mixer_feedback_t *feedback,
                                          uint32_t timestamp_ms,
                                          uint16_t *base,
                                          int16_t *output)
{
  baro_sample_t baro = make_baro(altitude_cm, velocity_cms, timestamp_ms);
  control_setpoint_t setpoint = make_setpoint(throttle_permille);
  controller_altitude_debug_t debug;
  *output = ControllerAltitude_Update(&baro,
                                      &setpoint,
                                      true,
                                      ready,
                                      freeze_reason,
                                      feedback,
                                      0.002f,
                                      base);
  ControllerAltitude_GetDebug(&debug);
  return debug;
}

static controller_altitude_debug_t enter_alt_hold(uint32_t *timestamp_ms,
                                                  uint16_t *base,
                                                  int16_t *output)
{
  controller_altitude_debug_t debug;
  mixer_feedback_t feedback = make_feedback();

  ControllerAltitude_Reset();
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, true, 0U, &feedback, *timestamp_ms, base, output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE);

  for (uint8_t i = 0U; i < 80U; i++)
  {
    *timestamp_ms += 25U;
    if (i > 20U)
    {
      feedback.motor_max_permille = *base;
    }
    debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 280U,
                   (i > 20U) ? 25 : 0,
                   (i > 20U) ? 30 : 0,
                   true,
                   0U,
                   &feedback,
                   *timestamp_ms,
                   base,
                   output);
    if (debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD)
    {
      return debug;
    }
  }

  assert(!"alt hold did not engage");
  return debug;
}

static void test_ground_idle_holds_idle_at_or_below_center(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  controller_altitude_debug_t debug;
  mixer_feedback_t feedback = make_feedback();

  ControllerAltitude_Reset();
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, true, 0U, &feedback, 100U, &base, &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE);
  assert(debug.target_velocity_cms == 0);
  assert(debug.correction_permille == 0);
  assert(debug.base_permille == (int16_t)BOARD_MOTOR_IDLE_PERMILLE);
  assert(debug.output_permille == (int16_t)BOARD_MOTOR_IDLE_PERMILLE);
  assert(base == BOARD_MOTOR_IDLE_PERMILLE);
  assert(output == 0);

  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE - 40U, 0, 0, true, 0U, &feedback, 125U, &base, &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE);
  assert(base == BOARD_MOTOR_IDLE_PERMILLE);
  assert(output == 0);
}

static void test_takeoff_spools_base_without_negative_baro_boost(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  uint16_t previous = 0U;
  uint32_t timestamp_ms = 100U;
  controller_altitude_debug_t debug;
  mixer_feedback_t feedback = make_feedback();

  ControllerAltitude_Reset();
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, true, 0U, &feedback, timestamp_ms, &base, &output);

  for (uint8_t i = 0U; i < 35U; i++)
  {
    timestamp_ms += 25U;
    previous = base;
    debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 300U,
                   0,
                   BOARD_ALT_TOY_NEG_VEL_REJECT_CMS - 40,
                   true,
                   0U,
                   &feedback,
                   timestamp_ms,
                   &base,
                   &output);

    assert(debug.state == CONTROLLER_ALTITUDE_STATE_TAKEOFF ||
           debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
    assert(base <= BOARD_ALT_BASE_MAX_PERMILLE);
    assert((int16_t)(base - previous) <= (int16_t)BOARD_ALT_TAKEOFF_SLEW_PER_SAMPLE);
    assert(output <= 0);
  }
}

static void test_slightly_above_center_does_not_force_hover_throttle(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  uint16_t previous = 0U;
  uint32_t timestamp_ms = 100U;
  mixer_feedback_t feedback = make_feedback();
  controller_altitude_debug_t debug;

  ControllerAltitude_Reset();
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, true, 0U, &feedback, timestamp_ms, &base, &output);

  for (uint8_t i = 0U; i < 25U; i++)
  {
    timestamp_ms += 2U;
    previous = base;
    debug = update(BOARD_ALT_TAKEOFF_THROTTLE + 10U,
                   0,
                   0,
                   true,
                   0U,
                   &feedback,
                   timestamp_ms,
                   &base,
                   &output);
    assert(debug.state == CONTROLLER_ALTITUDE_STATE_TAKEOFF);
    assert((int16_t)(base - previous) <= 1);
  }

  assert(base < BOARD_ALT_TOY_HOVER_MIN_PERMILLE);
  assert(output == 0);
}

static void test_midstick_runs_cascaded_hold_correction(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  uint32_t timestamp_ms = 100U;
  mixer_feedback_t feedback = make_feedback();
  controller_altitude_debug_t debug = enter_alt_hold(&timestamp_ms, &base, &output);

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE,
                 debug.hold_altitude_cm - (BOARD_ALT_HOLD_DB_CM - 1),
                 0,
                 true,
                 0U,
                 &feedback,
                 timestamp_ms,
                 &base,
                 &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
  assert(debug.target_velocity_cms == 0);

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE,
                 debug.hold_altitude_cm - 120,
                 0,
                 true,
                 0U,
                 &feedback,
                 timestamp_ms,
                 &base,
                 &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
  assert(debug.target_velocity_cms > 0);
  assert(abs_i16_local(debug.correction_permille) <= (int16_t)BOARD_ALT_OUTPUT_LIMIT_PERMILLE);
  assert(debug.output_permille <= (int16_t)(debug.base_permille + BOARD_ALT_OUTPUT_LIMIT_PERMILLE));
}

static void test_midstick_small_altitude_error_stays_inside_deadband(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  uint32_t timestamp_ms = 100U;
  mixer_feedback_t feedback = make_feedback();
  controller_altitude_debug_t debug = enter_alt_hold(&timestamp_ms, &base, &output);

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE,
                 debug.hold_altitude_cm + 19,
                 0,
                 true,
                 0U,
                 &feedback,
                 timestamp_ms,
                 &base,
                 &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
  assert(debug.target_velocity_cms == 0);
  assert(debug.correction_permille == 0);
  assert(BOARD_ALT_HOLD_DB_CM == 20);
}

static void test_altitude_correction_is_slew_limited(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  uint32_t timestamp_ms = 100U;
  mixer_feedback_t feedback = make_feedback();
  controller_altitude_debug_t debug = enter_alt_hold(&timestamp_ms, &base, &output);

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE,
                 debug.hold_altitude_cm - 200,
                 -120,
                 true,
                 0U,
                 &feedback,
                 timestamp_ms,
                 &base,
                 &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
  assert(debug.correction_permille <= (int16_t)BOARD_ALT_CORR_SLEW_UP_PER_SAMPLE);

  int16_t previous = debug.correction_permille;
  timestamp_ms += 25U;
  debug = update(0U,
                 debug.hold_altitude_cm + 80,
                 120,
                 true,
                 0U,
                 &feedback,
                 timestamp_ms,
                 &base,
                 &output);

  assert(previous - debug.correction_permille <= (int16_t)BOARD_ALT_CORR_SLEW_DOWN_PER_SAMPLE);
}

static void test_low_stick_has_enough_negative_authority_to_pull_down(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  uint32_t timestamp_ms = 100U;
  mixer_feedback_t feedback = make_feedback();
  controller_altitude_debug_t debug;

  ControllerAltitude_Reset();
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, true, 0U, &feedback, timestamp_ms, &base, &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE);
  for (uint16_t i = 0U; i < 430U; i++)
  {
    timestamp_ms += 2U;
    debug = update(1000U, 0, 0, true, 0U, &feedback, timestamp_ms, &base, &output);
    if (debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD)
    {
      break;
    }
  }
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
  assert(debug.hover_permille > (int16_t)(BOARD_ALT_BASE_MIN_PERMILLE + 120U));

  for (uint8_t i = 0U; i < 40U; i++)
  {
    timestamp_ms += 25U;
    debug = update(0U, 25, 0, true, 0U, &feedback, timestamp_ms, &base, &output);
  }

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
  assert(debug.target_velocity_cms <= (int16_t)(-BOARD_ALT_STICK_DESCEND_MAX_CMS + 2));
  assert(output <= -30);
  assert(debug.output_permille < debug.hover_permille);
}

static void test_ready_loss_freezes_without_reset(void)
{
  int16_t output = -1;
  int16_t frozen_output = 0;
  uint16_t base = 0U;
  uint16_t frozen_base = 0U;
  uint32_t timestamp_ms = 100U;
  mixer_feedback_t feedback = make_feedback();
  controller_altitude_debug_t debug = enter_alt_hold(&timestamp_ms, &base, &output);

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, 15, -10, true, 0U, &feedback, timestamp_ms, &base, &output);
  frozen_base = base;
  frozen_output = output;

  timestamp_ms += 2U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE,
                 15,
                 -10,
                 false,
                 CONTROLLER_ALTITUDE_FREEZE_BARO,
                 &feedback,
                 timestamp_ms,
                 &base,
                 &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_FROZEN);
  assert(debug.active);
  assert(debug.freeze_reason == CONTROLLER_ALTITUDE_FREEZE_BARO);
  assert(base == frozen_base);
  assert(output == frozen_output);
}

static void test_stick_maps_to_velocity_and_velocity_pi_not_absolute_throttle(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  uint32_t timestamp_ms = 100U;
  mixer_feedback_t feedback = make_feedback();
  controller_altitude_debug_t debug = enter_alt_hold(&timestamp_ms, &base, &output);

  timestamp_ms += 25U;
  debug = update(1000U, 15, 0, true, 0U, &feedback, timestamp_ms, &base, &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
  assert(debug.target_velocity_cms >= (int16_t)(BOARD_ALT_STICK_CLIMB_MAX_CMS - 2));
  assert(base == (uint16_t)debug.hover_permille);
  assert(output > 0);
  assert(debug.output_permille == (int16_t)(debug.base_permille + output));
}

static void test_early_liftoff_does_not_lock_hover_too_low_for_climbout(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  uint32_t timestamp_ms = 100U;
  mixer_feedback_t feedback = make_feedback();
  controller_altitude_debug_t debug;

  ControllerAltitude_Reset();
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, true, 0U, &feedback, timestamp_ms, &base, &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE);

  for (uint8_t i = 0U; i < 20U; i++)
  {
    timestamp_ms += 25U;
    debug = update(BOARD_ALT_TAKEOFF_THROTTLE + 40U,
                   0,
                   0,
                   true,
                   0U,
                   &feedback,
                   timestamp_ms,
                   &base,
                   &output);
    assert(debug.state == CONTROLLER_ALTITUDE_STATE_TAKEOFF);
  }

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_TAKEOFF_THROTTLE + 40U,
                 BOARD_ALT_LIFTOFF_ALT_CM,
                 BOARD_ALT_LIFTOFF_VEL_CMS,
                 true,
                 0U,
                 &feedback,
                 timestamp_ms,
                 &base,
                 &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
  assert(debug.hover_permille >= (int16_t)BOARD_ALT_HOVER_THRUST_PERMILLE);

  for (uint8_t i = 0U; i < 80U; i++)
  {
    timestamp_ms += 25U;
    debug = update(1000U,
                   BOARD_ALT_LIFTOFF_ALT_CM,
                   0,
                   true,
                   0U,
                   &feedback,
                   timestamp_ms,
                   &base,
                   &output);
  }

  assert(debug.target_velocity_cms >= (int16_t)(BOARD_ALT_STICK_CLIMB_MAX_CMS - 2));
  assert(debug.output_permille >=
         (int16_t)(BOARD_ALT_HOVER_THRUST_PERMILLE + 50));
}

static void test_low_stick_descends_without_dropping_to_idle(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  uint32_t timestamp_ms = 100U;
  mixer_feedback_t feedback = make_feedback();
  controller_altitude_debug_t debug = enter_alt_hold(&timestamp_ms, &base, &output);

  for (uint8_t i = 0U; i < 12U; i++)
  {
    timestamp_ms += 25U;
    debug = update(0U, 20, 0, true, 0U, &feedback, timestamp_ms, &base, &output);
  }

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
  assert(debug.target_velocity_cms <= (int16_t)(-BOARD_ALT_STICK_DESCEND_MAX_CMS + 2));
  assert(debug.output_permille > (int16_t)BOARD_MOTOR_IDLE_PERMILLE);
  assert(debug.output_permille >= (int16_t)BOARD_ALT_BASE_MIN_PERMILLE);
}

static void test_mixer_saturation_prevents_positive_integrator_windup(void)
{
  int16_t output = -1;
  uint16_t base = 0U;
  uint32_t timestamp_ms = 100U;
  mixer_feedback_t feedback = make_feedback();
  controller_altitude_debug_t debug = enter_alt_hold(&timestamp_ms, &base, &output);

  feedback.saturated_high = true;
  feedback.motor_max_permille = BOARD_MOTOR_MAX_PERMILLE;
  feedback.attitude_scaled = true;
  feedback.attitude_scale_permille = 700U;

  int16_t first = 0;
  for (uint8_t i = 0U; i < 20U; i++)
  {
    timestamp_ms += 25U;
    debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, -120, -20, true, 0U, &feedback, timestamp_ms, &base, &output);
    if (i == 0U)
    {
      first = output;
    }
  }

  assert(debug.sat_hi);
  assert(debug.attitude_scale_permille == 700U);
  assert(output <= (first + 2));
}

int main(void)
{
  test_ground_idle_holds_idle_at_or_below_center();
  test_takeoff_spools_base_without_negative_baro_boost();
  test_slightly_above_center_does_not_force_hover_throttle();
  test_midstick_runs_cascaded_hold_correction();
  test_midstick_small_altitude_error_stays_inside_deadband();
  test_altitude_correction_is_slew_limited();
  test_low_stick_has_enough_negative_authority_to_pull_down();
  test_ready_loss_freezes_without_reset();
  test_stick_maps_to_velocity_and_velocity_pi_not_absolute_throttle();
  test_early_liftoff_does_not_lock_hover_too_low_for_climbout();
  test_low_stick_descends_without_dropping_to_idle();
  test_mixer_saturation_prevents_positive_integrator_windup();
  puts("controller_altitude_host_test: PASS");
  return 0;
}
