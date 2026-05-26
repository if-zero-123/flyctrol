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

static controller_altitude_debug_t update(uint16_t throttle_permille,
                                          int32_t altitude_cm,
                                          int16_t velocity_cms,
                                          uint32_t timestamp_ms,
                                          int16_t *output)
{
  baro_sample_t baro = make_baro(altitude_cm, velocity_cms, timestamp_ms);
  control_setpoint_t setpoint = make_setpoint(throttle_permille);
  controller_altitude_debug_t debug;
  *output = ControllerAltitude_Update(&baro, &setpoint, true, 0.002f);
  ControllerAltitude_GetDebug(&debug);
  return debug;
}

static controller_altitude_debug_t enter_toy_assist(uint32_t *timestamp_ms, int16_t *output)
{
  controller_altitude_debug_t debug;

  ControllerAltitude_Reset();
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, *timestamp_ms, output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE);

  for (uint8_t i = 0U; i < 80U; i++)
  {
    *timestamp_ms += 25U;
    debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 280U, 0, 0, *timestamp_ms, output);
    if (debug.state == CONTROLLER_ALTITUDE_STATE_TOY_ASSIST)
    {
      return debug;
    }
  }

  assert(!"toy assist did not engage");
  return debug;
}

static void test_ground_idle_holds_idle_at_or_below_center(void)
{
  int16_t output = -1;
  controller_altitude_debug_t debug;

  ControllerAltitude_Reset();
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, 100U, &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE);
  assert(debug.target_velocity_cms == 0);
  assert(debug.correction_permille == 0);
  assert(debug.output_permille == (int16_t)BOARD_MOTOR_IDLE_PERMILLE);
  assert(output == (int16_t)BOARD_MOTOR_IDLE_PERMILLE);

  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE - 40U, 0, 0, 125U, &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE);
  assert(output == (int16_t)BOARD_MOTOR_IDLE_PERMILLE);
}

static void test_toy_takeoff_spools_without_negative_baro_boost(void)
{
  int16_t output = -1;
  int16_t previous = -1;
  uint32_t timestamp_ms = 100U;
  controller_altitude_debug_t debug;

  ControllerAltitude_Reset();
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, timestamp_ms, &output);

  for (uint8_t i = 0U; i < 35U; i++)
  {
    timestamp_ms += 25U;
    previous = output;
    debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 300U,
                   0,
                   BOARD_ALT_TOY_NEG_VEL_REJECT_CMS - 40,
                   timestamp_ms,
                   &output);

    assert(debug.state == CONTROLLER_ALTITUDE_STATE_TOY_TAKEOFF ||
           debug.state == CONTROLLER_ALTITUDE_STATE_TOY_ASSIST);
    assert(output <= (int16_t)BOARD_ALT_TOY_HOVER_MAX_PERMILLE);
    assert((output - previous) <= (int16_t)BOARD_ALT_TAKEOFF_SLEW_PER_SAMPLE);
    assert(debug.correction_permille <= (int16_t)BOARD_ALT_TOY_BAD_BARO_LIMIT_PERMILLE);
  }
}

static void test_midstick_assist_correction_is_small_and_limited(void)
{
  int16_t output = -1;
  uint32_t timestamp_ms = 100U;
  controller_altitude_debug_t debug = enter_toy_assist(&timestamp_ms, &output);

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, 20, 0, timestamp_ms, &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_TOY_ASSIST);
  assert(debug.target_velocity_cms == 0);

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, -80, 0, timestamp_ms, &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_TOY_ASSIST);
  assert(debug.target_velocity_cms == 0);
  assert(abs_i16_local(debug.correction_permille) <= (int16_t)BOARD_ALT_TOY_CORR_LIMIT_PERMILLE);
  assert(debug.output_permille <= (int16_t)(debug.throttle_base_permille + BOARD_ALT_TOY_CORR_LIMIT_PERMILLE));
}

static void test_negative_baro_velocity_rejects_positive_assist(void)
{
  int16_t output = -1;
  uint32_t timestamp_ms = 100U;
  controller_altitude_debug_t debug = enter_toy_assist(&timestamp_ms, &output);

  for (uint8_t i = 0U; i < 12U; i++)
  {
    timestamp_ms += 25U;
    debug = update(BOARD_ALT_STICK_CENTER_PERMILLE,
                   -120,
                   BOARD_ALT_TOY_NEG_VEL_REJECT_CMS - 30,
                   timestamp_ms,
                   &output);
  }

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_TOY_ASSIST);
  assert(debug.baro_rejected);
  assert(!debug.assist_reliable);
  assert(debug.correction_permille <= 0);
  assert(debug.output_permille <= debug.throttle_base_permille);
}

static void test_high_stick_follows_pilot_not_baro_error(void)
{
  int16_t output = -1;
  uint32_t timestamp_ms = 100U;
  controller_altitude_debug_t debug = enter_toy_assist(&timestamp_ms, &output);

  for (uint8_t i = 0U; i < 20U; i++)
  {
    timestamp_ms += 25U;
    debug = update(1000U,
                   15,
                   BOARD_ALT_TOY_NEG_VEL_REJECT_CMS - 40,
                   timestamp_ms,
                   &output);
  }

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_TOY_ASSIST);
  assert(debug.target_velocity_cms >= (int16_t)(BOARD_ALT_TOY_CLIMB_MAX_CMS - 2));
  assert(debug.baro_rejected);
  assert(debug.correction_permille <= 0);
  assert(output > debug.throttle_base_permille);
  assert(output <= (int16_t)BOARD_MOTOR_MAX_PERMILLE);
}

static void test_low_stick_descends_without_dropping_to_idle(void)
{
  int16_t output = -1;
  uint32_t timestamp_ms = 100U;
  controller_altitude_debug_t debug = enter_toy_assist(&timestamp_ms, &output);

  for (uint8_t i = 0U; i < 12U; i++)
  {
    timestamp_ms += 25U;
    debug = update(0U, 20, 0, timestamp_ms, &output);
  }

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_TOY_ASSIST);
  assert(debug.target_velocity_cms <= (int16_t)(-BOARD_ALT_TOY_DESCEND_MAX_CMS + 2));
  assert(output > (int16_t)BOARD_MOTOR_IDLE_PERMILLE);
  assert(output >= (int16_t)BOARD_ALT_BASE_MIN_PERMILLE);
}

static void test_altitude_jump_temporarily_rejects_baro_assist(void)
{
  int16_t output = -1;
  uint32_t timestamp_ms = 100U;
  controller_altitude_debug_t debug = enter_toy_assist(&timestamp_ms, &output);

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, 15, 0, timestamp_ms, &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_TOY_ASSIST);

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE,
                 15 + BOARD_ALT_TOY_ALT_JUMP_REJECT_CM + 30,
                 0,
                 timestamp_ms,
                 &output);

  assert(debug.baro_rejected);
  assert(abs_i16_local(debug.correction_permille) <= (int16_t)BOARD_ALT_TOY_BAD_BARO_LIMIT_PERMILLE);
}

int main(void)
{
  test_ground_idle_holds_idle_at_or_below_center();
  test_toy_takeoff_spools_without_negative_baro_boost();
  test_midstick_assist_correction_is_small_and_limited();
  test_negative_baro_velocity_rejects_positive_assist();
  test_high_stick_follows_pilot_not_baro_error();
  test_low_stick_descends_without_dropping_to_idle();
  test_altitude_jump_temporarily_rejects_baro_assist();
  puts("controller_altitude_host_test: PASS");
  return 0;
}
