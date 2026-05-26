#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "controller_altitude.h"
#include "board_config.h"

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

static void test_ground_idle_holds_idle_at_or_below_center(void)
{
  int16_t output = -1;
  controller_altitude_debug_t debug;

  ControllerAltitude_Reset();
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, 100, &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE);
  assert(debug.target_velocity_cms == 0);
  assert(debug.correction_permille == 0);
  assert(output == BOARD_MOTOR_IDLE_PERMILLE);

  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE - 40U, 0, 0, 125, &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE);
  assert(output == BOARD_MOTOR_IDLE_PERMILLE);
}

static void test_stick_maps_linearly_to_vertical_speed(void)
{
  int16_t output = -1;
  controller_altitude_debug_t debug;

  ControllerAltitude_Reset();
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, 100, &output);
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + BOARD_ALT_HOLD_DB_PERMILLE + 10U,
                 0,
                 0,
                 125,
                 &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_TAKEOFF);
  assert(debug.target_velocity_cms > 0);
  assert(debug.target_velocity_cms <= 20);
  assert(output > (int16_t)BOARD_MOTOR_IDLE_PERMILLE);
  assert(output <= (int16_t)(BOARD_MOTOR_IDLE_PERMILLE + BOARD_ALT_OUTPUT_SLEW_UP_PER_SAMPLE));

  debug = update(1000U,
                 BOARD_ALT_LIFTOFF_ALT_CM + 2,
                 BOARD_ALT_LIFTOFF_VEL_CMS + 5,
                 150,
                 &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_FLYING);

  debug = update(1000U, BOARD_ALT_LIFTOFF_ALT_CM + 10, 20, 175, &output);
  assert(debug.target_velocity_cms >= (int16_t)(BOARD_ALT_STICK_CLIMB_MAX_CMS - 2));

  debug = update(0U, BOARD_ALT_LIFTOFF_ALT_CM + 10, 0, 200, &output);
  assert(debug.target_velocity_cms <= (int16_t)(-BOARD_ALT_STICK_DESCEND_MAX_CMS + 2));
}

static void test_takeoff_enters_flying_after_liftoff(void)
{
  int16_t output = -1;
  controller_altitude_debug_t debug;

  ControllerAltitude_Reset();
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, 100, &output);
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE + 250U, 0, 45, 125, &output);
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE, BOARD_ALT_LIFTOFF_ALT_CM + 2, 20, 150, &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_FLYING);
  assert(debug.velocity_control == false);
  assert(debug.target_velocity_cms == 0);
}

static void test_takeoff_below_center_after_spool_does_not_drop_to_idle(void)
{
  int16_t output = -1;
  controller_altitude_debug_t debug;
  uint32_t timestamp_ms = 100U;

  ControllerAltitude_Reset();
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, timestamp_ms, &output);
  while (output < (int16_t)BOARD_ALT_BASE_MIN_PERMILLE)
  {
    timestamp_ms += 25U;
    debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 250U, 0, 0, timestamp_ms, &output);
    assert(debug.state == CONTROLLER_ALTITUDE_STATE_TAKEOFF);
  }

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE - 120U, 0, 0, timestamp_ms, &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_FLYING);
  assert(debug.target_velocity_cms < 0);
  assert(output > (int16_t)BOARD_MOTOR_IDLE_PERMILLE);
}

static void test_flying_climb_overspeed_brakes_below_hover(void)
{
  int16_t output = -1;
  controller_altitude_debug_t debug;
  uint32_t timestamp_ms = 100U;

  ControllerAltitude_Reset();
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, timestamp_ms, &output);
  timestamp_ms += 25U;
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE + 250U, 0, BOARD_ALT_LIFTOFF_VEL_CMS + 5, timestamp_ms, &output);

  for (uint8_t i = 0U; i < 12U; i++)
  {
    timestamp_ms += 25U;
    debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 350U, 30, 0, timestamp_ms, &output);
    assert(debug.state == CONTROLLER_ALTITUDE_STATE_FLYING);
  }

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 350U,
                 60,
                 debug.target_velocity_cms + BOARD_ALT_ASCENT_OVERSPEED_MARGIN_CMS + 10,
                 timestamp_ms,
                 &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_FLYING);
  assert(output < debug.throttle_base_permille);
}

static void test_flying_full_climb_is_not_capped_by_old_low_ceiling(void)
{
  int16_t output = -1;
  controller_altitude_debug_t debug;
  uint32_t timestamp_ms = 100U;

  ControllerAltitude_Reset();
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, timestamp_ms, &output);
  for (uint8_t i = 0U; (i < 80U) && (output < (int16_t)(BOARD_ALT_HOVER_THRUST_PERMILLE + 16U)); i++)
  {
    timestamp_ms += 25U;
    debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 350U, 0, 0, timestamp_ms, &output);
    assert(debug.state == CONTROLLER_ALTITUDE_STATE_TAKEOFF);
  }
  assert(output >= (int16_t)(BOARD_ALT_HOVER_THRUST_PERMILLE + 16U));

  int16_t liftoff_output = output;
  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 350U,
                 BOARD_ALT_LIFTOFF_ALT_CM + 2,
                 BOARD_ALT_LIFTOFF_VEL_CMS + 5,
                 timestamp_ms,
                 &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_FLYING);

  for (uint8_t i = 0U; i < 40U; i++)
  {
    timestamp_ms += 25U;
    debug = update(1000U, 30, 0, timestamp_ms, &output);
    assert(debug.state == CONTROLLER_ALTITUDE_STATE_FLYING);
  }

  assert(debug.target_velocity_cms >= (int16_t)(BOARD_ALT_STICK_CLIMB_MAX_CMS - 2));
  assert(output > 420);
  assert(output > (int16_t)(liftoff_output + 40));
}

static void test_small_climb_target_overspeed_forces_output_down(void)
{
  int16_t output = -1;
  controller_altitude_debug_t debug;
  uint32_t timestamp_ms = 100U;

  ControllerAltitude_Reset();
  (void)update(BOARD_ALT_STICK_CENTER_PERMILLE, 0, 0, timestamp_ms, &output);
  while (output < (int16_t)(BOARD_ALT_BASE_MIN_PERMILLE + 24U))
  {
    timestamp_ms += 25U;
    debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 350U, 0, 0, timestamp_ms, &output);
    assert(debug.state == CONTROLLER_ALTITUDE_STATE_TAKEOFF);
  }

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + 350U,
                 BOARD_ALT_LIFTOFF_ALT_CM + 2,
                 BOARD_ALT_LIFTOFF_VEL_CMS + 5,
                 timestamp_ms,
                 &output);
  assert(debug.state == CONTROLLER_ALTITUDE_STATE_FLYING);

  timestamp_ms += 25U;
  debug = update(BOARD_ALT_STICK_CENTER_PERMILLE + BOARD_ALT_HOLD_DB_PERMILLE + 20U,
                 32,
                 35,
                 timestamp_ms,
                 &output);

  assert(debug.state == CONTROLLER_ALTITUDE_STATE_FLYING);
  assert(debug.target_velocity_cms < 10);
  assert(output < debug.throttle_base_permille);
}

int main(void)
{
  test_ground_idle_holds_idle_at_or_below_center();
  test_stick_maps_linearly_to_vertical_speed();
  test_takeoff_enters_flying_after_liftoff();
  test_takeoff_below_center_after_spool_does_not_drop_to_idle();
  test_flying_climb_overspeed_brakes_below_hover();
  test_flying_full_climb_is_not_capped_by_old_low_ceiling();
  test_small_climb_target_overspeed_forces_output_down();
  puts("controller_altitude_host_test: PASS");
  return 0;
}
