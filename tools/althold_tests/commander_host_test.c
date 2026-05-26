#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "board_config.h"
#include "commander.h"

static app_rc_t make_rc(int16_t roll, int16_t pitch, bool baro_mode)
{
  app_rc_t rc = {0};
  rc.roll = roll;
  rc.pitch = pitch;
  rc.connected = true;
  rc.angle_mode = true;
  rc.baro_mode = baro_mode;
  rc.throttle = BOARD_ALT_STICK_CENTER_PERMILLE;
  return rc;
}

static void test_normal_angle_mode_keeps_full_manual_angle(void)
{
  app_rc_t rc = make_rc(1000, 0, false);
  control_setpoint_t sp = {0};

  Commander_Reset();
  Commander_BuildSetpoint(&rc, &sp);

  assert(fabsf(sp.roll_deg - BOARD_MAX_ANGLE_DEG) < 0.01f);
}

static void test_baro_mode_uses_smaller_toy_angle_limit(void)
{
  app_rc_t rc = make_rc(1000, 0, true);
  control_setpoint_t sp = {0};

  Commander_Reset();
  Commander_BuildSetpoint(&rc, &sp);

  assert(sp.roll_deg <= (BOARD_BARO_MAX_ANGLE_DEG + 0.01f));
  assert(sp.roll_deg < BOARD_MAX_ANGLE_DEG);
}

static void test_baro_mode_has_wide_center_deadband(void)
{
  app_rc_t rc = make_rc(BOARD_BARO_RC_DEADBAND - 1, -(BOARD_BARO_RC_DEADBAND - 1), true);
  control_setpoint_t sp = {0};

  Commander_Reset();
  Commander_BuildSetpoint(&rc, &sp);

  assert(sp.roll_deg == 0.0f);
  assert(sp.pitch_deg == 0.0f);
}

static void test_baro_mode_slews_roll_pitch_targets_after_centered_entry(void)
{
  control_setpoint_t sp = {0};
  app_rc_t rc = make_rc(0, 0, true);

  Commander_Reset();
  Commander_BuildSetpoint(&rc, &sp);
  assert(sp.roll_deg == 0.0f);

  rc.roll = 1000;
  Commander_BuildSetpoint(&rc, &sp);

  assert(sp.roll_deg > 0.0f);
  assert(sp.roll_deg <= (BOARD_BARO_SETPOINT_SLEW_DEG_PER_SAMPLE + 0.001f));
}

int main(void)
{
  test_normal_angle_mode_keeps_full_manual_angle();
  test_baro_mode_uses_smaller_toy_angle_limit();
  test_baro_mode_has_wide_center_deadband();
  test_baro_mode_slews_roll_pitch_targets_after_centered_entry();
  puts("commander_host_test: PASS");
  return 0;
}
