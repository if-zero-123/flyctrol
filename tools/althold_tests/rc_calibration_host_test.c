#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "rc_calibration.h"

static void test_default_center_matches_crsf_midpoint(void)
{
  RcCalibration_Reset();

  assert(RcCalibration_NormalizeStick(RC_CAL_AXIS_ROLL, 992U) == 0);
  assert(RcCalibration_NormalizeStick(RC_CAL_AXIS_PITCH, 992U) == 0);
  assert(RcCalibration_NormalizeStick(RC_CAL_AXIS_YAW, 992U) == 0);
}

static void test_runtime_center_removes_stick_bias(void)
{
  RcCalibration_Reset();
  RcCalibration_SetCenter(1018U, 975U, 1006U);

  assert(RcCalibration_NormalizeStick(RC_CAL_AXIS_ROLL, 1018U) == 0);
  assert(RcCalibration_NormalizeStick(RC_CAL_AXIS_PITCH, 975U) == 0);
  assert(RcCalibration_NormalizeStick(RC_CAL_AXIS_YAW, 1006U) == 0);
  assert(RcCalibration_GetCenter(RC_CAL_AXIS_ROLL) == 1018U);
  assert(RcCalibration_GetCenter(RC_CAL_AXIS_PITCH) == 975U);
  assert(RcCalibration_GetCenter(RC_CAL_AXIS_YAW) == 1006U);
}

static void test_runtime_center_preserves_full_stick_range(void)
{
  RcCalibration_Reset();
  RcCalibration_SetCenter(1018U, 975U, 1006U);

  assert(RcCalibration_NormalizeStick(RC_CAL_AXIS_ROLL, 1811U) == 1000);
  assert(RcCalibration_NormalizeStick(RC_CAL_AXIS_ROLL, 172U) == -1000);
  assert(RcCalibration_NormalizeThrottle(172U) == 0U);
  assert(RcCalibration_NormalizeThrottle(1811U) == 1000U);
}

int main(void)
{
  test_default_center_matches_crsf_midpoint();
  test_runtime_center_removes_stick_bias();
  test_runtime_center_preserves_full_stick_range();
  puts("rc_calibration_host_test: PASS");
  return 0;
}
