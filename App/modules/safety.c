#include "safety.h"

#include "board_config.h"
#include "board_time.h"
#include "motor_pwm.h"
#include "topic.h"

static flight_status_t s_status;
static bool s_cli_arm_request;
static uint32_t s_motor_test_until_ms;
static bool s_motor_test_bench;

static float absf_local(float v)
{
  return (v < 0.0f) ? -v : v;
}

void Safety_Init(void)
{
  s_cli_arm_request = false;
  s_motor_test_until_ms = 0U;
  s_motor_test_bench = false;
  s_status.armed = false;
  s_status.failsafe = true;
  Topic_PublishStatus(&s_status);
}

void Safety_Update(void)
{
  app_rc_t rc = Topic_GetRc();
  attitude_t att = Topic_GetAttitude();
  baro_sample_t baro = Topic_GetBaro();
  battery_status_t batt = Topic_GetBattery();
  uint32_t now = BoardTime_Millis();

  bool rc_recent = rc.connected && ((now - rc.last_update_ms) <= BOARD_RC_TIMEOUT_MS) && !rc.failsafe;
  bool level_ok = att.healthy && (absf_local(att.roll_deg) < 75.0f) && (absf_local(att.pitch_deg) < 75.0f);
  bool battery_ok = !batt.critical;
  bool arm_request = rc.arm_switch || s_cli_arm_request;
  bool throttle_low = rc.throttle <= BOARD_ARM_THROTTLE_MAX;

  s_status.rc_ok = rc_recent;
  s_status.imu_ok = level_ok;
  s_status.baro_ok = baro.healthy;
  s_status.angle_mode = rc.angle_mode;
  s_status.baro_mode = rc.baro_mode && baro.healthy;
  s_status.throttle_permille = rc.throttle;
  s_status.uptime_ms = now;
  s_status.failsafe = (!rc_recent) || (!level_ok) || (!battery_ok);
  if (now > s_motor_test_until_ms)
  {
    s_motor_test_bench = false;
  }
  s_status.motor_test_unlocked = (!s_status.armed) && (now <= s_motor_test_until_ms) && (s_motor_test_bench || !s_status.failsafe);

  if (s_status.failsafe || !arm_request)
  {
    s_status.armed = false;
  }
  else if (!s_status.armed && arm_request && throttle_low)
  {
    s_status.armed = true;
  }

  if (!s_status.armed && !s_status.motor_test_unlocked)
  {
    MotorPwm_SetAll(0.0f);
  }

  Topic_PublishStatus(&s_status);
}

bool Safety_CanRunMotors(void)
{
  return s_status.armed && !s_status.failsafe;
}

bool Safety_CanMotorTest(void)
{
  Safety_Update();
  return s_status.motor_test_unlocked;
}

void Safety_RequestArm(bool enable)
{
  s_cli_arm_request = enable;
}

void Safety_RequestDisarm(void)
{
  s_cli_arm_request = false;
  s_status.armed = false;
  s_motor_test_bench = false;
  s_motor_test_until_ms = 0U;
  MotorPwm_SetAll(0.0f);
  Topic_PublishStatus(&s_status);
}

void Safety_MotorTestUnlock(uint32_t window_ms)
{
  if (window_ms == 0U)
  {
    window_ms = BOARD_MOTOR_TEST_WINDOW_MS;
  }
  s_motor_test_until_ms = BoardTime_Millis() + window_ms;
  s_motor_test_bench = false;
  Safety_Update();
}

void Safety_MotorTestBenchUnlock(uint32_t window_ms)
{
  if (window_ms == 0U)
  {
    window_ms = BOARD_MOTOR_TEST_WINDOW_MS;
  }
  s_status.armed = false;
  s_cli_arm_request = false;
  s_motor_test_until_ms = BoardTime_Millis() + window_ms;
  s_motor_test_bench = true;
  Safety_Update();
}

flight_status_t Safety_GetStatus(void)
{
  return s_status;
}
