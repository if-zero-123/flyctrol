#include "safety.h"

#include "board_config.h"
#include "board_time.h"
#include "flight_monitor.h"
#include "mixer_quad.h"
#include "motor_pwm.h"
#include "topic.h"

static flight_status_t s_status;
static bool s_cli_arm_request;
static bool s_arm_seen_low;
static bool s_arm_raw_prev;
static bool s_arm_request_stable;
static uint32_t s_arm_change_ms;
static uint32_t s_arm_lost_since_ms;
static uint32_t s_motor_test_until_ms;
static uint32_t s_battery_critical_since_ms;
static uint32_t s_crash_since_ms;
static bool s_motor_test_bench;
static bool s_crash_latched;

static float absf_local(float v)
{
  return (v < 0.0f) ? -v : v;
}

void Safety_Init(void)
{
  s_cli_arm_request = false;
  s_arm_seen_low = false;
  s_arm_raw_prev = false;
  s_arm_request_stable = false;
  s_arm_change_ms = 0U;
  s_arm_lost_since_ms = 0U;
  s_motor_test_until_ms = 0U;
  s_battery_critical_since_ms = 0U;
  s_crash_since_ms = 0U;
  s_motor_test_bench = false;
  s_crash_latched = false;
  s_status.armed = false;
  s_status.failsafe = true;
  Topic_PublishStatus(&s_status);
}

void Safety_Update(void)
{
  app_rc_t rc = Topic_GetRc();
  attitude_t att = Topic_GetAttitude();
  imu_sample_t imu = Topic_GetImu();
  baro_sample_t baro = Topic_GetBaro();
  battery_status_t batt = Topic_GetBattery();
  uint32_t now = BoardTime_Millis();
  bool was_armed = s_status.armed;

  bool rc_recent = rc.connected && ((now - rc.last_update_ms) <= BOARD_RC_TIMEOUT_MS) && !rc.failsafe;
  bool attitude_recent = att.healthy && (att.timestamp_ms != 0U) &&
                         ((now - att.timestamp_ms) <= BOARD_IMU_FAILSAFE_TIMEOUT_MS);
  bool baro_recent = baro.healthy && (baro.timestamp_ms != 0U) &&
                     ((now - baro.timestamp_ms) <= BOARD_BARO_TIMEOUT_MS);
  bool arming_angle_ok = attitude_recent && (absf_local(att.roll_deg) < 75.0f) &&
                         (absf_local(att.pitch_deg) < 75.0f);
  bool gyro_recent = imu.healthy && (imu.timestamp_ms != 0U) &&
                     ((now - imu.timestamp_ms) <= BOARD_IMU_FAILSAFE_TIMEOUT_MS);
  bool crash_motion = s_status.armed && attitude_recent &&
                      ((absf_local(att.roll_deg) > BOARD_CRASH_ANGLE_DEG) ||
                       (absf_local(att.pitch_deg) > BOARD_CRASH_ANGLE_DEG));
  if (s_status.armed && gyro_recent)
  {
    crash_motion = crash_motion ||
                   (absf_local(imu.gyro_dps[0]) > BOARD_CRASH_GYRO_DPS) ||
                   (absf_local(imu.gyro_dps[1]) > BOARD_CRASH_GYRO_DPS);
  }
  if (!s_status.armed || !attitude_recent)
  {
    s_crash_since_ms = 0U;
    s_crash_latched = false;
  }
  else if (crash_motion)
  {
    if (s_crash_since_ms == 0U)
    {
      s_crash_since_ms = now;
    }
    if ((now - s_crash_since_ms) >= BOARD_CRASH_HOLD_MS)
    {
      s_crash_latched = true;
    }
  }
  else
  {
    s_crash_since_ms = 0U;
  }
  bool battery_sample_critical = batt.voltage_mv <= BOARD_BATT_CRITICAL_MV;
  if (batt.voltage_mv >= BOARD_BATT_RECOVER_MV)
  {
    s_battery_critical_since_ms = 0U;
  }
  else if (battery_sample_critical && (s_battery_critical_since_ms == 0U))
  {
    s_battery_critical_since_ms = now;
  }
  bool battery_ok = (s_battery_critical_since_ms == 0U) ||
                    ((now - s_battery_critical_since_ms) < BOARD_BATT_CRITICAL_HOLD_MS);
  bool arm_raw = rc.arm_switch || s_cli_arm_request;
  if (arm_raw != s_arm_raw_prev)
  {
    s_arm_raw_prev = arm_raw;
    s_arm_change_ms = now;
  }
  if ((now - s_arm_change_ms) >= BOARD_ARM_SWITCH_DEBOUNCE_MS)
  {
    s_arm_request_stable = arm_raw;
  }
  bool arm_request = s_arm_request_stable;
  if (!arm_request)
  {
    s_arm_seen_low = true;
  }
  bool throttle_low = rc.throttle <= BOARD_ARM_THROTTLE_MAX;
  uint16_t failsafe_flags = 0U;
  uint16_t arm_block_flags = 0U;

  if (!rc_recent)
  {
    failsafe_flags |= SAFETY_FAILSAFE_RC;
    arm_block_flags |= SAFETY_ARM_BLOCK_RC;
  }
  if (!attitude_recent)
  {
    failsafe_flags |= SAFETY_FAILSAFE_IMU;
    arm_block_flags |= SAFETY_ARM_BLOCK_IMU;
  }
  if (attitude_recent && !arming_angle_ok)
  {
    arm_block_flags |= SAFETY_ARM_BLOCK_LEVEL;
  }
  if (!battery_ok)
  {
    arm_block_flags |= SAFETY_ARM_BLOCK_BATTERY;
  }
  if (s_crash_latched && (BOARD_CRASH_DISARM_ENABLE != 0U))
  {
    failsafe_flags |= SAFETY_FAILSAFE_CRASH;
    arm_block_flags |= SAFETY_ARM_BLOCK_CRASH;
  }
  if (!throttle_low)
  {
    arm_block_flags |= SAFETY_ARM_BLOCK_THROTTLE;
  }
  if (!s_arm_seen_low)
  {
    arm_block_flags |= SAFETY_ARM_BLOCK_LATCH;
  }

  s_status.rc_ok = rc_recent;
  s_status.imu_ok = attitude_recent;
  s_status.baro_ok = baro_recent;
  s_status.angle_mode = rc.angle_mode;
  s_status.baro_mode = rc.baro_mode && baro_recent;
  s_status.crash_detected = s_crash_latched;
  s_status.throttle_permille = rc.throttle;
  s_status.motor_idle_permille = MixerQuad_GetMotorIdlePermille();
  s_status.motor_max_permille = MixerQuad_GetMotorMaxPermille();
  s_status.uptime_ms = now;
  s_status.failsafe_flags = failsafe_flags;
  s_status.arm_block_flags = arm_block_flags;
  s_status.failsafe = failsafe_flags != 0U;
  if (now > s_motor_test_until_ms)
  {
    s_motor_test_bench = false;
  }
  s_status.motor_test_unlocked = (!s_status.armed) && (now <= s_motor_test_until_ms) && (s_motor_test_bench || !s_status.failsafe);

  bool inflight_stop = !rc_recent || !attitude_recent ||
                       (s_crash_latched && (BOARD_CRASH_DISARM_ENABLE != 0U));
  bool arm_lost_confirmed = !arm_request;
  if (was_armed && !arm_request)
  {
    if (s_arm_lost_since_ms == 0U)
    {
      s_arm_lost_since_ms = now;
    }
    arm_lost_confirmed = (now - s_arm_lost_since_ms) >= BOARD_INFLIGHT_ARM_LOST_HOLD_MS;
  }
  else
  {
    s_arm_lost_since_ms = 0U;
  }

  if (arm_lost_confirmed || (was_armed && inflight_stop))
  {
    s_status.armed = false;
    if (was_armed)
    {
      if (s_crash_latched && (BOARD_CRASH_DISARM_ENABLE != 0U))
      {
        s_status.last_disarm_flags = SAFETY_DISARM_CRASH;
      }
      else
      {
        s_status.last_disarm_flags = arm_request ? failsafe_flags : SAFETY_DISARM_ARM_LOST;
      }
      if (arm_request)
      {
        s_arm_seen_low = false;
      }
    }
  }
  else if (!s_status.armed && arm_request && (arm_block_flags == 0U))
  {
    s_status.armed = true;
    s_status.last_disarm_flags = 0U;
    s_arm_seen_low = false;
    s_crash_since_ms = 0U;
    s_crash_latched = false;
  }

  if (!s_status.armed && !s_status.motor_test_unlocked)
  {
    MotorPwm_SetAll(0.0f);
  }

  flight_status_t latest = Topic_GetStatus();
  s_status.setpoint = latest.setpoint;
  s_status.control = latest.control;
  s_status.air_mode = latest.air_mode;
  for (uint8_t i = 0U; i < APP_MOTOR_COUNT; i++)
  {
    s_status.motor[i] = latest.motor[i];
  }

  FlightMonitor_Update(&s_status);
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
  s_status.last_disarm_flags = SAFETY_DISARM_ARM_LOST;
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
