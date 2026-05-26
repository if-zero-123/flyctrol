#include "telemetry.h"

#if defined(__GNUC__)
#pragma GCC optimize ("Os")
#endif

#include "app_main.h"
#include "controller_altitude.h"
#include "debug_uart.h"
#include "motor_pwm.h"
#include "topic.h"

static int32_t deg_to_cdeg(float deg)
{
  return (int32_t)(deg * 100.0f);
}

static int32_t duty_to_permille(float duty)
{
  if (duty < 0.0f)
  {
    duty = 0.0f;
  }
  if (duty > 1.0f)
  {
    duty = 1.0f;
  }
  return (int32_t)(duty * 1000.0f);
}

void Telemetry_PrintBoot(void)
{
  DebugUart_WriteLine("");
  DebugUart_WriteLine("NAZE32 custom firmware boot");
  DebugUart_WriteLine("CLI ready: type help");
}

void Telemetry_PrintOnce(void)
{
  if (!App_GetTelemetryLog())
  {
    return;
  }

  flight_status_t st = Topic_GetStatus();
  app_rc_t rc = Topic_GetRc();
  attitude_t att = Topic_GetAttitude();
  baro_sample_t baro = Topic_GetBaro();
  battery_status_t batt = Topic_GetBattery();
  controller_altitude_debug_t alt;
  float motor[4];
  ControllerAltitude_GetDebug(&alt);
  MotorPwm_GetLast(motor);

  DebugUart_Printf("st arm=%u fs=%u flags=0x%04X dis=0x%04X rc=%u imu=%u baro=%u air=%u crash=%u thr=%u batt=%umV\r\n",
                   st.armed ? 1U : 0U,
                   st.failsafe ? 1U : 0U,
                   st.failsafe_flags,
                   st.last_disarm_flags,
                   st.rc_ok ? 1U : 0U,
                   st.imu_ok ? 1U : 0U,
                   st.baro_ok ? 1U : 0U,
                   st.air_mode ? 1U : 0U,
                   st.crash_detected ? 1U : 0U,
                   rc.throttle,
                   batt.voltage_mv);
  DebugUart_Printf("att cd r=%ld p=%ld y=%ld baro=%ldcm vel=%dcm/s p=%ldPa\r\n",
                   (long)deg_to_cdeg(att.roll_deg),
                   (long)deg_to_cdeg(att.pitch_deg),
                   (long)deg_to_cdeg(att.yaw_deg),
                   (long)baro.altitude_cm,
                   baro.velocity_cms,
                   (long)baro.pressure_pa);
  DebugUart_Printf("ctl sp=%ld,%ld,%ld thr=%u alt_act=%u alt_base=%d alt_hover=%d alt_corr=%d alt_out=%d out=%ld,%ld,%ld alt_state=%u alt_rel=%u alt_rej=%u alt_lim=%d freeze=%u bq=%u sat=%u,%u scale=%u\r\n",
                   (long)deg_to_cdeg(st.setpoint.roll_deg),
                   (long)deg_to_cdeg(st.setpoint.pitch_deg),
                   (long)deg_to_cdeg(st.setpoint.yaw_rate_dps),
                   st.setpoint.throttle_permille,
                   alt.active ? 1U : 0U,
                   alt.base_permille,
                   alt.hover_permille,
                   alt.correction_permille,
                   alt.output_permille,
                   (long)(st.control.roll * 1000.0f),
                   (long)(st.control.pitch * 1000.0f),
                   (long)(st.control.yaw * 1000.0f),
                   (unsigned int)alt.state,
                   alt.assist_reliable ? 1U : 0U,
                   alt.baro_rejected ? 1U : 0U,
                   alt.assist_correction_limit_permille,
                   alt.freeze_reason,
                   alt.baro_quality,
                   alt.sat_hi ? 1U : 0U,
                   alt.sat_lo ? 1U : 0U,
                   alt.attitude_scale_permille);
  DebugUart_Printf("mot %ld %ld %ld %ld\r\n",
                   (long)duty_to_permille(motor[0]),
                   (long)duty_to_permille(motor[1]),
                   (long)duty_to_permille(motor[2]),
                   (long)duty_to_permille(motor[3]));
}
