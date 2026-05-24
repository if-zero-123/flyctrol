#include "telemetry.h"

#include "FreeRTOS.h"
#include "app_main.h"
#include "board_time.h"
#include "debug_uart.h"
#include "motor_pwm.h"
#include "task.h"
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

void Telemetry_PrintHeartbeat(void)
{
  if (!App_GetHeartbeat())
  {
    return;
  }

  flight_status_t st = Topic_GetStatus();
  app_rc_t rc = Topic_GetRc();
  battery_status_t batt = Topic_GetBattery();

  DebugUart_Printf("hb tick=%lums heap=%lu arm=%u fs=%u rc=%u imu=%u baro=%u thr=%u batt=%umV\r\n",
                   (unsigned long)BoardTime_Millis(),
                   (unsigned long)xPortGetFreeHeapSize(),
                   st.armed ? 1U : 0U,
                   st.failsafe ? 1U : 0U,
                   rc.connected ? 1U : 0U,
                   st.imu_ok ? 1U : 0U,
                   st.baro_ok ? 1U : 0U,
                   rc.throttle,
                   batt.voltage_mv);
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
  float motor[4];
  MotorPwm_GetLast(motor);

  DebugUart_Printf("st arm=%u fs=%u rc=%u imu=%u baro=%u thr=%u batt=%umV\r\n",
                   st.armed ? 1U : 0U,
                   st.failsafe ? 1U : 0U,
                   st.rc_ok ? 1U : 0U,
                   st.imu_ok ? 1U : 0U,
                   st.baro_ok ? 1U : 0U,
                   rc.throttle,
                   batt.voltage_mv);
  DebugUart_Printf("att cd r=%ld p=%ld y=%ld baro=%ldcm p=%ldPa\r\n",
                   (long)deg_to_cdeg(att.roll_deg),
                   (long)deg_to_cdeg(att.pitch_deg),
                   (long)deg_to_cdeg(att.yaw_deg),
                   (long)baro.altitude_cm,
                   (long)baro.pressure_pa);
  DebugUart_Printf("mot %ld %ld %ld %ld\r\n",
                   (long)duty_to_permille(motor[0]),
                   (long)duty_to_permille(motor[1]),
                   (long)duty_to_permille(motor[2]),
                   (long)duty_to_permille(motor[3]));
}
