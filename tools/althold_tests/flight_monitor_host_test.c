#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "flight_monitor.h"

static battery_status_t s_battery;
static attitude_t s_attitude;
static baro_sample_t s_baro;
static imu_sample_t s_imu;
static app_rc_t s_rc;
static char s_printed[1024];
static size_t s_printed_len;

battery_status_t Topic_GetBattery(void)
{
  return s_battery;
}

attitude_t Topic_GetAttitude(void)
{
  return s_attitude;
}

baro_sample_t Topic_GetBaro(void)
{
  return s_baro;
}

app_rc_t Topic_GetRc(void)
{
  return s_rc;
}

imu_sample_t Topic_GetImu(void)
{
  return s_imu;
}

void DebugUart_WriteLine(const char *text)
{
  if (text == NULL)
  {
    return;
  }
  int written = snprintf(&s_printed[s_printed_len],
                         sizeof(s_printed) - s_printed_len,
                         "%s\n",
                         text);
  if (written > 0)
  {
    s_printed_len += (size_t)written;
  }
}

void DebugUart_Printf(const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  int written = vsnprintf(&s_printed[s_printed_len],
                          sizeof(s_printed) - s_printed_len,
                          fmt,
                          ap);
  va_end(ap);
  if (written > 0)
  {
    s_printed_len += (size_t)written;
  }
}

static void clear_printed(void)
{
  memset(s_printed, 0, sizeof(s_printed));
  s_printed_len = 0U;
}

static void test_flight_summary_reports_signed_drift_bias(void)
{
  flight_status_t status = {0};

  FlightMonitor_Init();
  clear_printed();

  s_battery.voltage_mv = 8200U;
  s_baro.healthy = true;
  s_baro.altitude_cm = 12;
  s_baro.velocity_cms = 3;
  s_attitude.healthy = true;
  s_imu.gyro_dps[0] = 12.0f;
  s_imu.gyro_dps[1] = -8.0f;
  s_rc.roll = 40;
  s_rc.pitch = -30;

  status.armed = true;
  status.motor_max_permille = 760U;
  status.motor_idle_permille = 80U;
  status.throttle_permille = 420U;
  status.setpoint.roll_deg = 0.50f;
  status.setpoint.pitch_deg = -0.40f;
  status.control.roll = 0.020f;
  status.control.pitch = -0.010f;
  status.motor[0] = 0.40f;
  status.motor[1] = 0.44f;
  status.motor[2] = 0.42f;
  status.motor[3] = 0.41f;

  for (uint8_t i = 0U; i < 10U; i++)
  {
    status.uptime_ms = (uint32_t)(i * 2U);
    s_attitude.roll_deg = 2.00f;
    s_attitude.pitch_deg = -1.00f;
    FlightMonitor_Update(&status);
  }

  status.armed = false;
  status.uptime_ms = 30U;
  FlightMonitor_Update(&status);
  FlightMonitor_Print();

  assert(strstr(s_printed, "flight avg_cd") != NULL);
  assert(strstr(s_printed, "att=200,-100") != NULL);
  assert(strstr(s_printed, "sp=50,-40") != NULL);
  assert(strstr(s_printed, "rc=40,-30") != NULL);
  assert(strstr(s_printed, "out=20,-10") != NULL);
}

int main(void)
{
  test_flight_summary_reports_signed_drift_bias();
  puts("flight_monitor_host_test: PASS");
  return 0;
}
