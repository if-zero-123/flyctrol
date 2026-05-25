#include "flight_monitor.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "battery_adc.h"
#include "debug_uart.h"
#include "topic.h"

typedef struct {
  bool seen;
  bool active;
  uint32_t start_ms;
  uint32_t end_ms;
  uint16_t stop_flags;
  uint16_t last_disarm;
  uint16_t min_batt_mv;
  uint16_t max_throttle;
  uint16_t max_motor;
  uint16_t max_motor_spread;
  uint16_t min_heap_free;
  bool althold_seen;
  int32_t min_baro_alt_cm;
  int32_t max_baro_alt_cm;
  int16_t max_abs_baro_vel_cms;
  int16_t max_abs_alt_out_permille;
  int16_t max_abs_roll_cd;
  int16_t max_abs_pitch_cd;
  int16_t max_out_r_milli;
  int16_t max_out_p_milli;
  int16_t max_out_y_milli;
} flight_summary_t;

static flight_summary_t s_summary;
static bool s_prev_armed;

static int16_t abs_i16(int16_t v)
{
  return (v < 0) ? (int16_t)-v : v;
}

static int16_t clamp_i16(int32_t v)
{
  if (v > 32767) { return 32767; }
  if (v < -32768) { return -32768; }
  return (int16_t)v;
}

static uint16_t duty_permille(float duty)
{
  if (duty < 0.0f) { duty = 0.0f; }
  if (duty > 1.0f) { duty = 1.0f; }
  return (uint16_t)(duty * 1000.0f);
}

static void begin_flight(uint32_t now)
{
  memset(&s_summary, 0, sizeof(s_summary));
  s_summary.seen = true;
  s_summary.active = true;
  s_summary.start_ms = now;
  s_summary.end_ms = now;
  s_summary.min_batt_mv = 65535U;
  s_summary.min_heap_free = 65535U;
  s_summary.min_baro_alt_cm = 2147483647L;
  s_summary.max_baro_alt_cm = -2147483647L;
}

void FlightMonitor_Init(void)
{
  taskENTER_CRITICAL();
  memset(&s_summary, 0, sizeof(s_summary));
  s_prev_armed = false;
  taskEXIT_CRITICAL();
}

void FlightMonitor_Update(const flight_status_t *status)
{
  if (status == 0)
  {
    return;
  }

  battery_status_t batt = Topic_GetBattery();
  attitude_t att = Topic_GetAttitude();
  baro_sample_t baro = Topic_GetBaro();
  uint32_t now = status->uptime_ms;

  taskENTER_CRITICAL();
  if (status->armed && !s_prev_armed)
  {
    begin_flight(now);
  }

  if (s_summary.seen && (status->armed || s_prev_armed))
  {
    uint16_t min_motor = duty_permille(status->motor[0]);
    uint16_t max_motor = min_motor;
    for (uint8_t i = 1U; i < APP_MOTOR_COUNT; i++)
    {
      uint16_t m = duty_permille(status->motor[i]);
      if (m < min_motor) { min_motor = m; }
      if (m > max_motor) { max_motor = m; }
    }

    int16_t roll_cd = abs_i16(clamp_i16((int32_t)(att.roll_deg * 100.0f)));
    int16_t pitch_cd = abs_i16(clamp_i16((int32_t)(att.pitch_deg * 100.0f)));
    int16_t out_r = abs_i16(clamp_i16((int32_t)(status->control.roll * 1000.0f)));
    int16_t out_p = abs_i16(clamp_i16((int32_t)(status->control.pitch * 1000.0f)));
    int16_t out_y = abs_i16(clamp_i16((int32_t)(status->control.yaw * 1000.0f)));
    int16_t alt_out = abs_i16(status->control.altitude_permille);
    int16_t baro_vel = abs_i16(baro.velocity_cms);

    s_summary.end_ms = now;
    s_summary.stop_flags = status->failsafe_flags;
    s_summary.last_disarm = status->last_disarm_flags;
    if ((batt.voltage_mv > 0U) && (batt.voltage_mv < s_summary.min_batt_mv))
    {
      s_summary.min_batt_mv = batt.voltage_mv;
    }
    if (status->throttle_permille > s_summary.max_throttle)
    {
      s_summary.max_throttle = status->throttle_permille;
    }
    {
      size_t heap_free = xPortGetFreeHeapSize();
      if (heap_free < s_summary.min_heap_free)
      {
        s_summary.min_heap_free = (heap_free > 65535U) ? 65535U : (uint16_t)heap_free;
      }
    }
    if (max_motor > s_summary.max_motor)
    {
      s_summary.max_motor = max_motor;
    }
    uint16_t spread = (uint16_t)(max_motor - min_motor);
    if (spread > s_summary.max_motor_spread)
    {
      s_summary.max_motor_spread = spread;
    }
    if (status->baro_mode || status->setpoint.baro_hold || (status->control.altitude_permille != 0))
    {
      s_summary.althold_seen = true;
    }
    if (baro.healthy)
    {
      if (baro.altitude_cm < s_summary.min_baro_alt_cm) { s_summary.min_baro_alt_cm = baro.altitude_cm; }
      if (baro.altitude_cm > s_summary.max_baro_alt_cm) { s_summary.max_baro_alt_cm = baro.altitude_cm; }
      if (baro_vel > s_summary.max_abs_baro_vel_cms) { s_summary.max_abs_baro_vel_cms = baro_vel; }
    }
    if (alt_out > s_summary.max_abs_alt_out_permille) { s_summary.max_abs_alt_out_permille = alt_out; }
    if (roll_cd > s_summary.max_abs_roll_cd) { s_summary.max_abs_roll_cd = roll_cd; }
    if (pitch_cd > s_summary.max_abs_pitch_cd) { s_summary.max_abs_pitch_cd = pitch_cd; }
    if (out_r > s_summary.max_out_r_milli) { s_summary.max_out_r_milli = out_r; }
    if (out_p > s_summary.max_out_p_milli) { s_summary.max_out_p_milli = out_p; }
    if (out_y > s_summary.max_out_y_milli) { s_summary.max_out_y_milli = out_y; }
  }

  if (!status->armed && s_prev_armed)
  {
    s_summary.active = false;
    s_summary.end_ms = now;
    s_summary.stop_flags = status->failsafe_flags;
    s_summary.last_disarm = status->last_disarm_flags;
  }
  s_prev_armed = status->armed;
  taskEXIT_CRITICAL();
}

void FlightMonitor_Print(void)
{
  flight_summary_t s;

  taskENTER_CRITICAL();
  s = s_summary;
  taskEXIT_CRITICAL();

  if (!s.seen)
  {
    DebugUart_WriteLine("flight none");
    return;
  }

  uint32_t duration_ms = (s.end_ms >= s.start_ms) ? (s.end_ms - s.start_ms) : 0U;
  DebugUart_Printf("flight active=%u duration=%lums stop_flags=0x%04X last_disarm=0x%04X\r\n",
                   s.active ? 1U : 0U,
                   (unsigned long)duration_ms,
                   s.stop_flags,
                   s.last_disarm);
  DebugUart_Printf("flight min_batt=%umV max_thr=%u max_motor=%u spread=%u\r\n",
                   (s.min_batt_mv == 65535U) ? 0U : s.min_batt_mv,
                   s.max_throttle,
                   s.max_motor,
                   s.max_motor_spread);
  DebugUart_Printf("flight max_angle_cd roll=%d pitch=%d max_out_milli r=%d p=%d y=%d\r\n",
                   s.max_abs_roll_cd,
                   s.max_abs_pitch_cd,
                   s.max_out_r_milli,
                   s.max_out_p_milli,
                   s.max_out_y_milli);
  DebugUart_Printf("flight althold=%u alt_min=%ldcm alt_max=%ldcm max_vel=%dcm/s max_alt_out=%d\r\n",
                   s.althold_seen ? 1U : 0U,
                   (long)((s.min_baro_alt_cm == 2147483647L) ? 0 : s.min_baro_alt_cm),
                   (long)((s.max_baro_alt_cm == -2147483647L) ? 0 : s.max_baro_alt_cm),
                   s.max_abs_baro_vel_cms,
                   s.max_abs_alt_out_permille);
  DebugUart_Printf("flight min_heap=%u bytes\r\n",
                   (s.min_heap_free == 65535U) ? 0U : s.min_heap_free);
}
