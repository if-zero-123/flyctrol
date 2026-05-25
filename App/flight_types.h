#ifndef APP_FLIGHT_TYPES_H
#define APP_FLIGHT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define APP_RC_CHANNELS 16U
#define APP_MOTOR_COUNT 4U

typedef struct {
  int16_t ch[APP_RC_CHANNELS];
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  uint16_t throttle;
  bool connected;
  bool failsafe;
  bool arm_switch;
  bool angle_mode;
  bool baro_mode;
  uint32_t last_update_ms;
} app_rc_t;

typedef struct {
  float accel_g[3];
  float gyro_dps[3];
  int16_t temp_centi_c;
  bool healthy;
  uint32_t timestamp_ms;
} imu_sample_t;

typedef struct {
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  bool healthy;
  uint32_t timestamp_ms;
} attitude_t;

typedef struct {
  int32_t pressure_pa;
  int16_t temperature_centi_c;
  int32_t altitude_cm;
  bool healthy;
  uint32_t timestamp_ms;
} baro_sample_t;

typedef struct {
  uint16_t adc_raw;
  uint16_t voltage_mv;
  uint8_t percent;
  bool low;
  bool critical;
  uint32_t timestamp_ms;
} battery_status_t;

typedef struct {
  float roll_deg;
  float pitch_deg;
  float yaw_rate_dps;
  uint16_t throttle_permille;
  bool baro_hold;
} control_setpoint_t;

typedef struct {
  float roll;
  float pitch;
  float yaw;
  int16_t altitude_permille;
} control_output_t;

typedef struct {
  bool armed;
  bool failsafe;
  bool rc_ok;
  bool imu_ok;
  bool baro_ok;
  bool angle_mode;
  bool baro_mode;
  bool motor_test_unlocked;
  uint16_t failsafe_flags;
  uint16_t last_disarm_flags;
  uint16_t motor_idle_permille;
  uint16_t throttle_permille;
  float motor[APP_MOTOR_COUNT];
  uint32_t uptime_ms;
} flight_status_t;

#endif /* APP_FLIGHT_TYPES_H */
