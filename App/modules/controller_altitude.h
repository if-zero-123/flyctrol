#ifndef APP_CONTROLLER_ALTITUDE_H
#define APP_CONTROLLER_ALTITUDE_H

#include "flight_types.h"

void ControllerAltitude_Init(void);
void ControllerAltitude_Reset(void);
bool ControllerAltitude_IsActive(void);
bool ControllerAltitude_IsFlying(void);
int16_t ControllerAltitude_Update(const baro_sample_t *baro,
                                  const control_setpoint_t *setpoint,
                                  bool requested,
                                  bool ready,
                                  uint8_t freeze_reason,
                                  const mixer_feedback_t *feedback,
                                  float dt_s,
                                  uint16_t *base_throttle_permille);

typedef enum {
  CONTROLLER_ALTITUDE_STATE_INACTIVE = 0,
  CONTROLLER_ALTITUDE_STATE_GROUND_IDLE,
  CONTROLLER_ALTITUDE_STATE_TAKEOFF,
  CONTROLLER_ALTITUDE_STATE_ALT_HOLD,
  CONTROLLER_ALTITUDE_STATE_FROZEN,
  CONTROLLER_ALTITUDE_STATE_TOY_TAKEOFF = CONTROLLER_ALTITUDE_STATE_TAKEOFF,
  CONTROLLER_ALTITUDE_STATE_TOY_ASSIST = CONTROLLER_ALTITUDE_STATE_ALT_HOLD,
  CONTROLLER_ALTITUDE_STATE_FLYING = CONTROLLER_ALTITUDE_STATE_ALT_HOLD
} controller_altitude_state_t;

typedef enum {
  CONTROLLER_ALTITUDE_FREEZE_NONE = 0U,
  CONTROLLER_ALTITUDE_FREEZE_BARO = 1U,
  CONTROLLER_ALTITUDE_FREEZE_IMU = 2U,
  CONTROLLER_ALTITUDE_FREEZE_TILT = 4U,
  CONTROLLER_ALTITUDE_FREEZE_TIMEOUT = 8U,
  CONTROLLER_ALTITUDE_FREEZE_SATURATION = 16U
} controller_altitude_freeze_reason_t;

typedef struct {
  bool active;
  bool velocity_control;
  controller_altitude_state_t state;
  int32_t hold_altitude_cm;
  int16_t altitude_error_cm;
  int16_t velocity_cms;
  int16_t estimated_velocity_cms;
  int16_t target_velocity_cms;
  int16_t stick_reference_permille;
  int16_t throttle_base_permille;
  int16_t base_permille;
  int16_t hover_permille;
  int16_t correction_permille;
  int16_t output_permille;
  bool assist_reliable;
  bool baro_rejected;
  int16_t assist_correction_limit_permille;
  uint8_t freeze_reason;
  uint8_t baro_quality;
  bool imu_predict_enabled;
  bool sat_hi;
  bool sat_lo;
  uint16_t motor_min_permille;
  uint16_t motor_max_permille;
  uint16_t motor_spread_permille;
  uint16_t attitude_scale_permille;
} controller_altitude_debug_t;

void ControllerAltitude_GetDebug(controller_altitude_debug_t *out);

#endif /* APP_CONTROLLER_ALTITUDE_H */
