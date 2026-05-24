#ifndef APP_BOARD_CONFIG_H
#define APP_BOARD_CONFIG_H

#include <stdint.h>

#define BOARD_PWM_PERIOD_COUNTS       4499U
#define BOARD_PWM_FULL_COUNTS         4500U
#define BOARD_BATTERY_DIVIDER_NUM     11U
#define BOARD_BATTERY_DIVIDER_DEN     1U
#define BOARD_ADC_REF_MV              3300U
#define BOARD_ADC_MAX_COUNTS          4095U
#define BOARD_BATT_LOW_MV             3500U
#define BOARD_BATT_CRITICAL_MV        3300U
#define BOARD_BATT_FULL_MV            4200U
#define BOARD_BATT_EMPTY_MV           3300U

#define BOARD_CONTROL_LOOP_HZ         500U
#define BOARD_MAX_ANGLE_DEG           25.0f
#define BOARD_MAX_YAW_RATE_DPS        120.0f
#define BOARD_ARM_THROTTLE_MAX        50U
#define BOARD_RC_TIMEOUT_MS           300U
#define BOARD_MOTOR_TEST_WINDOW_MS    5000U

typedef enum {
  BOARD_MOTOR_1 = 0,
  BOARD_MOTOR_2 = 1,
  BOARD_MOTOR_3 = 2,
  BOARD_MOTOR_4 = 3
} board_motor_id_t;

#endif /* APP_BOARD_CONFIG_H */
