#include "app_tasks.h"

#if defined(__GNUC__)
#pragma GCC optimize ("Os")
#endif

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "battery_adc.h"
#include "bmp280.h"
#include "board_config.h"
#include "board_time.h"
#include "cli.h"
#include "commander.h"
#include "controller_altitude.h"
#include "controller_attitude.h"
#include "crsf.h"
#include "debug_uart.h"
#include "estimator_altitude.h"
#include "estimator_attitude.h"
#include "flight_types.h"
#include "led.h"
#include "main.h"
#include "mixer_quad.h"
#include "motor_pwm.h"
#include "mpu6050.h"
#include "safety.h"
#include "topic.h"

static void StabilizerTask(void *argument);
static void CrsfTask(void *argument);
static void SafetyTask(void *argument);
static void BaroTask(void *argument);
static void BatteryTask(void *argument);
static void CliTask(void *argument);

static float absf_local(float v)
{
  return (v < 0.0f) ? -v : v;
}

static void create_task(TaskFunction_t fn,
                        const char *name,
                        uint16_t stack_words,
                        UBaseType_t priority,
                        uint32_t fail_stage)
{
  if (xTaskCreate(fn, name, stack_words, NULL, priority, NULL) != pdPASS)
  {
    DebugUart_Printf("task create failed: %s\r\n", name);
    AppBootStage_Set(fail_stage);
    Error_Handler();
  }
}

void App_CreateTasks(void)
{
  create_task(StabilizerTask, "stabilize", 320U, 5U, 51U);
  create_task(CrsfTask, "crsf", 192U, 6U, 52U);
  create_task(SafetyTask, "safety", 160U, 4U, 53U);
  create_task(BaroTask, "baro", 192U, 3U, 54U);
  create_task(BatteryTask, "battery", 144U, 2U, 55U);
  create_task(CliTask, "cli", 384U, 1U, 57U);
}

static void StabilizerTask(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(2U);
  uint32_t last_imu_retry_ms = 0U;
  bool airmode_latched = false;
  bool previous_armed = false;
  bool takeoff_latched = false;
  mixer_feedback_t previous_mixer_feedback = {0};
  previous_mixer_feedback.attitude_scale_permille = 1000U;

  for (;;)
  {
    imu_sample_t imu;
    attitude_t attitude;
    app_rc_t rc;
    control_setpoint_t sp;
    control_output_t control = {0};
    float motor[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool imu_updated = false;

    if (Mpu6050_Read(&imu))
    {
      Topic_PublishImu(&imu);
      EstimatorAttitude_Update(&imu, 0.002f, &attitude);
      Topic_PublishAttitude(&attitude);
      EstimatorAltitude_PredictImu(&imu, &attitude, 0.002f);
      imu_updated = true;
    }
    else
    {
      uint32_t now = BoardTime_Millis();
      imu = Topic_GetImu();
      attitude = Topic_GetAttitude();

      if ((now - last_imu_retry_ms) >= 1000U)
      {
        last_imu_retry_ms = now;
        (void)Mpu6050_Init();
      }
    }

    rc = Topic_GetRc();
    Commander_BuildSetpoint(&rc, &sp);
    Safety_Update();
    flight_status_t status = Safety_GetStatus();
    baro_sample_t baro = Topic_GetBaro();
    bool baro_recent = baro.healthy &&
                       ((BoardTime_Millis() - baro.timestamp_ms) <= BOARD_BARO_TIMEOUT_MS);
    if (status.armed && !previous_armed)
    {
      ControllerAttitude_Reset();
      ControllerAltitude_Reset();
      EstimatorAltitude_ResetDynamic();
      MixerQuad_ResetThrottleRamp();
      Mpu6050_ResetFilters();
    }
    else if (!status.armed && previous_armed)
    {
      ControllerAttitude_Reset();
      ControllerAltitude_Reset();
      EstimatorAltitude_ResetDynamic();
      MixerQuad_ResetThrottleRamp();
      Mpu6050_ResetFilters();
      EstimatorAttitude_Init();
    }
    previous_armed = status.armed;

    if (!status.armed)
    {
      airmode_latched = false;
      takeoff_latched = false;
      memset(&previous_mixer_feedback, 0, sizeof(previous_mixer_feedback));
      previous_mixer_feedback.attitude_scale_permille = 1000U;
    }
    else if ((BOARD_AIRMODE_ENABLE != 0U) &&
             (((!rc.baro_mode) && (sp.throttle_permille >= BOARD_AIRMODE_START_THROTTLE)) ||
              (rc.baro_mode && ControllerAltitude_IsFlying())))
    {
      airmode_latched = true;
    }
    if (status.armed && !rc.baro_mode)
    {
      takeoff_latched = true;
    }
    else if (status.armed && rc.baro_mode && (sp.throttle_permille > BOARD_ALT_TAKEOFF_THROTTLE))
    {
      takeoff_latched = true;
    }
    sp.air_mode = (BOARD_AIRMODE_ENABLE != 0U) && airmode_latched;
    bool baro_mode_requested = rc.baro_mode;
    bool control_enabled = status.armed &&
                           takeoff_latched &&
                           (baro_mode_requested ||
                            (sp.throttle_permille > BOARD_ARM_THROTTLE_MAX) ||
                            sp.air_mode);
    if (!control_enabled)
    {
      ControllerAttitude_Reset();
    }

    if (control_enabled && imu_updated)
    {
      ControllerAttitude_Update(&attitude, &imu, &sp, 0.002f, &control);
    }
    else if (control_enabled)
    {
      control = status.control;
    }
    bool tilt_ok = attitude.healthy &&
                   (absf_local(attitude.roll_deg) <= BOARD_ALT_TILT_LIMIT_DEG) &&
                   (absf_local(attitude.pitch_deg) <= BOARD_ALT_TILT_LIMIT_DEG);
    uint8_t altitude_freeze_reason = CONTROLLER_ALTITUDE_FREEZE_NONE;
    if (!baro_recent)
    {
      altitude_freeze_reason |= CONTROLLER_ALTITUDE_FREEZE_BARO;
    }
    if (!status.imu_ok || !attitude.healthy)
    {
      altitude_freeze_reason |= CONTROLLER_ALTITUDE_FREEZE_IMU;
    }
    if (!tilt_ok)
    {
      altitude_freeze_reason |= CONTROLLER_ALTITUDE_FREEZE_TILT;
    }
    bool altitude_requested = status.armed && baro_mode_requested && sp.baro_hold;
    bool altitude_ready = altitude_requested && baro_recent && status.imu_ok && attitude.healthy && tilt_ok;
    uint16_t altitude_base_throttle = sp.throttle_permille;
    control.altitude_permille = ControllerAltitude_Update(&baro,
                                                          &sp,
                                                          altitude_requested,
                                                          altitude_ready,
                                                          altitude_freeze_reason,
                                                          &previous_mixer_feedback,
                                                          0.002f,
                                                          &altitude_base_throttle);

    if (Safety_CanRunMotors())
    {
      if (control_enabled)
      {
        mixer_feedback_t mixer_feedback = {0};
        mixer_feedback.attitude_scale_permille = 1000U;
        if (baro_mode_requested && ControllerAltitude_IsActive())
        {
          MixerQuad_MixWithFeedback(altitude_base_throttle, &control, motor, &mixer_feedback);
        }
        else
        {
          MixerQuad_MixWithFeedback(sp.throttle_permille, &control, motor, &mixer_feedback);
        }
        previous_mixer_feedback = mixer_feedback;
        MotorPwm_Set4(motor);
      }
      else
      {
        MixerQuad_PrimeThrottleRamp(MixerQuad_GetMotorIdlePermille());
        MotorPwm_SetAllPermille(MixerQuad_GetMotorIdlePermille());
        memset(&previous_mixer_feedback, 0, sizeof(previous_mixer_feedback));
        previous_mixer_feedback.attitude_scale_permille = 1000U;
      }
    }
    else if (!Safety_CanMotorTest())
    {
      MixerQuad_ResetThrottleRamp();
      MotorPwm_SetAll(0.0f);
      memset(&previous_mixer_feedback, 0, sizeof(previous_mixer_feedback));
      previous_mixer_feedback.attitude_scale_permille = 1000U;
    }

    status = Safety_GetStatus();
    MotorPwm_GetLast(status.motor);
    status.throttle_permille = rc.throttle;
    status.setpoint = sp;
    status.air_mode = sp.air_mode;
    status.control = control;
    Topic_PublishStatus(&status);

    vTaskDelayUntil(&last, period);
  }
}

static void CrsfTask(void *argument)
{
  (void)argument;
  Crsf_SetTaskHandle(xTaskGetCurrentTaskHandle());
  for (;;)
  {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));
    Crsf_ProcessRx();
    Crsf_UpdateLinkState();
  }
}

static void SafetyTask(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
    Safety_Update();
    flight_status_t status = Safety_GetStatus();
    Led_UpdateStatus(&status);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10U));
  }
}

static void BaroTask(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  uint32_t last_baro_retry_ms = 0U;
  for (;;)
  {
    baro_sample_t raw;
    baro_sample_t filtered;
    if (Bmp280_Read(&raw))
    {
      EstimatorAltitude_Update(&raw, &filtered);
      Topic_PublishBaro(&filtered);
    }
    else
    {
      uint32_t now = BoardTime_Millis();
      raw.healthy = false;
      Topic_PublishBaro(&raw);

      if ((now - last_baro_retry_ms) >= 1000U)
      {
        last_baro_retry_ms = now;
        (void)Bmp280_Init();
      }
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(25U));
  }
}

static void BatteryTask(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
    battery_status_t batt;
    if (BatteryAdc_Read(&batt))
    {
      Topic_PublishBattery(&batt);
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(100U));
  }
}

static void CliTask(void *argument)
{
  (void)argument;
  Cli_TaskLoop();
}
