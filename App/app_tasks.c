#include "app_tasks.h"

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
#include "telemetry.h"
#include "topic.h"

static void StabilizerTask(void *argument);
static void CrsfTask(void *argument);
static void SafetyTask(void *argument);
static void BaroTask(void *argument);
static void BatteryTask(void *argument);
static void TelemetryTask(void *argument);
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
  create_task(TelemetryTask, "telem", 256U, 1U, 56U);
  create_task(CliTask, "cli", 512U, 1U, 57U);
}

static void StabilizerTask(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(2U);
  uint32_t last_imu_retry_ms = 0U;
  bool airmode_latched = false;
  bool previous_armed = false;

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
    if (status.armed && !previous_armed)
    {
      ControllerAttitude_Reset();
      ControllerAltitude_Reset();
      MixerQuad_ResetThrottleRamp();
      Mpu6050_ResetFilters();
    }
    else if (!status.armed && previous_armed)
    {
      ControllerAttitude_Reset();
      ControllerAltitude_Reset();
      MixerQuad_ResetThrottleRamp();
      Mpu6050_ResetFilters();
      EstimatorAttitude_Init();
    }
    previous_armed = status.armed;

    if (!status.armed)
    {
      airmode_latched = false;
    }
    else if ((BOARD_AIRMODE_ENABLE != 0U) && (sp.throttle_permille >= BOARD_AIRMODE_START_THROTTLE))
    {
      airmode_latched = true;
    }
    sp.air_mode = (BOARD_AIRMODE_ENABLE != 0U) && airmode_latched;
    bool control_enabled = status.armed &&
                           ((sp.throttle_permille > BOARD_ARM_THROTTLE_MAX) || sp.air_mode);
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
    bool baro_recent = baro.healthy &&
                       ((BoardTime_Millis() - baro.timestamp_ms) <= BOARD_BARO_TIMEOUT_MS);
    bool altitude_ready = control_enabled && status.baro_mode && attitude.healthy && baro_recent &&
                          (absf_local(attitude.roll_deg) <= BOARD_ALT_TILT_LIMIT_DEG) &&
                          (absf_local(attitude.pitch_deg) <= BOARD_ALT_TILT_LIMIT_DEG);
    bool altitude_active = altitude_ready &&
                           (ControllerAltitude_IsActive() ||
                            (sp.throttle_permille >= BOARD_ALT_ENABLE_THROTTLE_MIN));
    control.altitude_permille = ControllerAltitude_Update(&baro, &sp, altitude_active, 0.002f);

    if (Safety_CanRunMotors())
    {
      if (control_enabled)
      {
        MixerQuad_Mix(sp.throttle_permille, &control, motor);
        MotorPwm_Set4(motor);
      }
      else
      {
        MixerQuad_ResetThrottleRamp();
        MotorPwm_SetAllPermille(MixerQuad_GetMotorIdlePermille());
      }
    }
    else if (!Safety_CanMotorTest())
    {
      MixerQuad_ResetThrottleRamp();
      MotorPwm_SetAll(0.0f);
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

static void TelemetryTask(void *argument)
{
  (void)argument;
  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
    Telemetry_PrintOnce();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(200U));
  }
}

static void CliTask(void *argument)
{
  (void)argument;
  Cli_TaskLoop();
}
