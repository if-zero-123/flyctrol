#include "app_main.h"

#include "app_tasks.h"
#include "battery_adc.h"
#include "bmp280.h"
#include "controller_altitude.h"
#include "controller_attitude.h"
#include "crsf.h"
#include "debug_uart.h"
#include "estimator_altitude.h"
#include "estimator_attitude.h"
#include "flight_monitor.h"
#include "i2c_bus.h"
#include "led.h"
#include "main.h"
#include "motor_pwm.h"
#include "mpu6050.h"
#include "safety.h"
#include "telemetry.h"
#include "topic.h"

static bool s_telemetry_log;

void App_Start(void)
{
  AppBootStage_Set(40U);
  Topic_Init();
  FlightMonitor_Init();
  DebugUart_Init();
  Telemetry_PrintBoot();
  DebugUart_WriteLine("init: app start");

  AppBootStage_Set(41U);
  I2cBus_Init();
  DebugUart_WriteLine("init: i2c");

  AppBootStage_Set(42U);
  Led_Init();
  MotorPwm_Init();
  BatteryAdc_Init();
  DebugUart_WriteLine("init: board");

  AppBootStage_Set(43U);
  EstimatorAttitude_Init();
  EstimatorAltitude_Init();
  ControllerAttitude_Init();
  ControllerAltitude_Init();
  Safety_Init();
  DebugUart_WriteLine("init: modules");

  AppBootStage_Set(44U);
  DebugUart_WriteLine("init: mpu6050");
  bool imu_ok = Mpu6050_Init();
  if (imu_ok)
  {
    DebugUart_WriteLine("init: gyrocal");
    imu_ok = Mpu6050_CalibrateGyro(128U);
  }
  AppBootStage_Set(45U);
  DebugUart_WriteLine("init: bmp280");
  bool baro_ok = Bmp280_Init();
  AppBootStage_Set(46U);
  DebugUart_WriteLine("init: crsf");
  bool crsf_ok = Crsf_Init();
  AppBootStage_Set(47U);

  DebugUart_Printf("init imu=%u baro=%u crsf=%u\r\n",
                   imu_ok ? 1U : 0U,
                   baro_ok ? 1U : 0U,
                   crsf_ok ? 1U : 0U);

  DebugUart_WriteLine("init: tasks");
  AppBootStage_Set(48U);
  App_CreateTasks();
  AppBootStage_Set(49U);
  DebugUart_WriteLine("init: done");
}

void App_SetTelemetryLog(bool enabled)
{
  s_telemetry_log = enabled;
}

bool App_GetTelemetryLog(void)
{
  return s_telemetry_log;
}
