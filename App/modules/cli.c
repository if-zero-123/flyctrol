#include "cli.h"

#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "app_main.h"
#include "battery_adc.h"
#include "bmp280.h"
#include "board_config.h"
#include "controller_attitude.h"
#include "debug_uart.h"
#include "i2c_bus.h"
#include "main.h"
#include "motor_pwm.h"
#include "mpu6050.h"
#include "safety.h"
#include "topic.h"

static int32_t deg_to_cdeg(float deg)
{
  return (int32_t)(deg * 100.0f);
}

static int32_t gain_to_milli(float gain)
{
  return (int32_t)(gain * 1000.0f);
}

static void print_help(void)
{
  DebugUart_WriteLine("cmd: help status tasks heap i2cscan imu baro rc batt");
  DebugUart_WriteLine("cmd: motor unlock|stop|<1-4> <permille>, motors <permille>");
  DebugUart_WriteLine("cmd: pid [roll|pitch|yaw <kp_milli> <ki_milli> <kd_milli>]");
  DebugUart_WriteLine("cmd: arm disarm log on|off hb on|off reboot");
}

static void print_status(void)
{
  flight_status_t st = Topic_GetStatus();
  DebugUart_Printf("armed=%u failsafe=%u rc=%u imu=%u baro=%u mode angle=%u baro=%u\r\n",
                   st.armed ? 1U : 0U,
                   st.failsafe ? 1U : 0U,
                   st.rc_ok ? 1U : 0U,
                   st.imu_ok ? 1U : 0U,
                   st.baro_ok ? 1U : 0U,
                   st.angle_mode ? 1U : 0U,
                   st.baro_mode ? 1U : 0U);
  DebugUart_Printf("uptime=%lums throttle=%u motor_test=%u\r\n",
                   (unsigned long)st.uptime_ms,
                   st.throttle_permille,
                   st.motor_test_unlocked ? 1U : 0U);
}

static void print_tasks(void)
{
  char buf[512];
  memset(buf, 0, sizeof(buf));
  DebugUart_WriteLine("name          state prio stack num");
  vTaskList(buf);
  DebugUart_Write(buf);
}

static void print_heap(void)
{
  DebugUart_Printf("heap free=%lu min=%lu\r\n",
                   (unsigned long)xPortGetFreeHeapSize(),
                   (unsigned long)xPortGetMinimumEverFreeHeapSize());
}

static void scan_i2c(void)
{
  DebugUart_Write("i2c:");
  for (uint8_t addr = 0x08U; addr <= 0x77U; addr++)
  {
    bool found = false;
    if (I2cBus_Lock(20U))
    {
      found = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 1U, 2U) == HAL_OK;
      I2cBus_Unlock();
    }
    if (found)
    {
      DebugUart_Printf(" 0x%02X", addr);
    }
  }
  DebugUart_WriteLine("");
}

static void print_imu(void)
{
  imu_sample_t imu = Topic_GetImu();
  attitude_t att = Topic_GetAttitude();
  DebugUart_Printf("imu ok=%u acc_mg=%ld,%ld,%ld gyro_cdps=%ld,%ld,%ld temp=%d.%02dC\r\n",
                   imu.healthy ? 1U : 0U,
                   (long)(imu.accel_g[0] * 1000.0f),
                   (long)(imu.accel_g[1] * 1000.0f),
                   (long)(imu.accel_g[2] * 1000.0f),
                   (long)(imu.gyro_dps[0] * 100.0f),
                   (long)(imu.gyro_dps[1] * 100.0f),
                   (long)(imu.gyro_dps[2] * 100.0f),
                   imu.temp_centi_c / 100,
                   abs(imu.temp_centi_c % 100));
  DebugUart_Printf("att cd roll=%ld pitch=%ld yaw=%ld\r\n",
                   (long)deg_to_cdeg(att.roll_deg),
                   (long)deg_to_cdeg(att.pitch_deg),
                   (long)deg_to_cdeg(att.yaw_deg));
}

static void print_baro(void)
{
  baro_sample_t baro = Topic_GetBaro();
  DebugUart_Printf("baro ok=%u temp=%d.%02dC pressure=%ldPa altitude=%ldcm\r\n",
                   baro.healthy ? 1U : 0U,
                   baro.temperature_centi_c / 100,
                   abs(baro.temperature_centi_c % 100),
                   (long)baro.pressure_pa,
                   (long)baro.altitude_cm);
}

static void print_rc(void)
{
  app_rc_t rc = Topic_GetRc();
  DebugUart_Printf("rc connected=%u failsafe=%u arm=%u baro=%u age=%lums\r\n",
                   rc.connected ? 1U : 0U,
                   rc.failsafe ? 1U : 0U,
                   rc.arm_switch ? 1U : 0U,
                   rc.baro_mode ? 1U : 0U,
                   (unsigned long)(HAL_GetTick() - rc.last_update_ms));
  DebugUart_Printf("stick r=%d p=%d y=%d t=%u raw=%d,%d,%d,%d,%d,%d\r\n",
                   rc.roll,
                   rc.pitch,
                   rc.yaw,
                   rc.throttle,
                   rc.ch[0],
                   rc.ch[1],
                   rc.ch[2],
                   rc.ch[3],
                   rc.ch[4],
                   rc.ch[5]);
}

static void print_batt(void)
{
  battery_status_t batt = Topic_GetBattery();
  DebugUart_Printf("batt raw=%u voltage=%umV percent=%u low=%u critical=%u\r\n",
                   batt.adc_raw,
                   batt.voltage_mv,
                   batt.percent,
                   batt.low ? 1U : 0U,
                   batt.critical ? 1U : 0U);
}

static bool parse_axis(const char *name, pid_axis_t *axis)
{
  if ((name == NULL) || (axis == NULL))
  {
    return false;
  }
  if (strcmp(name, "roll") == 0)
  {
    *axis = PID_AXIS_ROLL;
    return true;
  }
  if (strcmp(name, "pitch") == 0)
  {
    *axis = PID_AXIS_PITCH;
    return true;
  }
  if (strcmp(name, "yaw") == 0)
  {
    *axis = PID_AXIS_YAW;
    return true;
  }
  return false;
}

static void print_pid_axis(pid_axis_t axis, const char *name)
{
  app_pid_t pid;
  if (ControllerAttitude_GetPid(axis, &pid))
  {
    DebugUart_Printf("%s kp=%ld ki=%ld kd=%ld milli\r\n",
                     name,
                     (long)gain_to_milli(pid.kp),
                     (long)gain_to_milli(pid.ki),
                     (long)gain_to_milli(pid.kd));
  }
}

static void cmd_pid(char *axis_name, char *kp_s, char *ki_s, char *kd_s)
{
  pid_axis_t axis;
  if (axis_name == NULL)
  {
    print_pid_axis(PID_AXIS_ROLL, "roll");
    print_pid_axis(PID_AXIS_PITCH, "pitch");
    print_pid_axis(PID_AXIS_YAW, "yaw");
    return;
  }

  if (!parse_axis(axis_name, &axis) || (kp_s == NULL) || (ki_s == NULL) || (kd_s == NULL))
  {
    DebugUart_WriteLine("usage: pid roll|pitch|yaw <kp_milli> <ki_milli> <kd_milli>");
    return;
  }

  float kp = (float)atoi(kp_s) / 1000.0f;
  float ki = (float)atoi(ki_s) / 1000.0f;
  float kd = (float)atoi(kd_s) / 1000.0f;
  if (ControllerAttitude_SetPid(axis, kp, ki, kd))
  {
    DebugUart_WriteLine("pid ok");
  }
}

static void cmd_motor(char *arg1, char *arg2)
{
  if (arg1 == NULL)
  {
    DebugUart_WriteLine("usage: motor unlock|stop|<1-4> <permille>");
    return;
  }

  if (strcmp(arg1, "unlock") == 0)
  {
    Safety_MotorTestUnlock(BOARD_MOTOR_TEST_WINDOW_MS);
    DebugUart_WriteLine("motor test unlocked for 5s if safety is ok");
    return;
  }

  if (strcmp(arg1, "stop") == 0)
  {
    MotorPwm_SetAll(0.0f);
    DebugUart_WriteLine("motors stopped");
    return;
  }

  if (!Safety_CanMotorTest())
  {
    DebugUart_WriteLine("motor test denied: disarm, valid RC, no failsafe, then motor unlock");
    return;
  }

  uint8_t motor = (uint8_t)atoi(arg1);
  uint16_t permille = (arg2 == NULL) ? 0U : (uint16_t)atoi(arg2);
  if ((motor < 1U) || (motor > 4U) || (permille > 1000U))
  {
    DebugUart_WriteLine("usage: motor <1-4> <0-1000>");
    return;
  }
  MotorPwm_SetPermille((uint8_t)(motor - 1U), permille);
  DebugUart_Printf("motor %u=%u\r\n", motor, permille);
}

static void cmd_motors(char *arg1)
{
  if (!Safety_CanMotorTest())
  {
    DebugUart_WriteLine("motors denied: disarm, valid RC, no failsafe, then motor unlock");
    return;
  }
  uint16_t permille = (arg1 == NULL) ? 0U : (uint16_t)atoi(arg1);
  if (permille > 1000U)
  {
    DebugUart_WriteLine("usage: motors <0-1000>");
    return;
  }
  MotorPwm_SetAllPermille(permille);
  DebugUart_Printf("motors=%u\r\n", permille);
}

static void execute_line(char *line)
{
  char *cmd = strtok(line, " \t");
  char *a1 = strtok(NULL, " \t");
  char *a2 = strtok(NULL, " \t");
  char *a3 = strtok(NULL, " \t");
  char *a4 = strtok(NULL, " \t");

  if (cmd == NULL)
  {
    return;
  }
  if (strcmp(cmd, "help") == 0)
  {
    print_help();
  }
  else if (strcmp(cmd, "status") == 0)
  {
    print_status();
  }
  else if (strcmp(cmd, "tasks") == 0)
  {
    print_tasks();
  }
  else if (strcmp(cmd, "heap") == 0)
  {
    print_heap();
  }
  else if (strcmp(cmd, "i2cscan") == 0)
  {
    scan_i2c();
  }
  else if (strcmp(cmd, "imu") == 0)
  {
    print_imu();
  }
  else if (strcmp(cmd, "baro") == 0)
  {
    print_baro();
  }
  else if (strcmp(cmd, "rc") == 0)
  {
    print_rc();
  }
  else if (strcmp(cmd, "batt") == 0)
  {
    print_batt();
  }
  else if (strcmp(cmd, "motor") == 0)
  {
    cmd_motor(a1, a2);
  }
  else if (strcmp(cmd, "motors") == 0)
  {
    cmd_motors(a1);
  }
  else if (strcmp(cmd, "pid") == 0)
  {
    cmd_pid(a1, a2, a3, a4);
  }
  else if (strcmp(cmd, "arm") == 0)
  {
    Safety_RequestArm(true);
    DebugUart_WriteLine("arm requested");
  }
  else if (strcmp(cmd, "disarm") == 0)
  {
    Safety_RequestDisarm();
    DebugUart_WriteLine("disarmed");
  }
  else if (strcmp(cmd, "log") == 0)
  {
    bool on = (a1 != NULL) && (strcmp(a1, "on") == 0);
    App_SetTelemetryLog(on);
    DebugUart_Printf("log=%u\r\n", on ? 1U : 0U);
  }
  else if (strcmp(cmd, "hb") == 0)
  {
    bool on = (a1 == NULL) || (strcmp(a1, "off") != 0);
    App_SetHeartbeat(on);
    DebugUart_Printf("hb=%u\r\n", on ? 1U : 0U);
  }
  else if (strcmp(cmd, "reboot") == 0)
  {
    DebugUart_WriteLine("rebooting");
    vTaskDelay(pdMS_TO_TICKS(50));
    NVIC_SystemReset();
  }
  else
  {
    DebugUart_WriteLine("unknown command");
  }
}

void Cli_TaskLoop(void)
{
  char line[96];
  uint8_t pos = 0U;
  uint8_t ch;

  DebugUart_Write("> ");
  for (;;)
  {
    if (!DebugUart_ReadByte(&ch, 20U))
    {
      continue;
    }

    if ((ch == '\r') || (ch == '\n'))
    {
      DebugUart_WriteLine("");
      line[pos] = '\0';
      execute_line(line);
      pos = 0U;
      DebugUart_Write("> ");
    }
    else if ((ch == 0x08U) || (ch == 0x7FU))
    {
      if (pos > 0U)
      {
        pos--;
        DebugUart_Write("\b \b");
      }
    }
    else if ((ch >= 32U) && (ch < 127U) && (pos < (sizeof(line) - 1U)))
    {
      line[pos++] = (char)ch;
      (void)HAL_UART_Transmit(&huart1, &ch, 1U, 20U);
    }
  }
}
