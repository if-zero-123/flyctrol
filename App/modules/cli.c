#include "cli.h"

#include <stdbool.h>
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
#include "estimator_attitude.h"
#include "i2c_bus.h"
#include "main.h"
#include "mixer_quad.h"
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

static int32_t float_to_milli(float value)
{
  return (int32_t)(value * 1000.0f);
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

static void print_help(void)
{
  DebugUart_WriteLine("cmd: help status clock tasks heap i2cscan imu baro rc rcmap batt battdiag");
  DebugUart_WriteLine("cmd: motormap control yawdir normal|reverse mixcheck [r_milli p_milli y_milli thr]");
  DebugUart_WriteLine("cmd: motoridle [0-200], motormax [700-1000], motor unlock|stop|<1-4> <permille>");
  DebugUart_WriteLine("cmd: pid [roll|pitch|yaw <kp_milli> <ki_milli> <kd_milli>]");
  DebugUart_WriteLine("cmd: trim [roll_cdeg pitch_cdeg], leveltrim, gyrocal acccal imucal arm disarm log on|off reboot");
}

static const char *sysclk_source_name(void)
{
  uint32_t source = __HAL_RCC_GET_SYSCLK_SOURCE();
  if (source == RCC_SYSCLKSOURCE_STATUS_HSI)
  {
    return "HSI";
  }
  if (source == RCC_SYSCLKSOURCE_STATUS_HSE)
  {
    return "HSE";
  }
  if (source == RCC_SYSCLKSOURCE_STATUS_PLLCLK)
  {
    return "PLL";
  }
  return "UNKNOWN";
}

static const char *pll_source_name(void)
{
  if (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
  {
    return "NONE";
  }
  return ((RCC->CFGR & RCC_CFGR_PLLSRC) != 0U) ? "HSE" : "HSI_DIV2";
}

static void print_clock(void)
{
  DebugUart_Printf("clock src=%s pll=%s sys=%luHz hclk=%luHz pclk1=%luHz pclk2=%luHz fallback=%lu\r\n",
                   sysclk_source_name(),
                   pll_source_name(),
                   (unsigned long)HAL_RCC_GetSysClockFreq(),
                   (unsigned long)HAL_RCC_GetHCLKFreq(),
                   (unsigned long)HAL_RCC_GetPCLK1Freq(),
                   (unsigned long)HAL_RCC_GetPCLK2Freq(),
                   (unsigned long)AppClock_IsHsiFallback());
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
  DebugUart_Printf("failsafe_flags=0x%04X last_disarm=0x%04X motor_idle=%u motor_max=%u\r\n",
                   st.failsafe_flags,
                   st.last_disarm_flags,
                   st.motor_idle_permille,
                   st.motor_max_permille);
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
  DebugUart_Printf("heap free=%lu min=%lu rxdrop=%lu\r\n",
                   (unsigned long)xPortGetFreeHeapSize(),
                   (unsigned long)xPortGetMinimumEverFreeHeapSize(),
                   (unsigned long)DebugUart_RxDropped());
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
  uint32_t read_ok = 0U;
  uint32_t read_fail = 0U;
  Mpu6050_GetReadStats(&read_ok, &read_fail);
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
  DebugUart_Printf("imu reads ok=%lu fail=%lu age=%lums\r\n",
                   (unsigned long)read_ok,
                   (unsigned long)read_fail,
                   (unsigned long)(HAL_GetTick() - imu.timestamp_ms));
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
  DebugUart_Printf("stick r=%d p=%d y=%d t=%u raw=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                   rc.roll,
                   rc.pitch,
                   rc.yaw,
                   rc.throttle,
                   rc.ch[0],
                   rc.ch[1],
                   rc.ch[2],
                   rc.ch[3],
                   rc.ch[4],
                   rc.ch[5],
                   rc.ch[6],
                   rc.ch[7],
                   rc.ch[8],
                   rc.ch[9],
                   rc.ch[10],
                   rc.ch[11],
                   rc.ch[12],
                   rc.ch[13],
                   rc.ch[14],
                   rc.ch[15]);
}

static void print_rcmap(void)
{
  DebugUart_WriteLine("rcmap order=AETR roll=CH1 pitch=CH2 throttle=CH3 yaw=CH4 arm=CH5 baro=CH6 raw_min=172 raw_mid=992 raw_max=1811");
}

static void print_batt(void)
{
  battery_status_t batt = Topic_GetBattery();
  uint32_t vadc_mv = ((uint32_t)batt.adc_raw * BOARD_ADC_REF_MV) / BOARD_ADC_MAX_COUNTS;
  DebugUart_Printf("batt raw=%u adc=%lumV voltage=%umV cells=%u percent=%u low=%u critical=%u\r\n",
                   batt.adc_raw,
                   (unsigned long)vadc_mv,
                   batt.voltage_mv,
                   BOARD_BATTERY_CELLS,
                   batt.percent,
                   batt.low ? 1U : 0U,
                   batt.critical ? 1U : 0U);
}

static uint32_t vbat_mv_to_raw(uint32_t vbat_mv)
{
  uint32_t vadc_mv = (vbat_mv * BOARD_BATTERY_DIVIDER_DEN) / BOARD_BATTERY_DIVIDER_NUM;
  return (vadc_mv * BOARD_ADC_MAX_COUNTS) / BOARD_ADC_REF_MV;
}

static void print_battdiag(void)
{
  battery_status_t batt = Topic_GetBattery();
  uint32_t vadc_mv = ((uint32_t)batt.adc_raw * BOARD_ADC_REF_MV) / BOARD_ADC_MAX_COUNTS;

  DebugUart_WriteLine("battdiag path=BAT-R29(100k)-ADC_BATT/PA4-R28(10k)-GND");
  DebugUart_Printf("battdiag ratio=%lu/%lu cells=%u vref=%umV adcmax=%u\r\n",
                   (unsigned long)BOARD_BATTERY_DIVIDER_NUM,
                   (unsigned long)BOARD_BATTERY_DIVIDER_DEN,
                   BOARD_BATTERY_CELLS,
                   BOARD_ADC_REF_MV,
                   BOARD_ADC_MAX_COUNTS);
  DebugUart_Printf("battdiag expected_raw empty=%lu low=%lu full=%lu\r\n",
                   (unsigned long)vbat_mv_to_raw(BOARD_BATT_EMPTY_MV),
                   (unsigned long)vbat_mv_to_raw(BOARD_BATT_LOW_MV),
                   (unsigned long)vbat_mv_to_raw(BOARD_BATT_FULL_MV));
  DebugUart_Printf("battdiag now raw=%u adc=%lumV voltage=%umV\r\n",
                   batt.adc_raw,
                   (unsigned long)vadc_mv,
                   batt.voltage_mv);
  DebugUart_Printf("battdiag regs smpr2=0x%08lX sqr3=0x%08lX cr2=0x%08lX sr=0x%08lX gpioa_crl=0x%08lX\r\n",
                   (unsigned long)ADC1->SMPR2,
                   (unsigned long)ADC1->SQR3,
                   (unsigned long)ADC1->CR2,
                   (unsigned long)ADC1->SR,
                   (unsigned long)GPIOA->CRL);
}

static void print_motormap(void)
{
  DebugUart_WriteLine("motormap QuadX nose-forward:");
  DebugUart_WriteLine("motormap layout: M4 front-left, M2 front-right, M3 rear-left, M1 rear-right");
  DebugUart_WriteLine("motormap output: M1=PA8/CN6 M2=PA11/CN4 M3=PB6/CN3 M4=PB7/CN1");
  DebugUart_WriteLine("motormap spin: standard M1/M4=CW, M2/M3=CCW viewed from top");
  DebugUart_WriteLine("motormap correction: right-low -> M1/M2 up, nose-low -> M2/M4 up");
}

static void print_control(void)
{
  flight_status_t st = Topic_GetStatus();
  attitude_t att = Topic_GetAttitude();
  imu_sample_t imu = Topic_GetImu();
  int8_t yawdir = ControllerAttitude_GetYawGyroDirection();
  float trim_roll = 0.0f;
  float trim_pitch = 0.0f;
  EstimatorAttitude_GetTrim(&trim_roll, &trim_pitch);

  DebugUart_Printf("control status arm=%u fs=%u flags=0x%04X rc=%u imu=%u imu_age=%lums yawdir=%d\r\n",
                   st.armed ? 1U : 0U,
                   st.failsafe ? 1U : 0U,
                   st.failsafe_flags,
                   st.rc_ok ? 1U : 0U,
                   st.imu_ok ? 1U : 0U,
                   (unsigned long)(HAL_GetTick() - imu.timestamp_ms),
                   yawdir);
  DebugUart_Printf("control sp_cd r=%ld p=%ld yawrate_cdps=%ld thr=%u baro=%u\r\n",
                   (long)deg_to_cdeg(st.setpoint.roll_deg),
                   (long)deg_to_cdeg(st.setpoint.pitch_deg),
                   (long)deg_to_cdeg(st.setpoint.yaw_rate_dps),
                   st.setpoint.throttle_permille,
                   st.setpoint.baro_hold ? 1U : 0U);
  DebugUart_Printf("control att_cd r=%ld p=%ld y=%ld healthy=%u\r\n",
                   (long)deg_to_cdeg(att.roll_deg),
                   (long)deg_to_cdeg(att.pitch_deg),
                   (long)deg_to_cdeg(att.yaw_deg),
                   att.healthy ? 1U : 0U);
  DebugUart_Printf("control trim_cd r=%ld p=%ld\r\n",
                   (long)deg_to_cdeg(trim_roll),
                   (long)deg_to_cdeg(trim_pitch));
  DebugUart_Printf("control out_milli r=%ld p=%ld y=%ld alt=%d\r\n",
                   (long)float_to_milli(st.control.roll),
                   (long)float_to_milli(st.control.pitch),
                   (long)float_to_milli(st.control.yaw),
                   st.control.altitude_permille);
  DebugUart_Printf("control mot_permille M1=%ld M2=%ld M3=%ld M4=%ld\r\n",
                   (long)duty_to_permille(st.motor[0]),
                   (long)duty_to_permille(st.motor[1]),
                   (long)duty_to_permille(st.motor[2]),
                   (long)duty_to_permille(st.motor[3]));
}

static void cmd_mixcheck(char *roll_s, char *pitch_s, char *yaw_s, char *thr_s)
{
  control_output_t control = {0};
  float motor[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  int throttle = 200;

  if (roll_s != NULL)
  {
    control.roll = (float)atoi(roll_s) / 1000.0f;
  }
  if (pitch_s != NULL)
  {
    control.pitch = (float)atoi(pitch_s) / 1000.0f;
  }
  if (yaw_s != NULL)
  {
    control.yaw = (float)atoi(yaw_s) / 1000.0f;
  }
  if (thr_s != NULL)
  {
    throttle = atoi(thr_s);
  }
  if (throttle < 0)
  {
    throttle = 0;
  }
  if (throttle > 1000)
  {
    throttle = 1000;
  }

  MixerQuad_Mix((uint16_t)throttle, &control, motor);
  DebugUart_Printf("mixcheck in_milli r=%ld p=%ld y=%ld thr=%d\r\n",
                   (long)float_to_milli(control.roll),
                   (long)float_to_milli(control.pitch),
                   (long)float_to_milli(control.yaw),
                   throttle);
  DebugUart_Printf("mixcheck M1=%ld M2=%ld M3=%ld M4=%ld\r\n",
                   (long)duty_to_permille(motor[0]),
                   (long)duty_to_permille(motor[1]),
                   (long)duty_to_permille(motor[2]),
                   (long)duty_to_permille(motor[3]));
  DebugUart_WriteLine("mixcheck +roll=>M3/M4 up, +pitch=>M1/M3 up, +yaw=>M2/M3 up");
}

static void cmd_yawdir(char *arg1)
{
  if (arg1 != NULL)
  {
    if ((strcmp(arg1, "normal") == 0) || (strcmp(arg1, "1") == 0))
    {
      ControllerAttitude_SetYawGyroDirection(1);
    }
    else if ((strcmp(arg1, "reverse") == 0) || (strcmp(arg1, "-1") == 0))
    {
      ControllerAttitude_SetYawGyroDirection(-1);
    }
    else
    {
      DebugUart_WriteLine("usage: yawdir normal|reverse");
      return;
    }
  }

  DebugUart_Printf("yawdir=%s (%d)\r\n",
                   (ControllerAttitude_GetYawGyroDirection() < 0) ? "reverse" : "normal",
                   ControllerAttitude_GetYawGyroDirection());
}

static void cmd_motoridle(char *arg1)
{
  if (arg1 != NULL)
  {
    int value = atoi(arg1);
    if (value < 0)
    {
      value = 0;
    }
    MixerQuad_SetMotorIdlePermille((uint16_t)value);
  }
  DebugUart_Printf("motoridle=%u permille\r\n", MixerQuad_GetMotorIdlePermille());
}

static void cmd_motormax(char *arg1)
{
  if (arg1 != NULL)
  {
    int value = atoi(arg1);
    if (value < 0)
    {
      value = 0;
    }
    MixerQuad_SetMotorMaxPermille((uint16_t)value);
  }
  DebugUart_Printf("motormax=%u permille\r\n", MixerQuad_GetMotorMaxPermille());
}

static void cmd_trim(char *roll_s, char *pitch_s)
{
  if ((roll_s != NULL) && (pitch_s != NULL))
  {
    EstimatorAttitude_SetTrim((float)atoi(roll_s) / 100.0f,
                              (float)atoi(pitch_s) / 100.0f);
  }
  float roll = 0.0f;
  float pitch = 0.0f;
  EstimatorAttitude_GetTrim(&roll, &pitch);
  DebugUart_Printf("trim roll_cd=%ld pitch_cd=%ld\r\n",
                   (long)deg_to_cdeg(roll),
                   (long)deg_to_cdeg(pitch));
}

static void cmd_leveltrim(void)
{
  attitude_t att = Topic_GetAttitude();
  float trim_roll = 0.0f;
  float trim_pitch = 0.0f;
  EstimatorAttitude_GetTrim(&trim_roll, &trim_pitch);
  EstimatorAttitude_SetTrim(trim_roll + att.roll_deg, trim_pitch + att.pitch_deg);
  cmd_trim(NULL, NULL);
}

static void reset_attitude_after_cal(bool ok)
{
  if (ok)
  {
    EstimatorAttitude_SetTrim(0.0f, 0.0f);
    EstimatorAttitude_Init();
  }
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
    DebugUart_WriteLine("usage: motor unlock|unlock bench|stop|<1-4> <permille>");
    return;
  }

  if (strcmp(arg1, "unlock") == 0)
  {
    bool bench = (arg2 != NULL) && (strcmp(arg2, "bench") == 0);
    if (bench)
    {
      Safety_MotorTestBenchUnlock(BOARD_MOTOR_TEST_WINDOW_MS);
    }
    else
    {
      Safety_MotorTestUnlock(BOARD_MOTOR_TEST_WINDOW_MS);
    }
    if (Safety_CanMotorTest())
    {
      DebugUart_Printf("motor test unlocked bench=%u\r\n", bench ? 1U : 0U);
    }
    else
    {
      DebugUart_WriteLine("motor test unlock denied: disarm, valid RC, no failsafe, battery ok");
    }
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
  else if (strcmp(cmd, "clock") == 0)
  {
    print_clock();
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
  else if (strcmp(cmd, "rcmap") == 0)
  {
    print_rcmap();
  }
  else if (strcmp(cmd, "batt") == 0)
  {
    print_batt();
  }
  else if ((strcmp(cmd, "battdiag") == 0) || (strcmp(cmd, "adc") == 0))
  {
    print_battdiag();
  }
  else if (strcmp(cmd, "motormap") == 0)
  {
    print_motormap();
  }
  else if ((strcmp(cmd, "control") == 0) || (strcmp(cmd, "ctrl") == 0))
  {
    print_control();
  }
  else if (strcmp(cmd, "yawdir") == 0)
  {
    cmd_yawdir(a1);
  }
  else if (strcmp(cmd, "mixcheck") == 0)
  {
    cmd_mixcheck(a1, a2, a3, a4);
  }
  else if (strcmp(cmd, "motoridle") == 0)
  {
    cmd_motoridle(a1);
  }
  else if (strcmp(cmd, "motormax") == 0)
  {
    cmd_motormax(a1);
  }
  else if (strcmp(cmd, "trim") == 0)
  {
    cmd_trim(a1, a2);
  }
  else if (strcmp(cmd, "leveltrim") == 0)
  {
    cmd_leveltrim();
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
  else if (strcmp(cmd, "gyrocal") == 0)
  {
    Safety_RequestDisarm();
    DebugUart_WriteLine("gyrocal: keep aircraft still");
    bool ok = Mpu6050_CalibrateGyro(200U);
    reset_attitude_after_cal(ok);
    DebugUart_Printf("gyrocal=%u\r\n", ok ? 1U : 0U);
  }
  else if (strcmp(cmd, "acccal") == 0)
  {
    Safety_RequestDisarm();
    DebugUart_WriteLine("acccal: level aircraft and keep still");
    bool ok = Mpu6050_CalibrateAccel(200U);
    reset_attitude_after_cal(ok);
    DebugUart_Printf("acccal=%u\r\n", ok ? 1U : 0U);
  }
  else if (strcmp(cmd, "imucal") == 0)
  {
    Safety_RequestDisarm();
    DebugUart_WriteLine("imucal: level aircraft and keep still");
    bool ok = Mpu6050_CalibrateImu(200U);
    reset_attitude_after_cal(ok);
    DebugUart_Printf("imucal=%u\r\n", ok ? 1U : 0U);
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
  bool skip_lf = false;

  DebugUart_Write("> ");
  for (;;)
  {
    if (!DebugUart_ReadByte(&ch, 20U))
    {
      continue;
    }

    if ((ch == '\n') && skip_lf)
    {
      skip_lf = false;
      continue;
    }
    skip_lf = (ch == '\r');

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
    }
  }
}
