# NAZE32 有刷小四轴自写飞控完整方案

适配硬件: 当前 PCB  
目标平台: STM32F103C8T6/CBT6 + STM32 HAL + FreeRTOS  
目标功能: ELRS/CRSF 接收、自稳飞行、BMP280 定高、四路有刷电机 PWM、电池检测、安全解锁和失控保护

> 重要原则: 第一版先做“能安全自稳飞”，再做“可用定高”。不要把高度保持放在姿态稳定之前。

---

## 1. 当前硬件连接总表

### 1.1 MCU 与时钟

| 功能 | STM32F103 引脚 | 网络/器件 | 说明 |
| --- | --- | --- | --- |
| HSE_IN | PD0 / OSC_IN / U9.5 | X2.3 | 8 MHz 谐振器/晶振 |
| HSE_OUT | PD1 / OSC_OUT / U9.6 | X2.1 | 8 MHz 谐振器/晶振 |
| NRST | U9.7 | R23 10k 上拉 3.3V | 复位 |
| BOOT0 | U9.44 | R18 10k 下拉，SW1 接 3.3V | 串口 Bootloader |
| BOOT1 | PB2 / U9.20 | R19 10k 下拉 | 默认从 Flash 启动 |
| SWDIO | PA13 / U9.34 | CN2.4 | SWD 下载 |
| SWCLK | PA14 / U9.37 | CN2.3 | SWD 下载 |

### 1.2 串口

| 功能 | STM32 引脚 | 外设 | 连接 | 用途 |
| --- | --- | --- | --- | --- |
| USART1_TX | PA9 / U9.30 | USART1 | CH340C RXD / U4.3 | 调试串口、日志 |
| USART1_RX | PA10 / U9.31 | USART1 | CH340C TXD / U4.2 | 调试命令 |
| USART2_TX | PA2 / U9.12 | USART2 | U5.4 / TX_CH3 -> ELRS RX | CRSF 回传/遥测 |
| USART2_RX | PA3 / U9.13 | USART2 | U5.3 / RX_CH4 <- ELRS TX | CRSF 接收 |

ELRS 接收机接口 `U5`:

| U5 引脚 | 网络 | 用途 |
| --- | --- | --- |
| U5.1 | GND | 接收机地 |
| U5.2 | IN_5V | 接收机供电 |
| U5.3 | RX_CH4 / PA3 | 飞控 RX，接 ELRS TX |
| U5.4 | TX_CH3 / PA2 | 飞控 TX，接 ELRS RX |

CRSF 串口参数:

```text
USART2 baud = 420000
Data = 8 bit
Parity = none
Stop = 1
```

### 1.3 I2C 传感器

| 功能 | STM32 引脚 | 外设 | 连接 |
| --- | --- | --- | --- |
| I2C2_SCL | PB10 / U9.21 | I2C2 | BMP280 SCK, MPU6050 SCL |
| I2C2_SDA | PB11 / U9.22 | I2C2 | BMP280 SDI, MPU6050 SDA |

I2C 上拉:

```text
R27 SCL -> VCC_SEN = 4.7k
R26 SDA -> VCC_SEN = 4.7k
```

I2C 设备:

| 器件 | 地址 | 关键连接 | 说明 |
| --- | --- | --- | --- |
| MPU6050 | 0x68 | AD0 下拉到 GND | 姿态核心传感器 |
| BMP280 | 通常 0x76 | SDO 下拉到 GND, CSB 上拉到 VCC_SEN | 气压计定高 |

### 1.4 电机 PWM

当前 PCB 是有刷电机低边 MOS 驱动，不是无刷三相驱动。

| 逻辑电机 | STM32 引脚 | 定时器建议 | PCB 接口 | MOS | 说明 |
| --- | --- | --- | --- | --- | --- |
| MOTOR1 | PA8 | TIM1_CH1 | CN6 | Q5 | 有刷电机低边 |
| MOTOR2 | PA11 | TIM1_CH4 | CN4 | Q3 | 有刷电机低边 |
| MOTOR3 | PB6 | TIM4_CH1 | CN3 | Q2 | 有刷电机低边 |
| MOTOR4 | PB7 | TIM4_CH2 | CN1 | Q1 | 有刷电机低边 |

每路结构:

```text
BAT -> 电机正极
电机负极 -> MOS Drain
MOS Source -> GND
MOS Gate -> STM32 PWM
Gate -> 100k -> GND
```

PWM 建议:

```text
PWM frequency = 16 kHz
Duty 0% = 电机停止
Duty 100% = 满功率
未解锁 = 全部 0%
失控 = 全部 0%
```

### 1.5 电池电压检测

| 功能 | STM32 引脚 | 连接 | 公式 |
| --- | --- | --- | --- |
| ADC_BATT | PA4 / U9.14 | BAT -> R29 100k -> PA4 -> R28 10k -> GND | BAT = ADC_V * 11 |

ADC 输入电压:

```text
Vadc = Vbat * 10k / (100k + 10k) = Vbat / 11
Vbat = Vadc * 11
```

### 1.6 LED

| LED | STM32 引脚 | 逻辑 |
| --- | --- | --- |
| LED1 / LED5 绿色 | PB3 / U9.39 | 低电平亮 |
| LED2 / LED4 蓝色 | PB4 / U9.40 | 低电平亮 |
| LED6 红色 | 不受 MCU 控制 | 3.3V 电源指示常亮 |

---

## 2. CubeMX / HAL 外设配置

### 2.1 时钟

```text
HSE = 8 MHz
PLL = x9
SYSCLK = 72 MHz
APB1 = 36 MHz
APB2 = 72 MHz
```

如果你的芯片/板子 HSE 不稳定，先用 HSI 跑通调试，再回到 HSE。

### 2.2 外设配置

| 外设 | 配置 |
| --- | --- |
| USART1 | 115200, 8N1, TX/RX, 可选 DMA TX |
| USART2 | 420000, 8N1, RX DMA circular + IDLE 中断 |
| I2C2 | 400 kHz, PB10/PB11 |
| ADC1 | PA4, 单通道, DMA 可选 |
| TIM1 | PWM, CH1 PA8, CH4 PA11, 16 kHz |
| TIM4 | PWM, CH1 PB6, CH2 PB7, 16 kHz |
| GPIO | PB3/PB4 LED 输出 |
| FreeRTOS | 开启，控制任务最高优先级 |

### 2.3 PWM 定时器计算

72 MHz 下 16 kHz PWM:

```text
Timer clock = 72 MHz
Prescaler = 0
Period = 72,000,000 / 16,000 - 1 = 4499
CCR = duty * 4500
```

PWM 输出函数:

```c
static inline uint16_t motor_duty_to_ccr(float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    return (uint16_t)(duty * 4500.0f);
}
```

---

## 3. 工程目录结构

建议在 CubeMX 生成的工程里新增 `App/`:

```text
Core/
  Inc/
  Src/
App/
  board/
    board_config.h
    board_time.c
    motor_pwm.c
    battery_adc.c
    led.c
  drivers/
    mpu6050.c
    mpu6050.h
    bmp280.c
    bmp280.h
    crsf.c
    crsf.h
  modules/
    topic.c
    commander.c
    estimator_attitude.c
    estimator_altitude.c
    pid.c
    controller_attitude.c
    controller_altitude.c
    mixer_quad.c
    safety.c
    telemetry.c
  app_main.c
  app_tasks.c
  flight_types.h
```

---

## 4. 板级配置文件

文件: `App/board/board_config.h`

```c
#pragma once

#include "stm32f1xx_hal.h"

// UART
#define BOARD_DEBUG_UART        huart1
#define BOARD_CRSF_UART         huart2

// I2C
#define BOARD_SENSOR_I2C        hi2c2

// Motors: physical PWM outputs
#define MOTOR_PWM_FREQ_HZ       16000U
#define MOTOR_PWM_PERIOD        4499U

// Physical motor channels on this PCB
#define MOTOR_PHY_1_TIMER       htim1
#define MOTOR_PHY_1_CHANNEL     TIM_CHANNEL_1   // PA8, CN6
#define MOTOR_PHY_2_TIMER       htim1
#define MOTOR_PHY_2_CHANNEL     TIM_CHANNEL_4   // PA11, CN4
#define MOTOR_PHY_3_TIMER       htim4
#define MOTOR_PHY_3_CHANNEL     TIM_CHANNEL_1   // PB6, CN3
#define MOTOR_PHY_4_TIMER       htim4
#define MOTOR_PHY_4_CHANNEL     TIM_CHANNEL_2   // PB7, CN1

// Logical motor mapping.
// If wire length forces swapping motors, only change this table, not timer code.
typedef enum {
    MOTOR_PHYSICAL_PA8_CN6 = 0,
    MOTOR_PHYSICAL_PA11_CN4,
    MOTOR_PHYSICAL_PB6_CN3,
    MOTOR_PHYSICAL_PB7_CN1,
} motor_physical_t;

// Default hardware mapping:
// logical M1->PA8/CN6, M2->PA11/CN4, M3->PB6/CN3, M4->PB7/CN1
#define LOGICAL_MOTOR1_PHY      MOTOR_PHYSICAL_PA8_CN6
#define LOGICAL_MOTOR2_PHY      MOTOR_PHYSICAL_PA11_CN4
#define LOGICAL_MOTOR3_PHY      MOTOR_PHYSICAL_PB6_CN3
#define LOGICAL_MOTOR4_PHY      MOTOR_PHYSICAL_PB7_CN1

// Battery divider
#define BATTERY_DIVIDER_RATIO   11.0f
#define ADC_REF_VOLTAGE         3.3f
#define ADC_MAX_COUNTS          4095.0f

// CRSF
#define CRSF_BAUDRATE           420000U
#define CRSF_TIMEOUT_US         100000U

// Control loop
#define CONTROL_LOOP_HZ         500U
#define CONTROL_DT              (1.0f / (float)CONTROL_LOOP_HZ)
```

如果你要软件交换 1、4 电机，只改这里:

```c
#define LOGICAL_MOTOR1_PHY      MOTOR_PHYSICAL_PB7_CN1
#define LOGICAL_MOTOR4_PHY      MOTOR_PHYSICAL_PA8_CN6
```

---

## 5. 核心数据结构

文件: `App/flight_types.h`

```c
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float x;
    float y;
    float z;
} vec3f_t;

typedef struct {
    vec3f_t gyro_rad_s;
    vec3f_t acc_g;
    uint32_t timestamp_us;
    bool healthy;
} imu_sample_t;

typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    vec3f_t body_rate_rad_s;
    uint32_t timestamp_us;
    bool valid;
} attitude_t;

typedef struct {
    uint16_t ch_us[16];       // 1000..2000
    bool frame_valid;
    bool failsafe;
    uint32_t last_frame_us;
} rc_input_t;

typedef struct {
    float roll_angle_sp_rad;
    float pitch_angle_sp_rad;
    float yaw_rate_sp_rad_s;
    float throttle;           // 0..1
    bool arm_switch;
    bool angle_mode;
    bool baro_mode;
} setpoint_t;

typedef struct {
    float altitude_m;
    float velocity_z_m_s;
    uint32_t timestamp_us;
    bool valid;
} altitude_estimate_t;

typedef struct {
    float voltage_v;
    bool low;
    bool valid;
} battery_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
} control_t;

typedef struct {
    float m[4];               // logical motor 1..4, 0..1
} motor_output_t;

typedef enum {
    FLIGHT_DISARMED = 0,
    FLIGHT_ARMED,
    FLIGHT_FAILSAFE,
    FLIGHT_ERROR,
} flight_state_t;
```

---

## 6. 数据传输结构

F103 上不要做复杂消息总线。推荐 latest-value topic。

文件: `App/modules/topic.h`

```c
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct {
    volatile uint32_t seq;
    volatile uint32_t timestamp_us;
} topic_header_t;

#define TOPIC_DECLARE(type, name) \
    typedef struct {              \
        volatile uint32_t seq;    \
        volatile uint32_t timestamp_us; \
        type data;                \
    } name

#define TOPIC_PUBLISH(topic_ptr, src_ptr, now_us) do { \
    (topic_ptr)->seq++;                                \
    (topic_ptr)->data = *(src_ptr);                    \
    (topic_ptr)->timestamp_us = (now_us);              \
    (topic_ptr)->seq++;                                \
} while (0)

#define TOPIC_READ(topic_ptr, dst_ptr) ({              \
    uint32_t s1, s2;                                   \
    do {                                               \
        s1 = (topic_ptr)->seq;                         \
        *(dst_ptr) = (topic_ptr)->data;                \
        s2 = (topic_ptr)->seq;                         \
    } while ((s1 != s2) || (s1 & 1U));                 \
    true;                                              \
})
```

全局 topics:

```c
TOPIC_DECLARE(rc_input_t, topic_rc_t);
TOPIC_DECLARE(altitude_estimate_t, topic_altitude_t);
TOPIC_DECLARE(battery_t, topic_battery_t);
TOPIC_DECLARE(flight_state_t, topic_state_t);

extern topic_rc_t g_topic_rc;
extern topic_altitude_t g_topic_altitude;
extern topic_battery_t g_topic_battery;
extern topic_state_t g_topic_state;
```

控制环不等待队列，只读取最新值。

---

## 7. FreeRTOS 任务设计

| 任务 | 优先级 | 频率 | 核心职责 |
| --- | --- | --- | --- |
| `StabilizerTask` | 最高 | 500 Hz 起步 | IMU、姿态解算、PID、混控、电机输出 |
| `CrsfTask` | 高 | DMA/IDLE 触发 | 解析 ELRS/CRSF，发布 RC |
| `SafetyTask` | 高 | 100 Hz | 解锁、失控、传感器健康、低压保护 |
| `BaroTask` | 中 | 25-50 Hz | BMP280、高度估计 |
| `BatteryTask` | 低 | 10 Hz | ADC 电池电压 |
| `TelemetryTask` | 最低 | 5-20 Hz | USART1 日志 |

原则:

```text
StabilizerTask 不 printf
StabilizerTask 不写 Flash
StabilizerTask 不等待串口
StabilizerTask 的 I2C timeout 必须很短
所有 failsafe 都优先输出电机 0
```

任务入口:

```c
void App_Start(void)
{
    Board_Init();
    Drivers_Init();
    Motors_SetAll(0.0f);

    xTaskCreate(StabilizerTask, "stab", 512, NULL, osPriorityRealtime, NULL);
    xTaskCreate(CrsfTask,       "crsf", 384, NULL, osPriorityHigh, NULL);
    xTaskCreate(SafetyTask,     "safe", 384, NULL, osPriorityHigh, NULL);
    xTaskCreate(BaroTask,       "baro", 384, NULL, osPriorityNormal, NULL);
    xTaskCreate(BatteryTask,    "batt", 256, NULL, osPriorityLow, NULL);
    xTaskCreate(TelemetryTask,  "telem",512, NULL, osPriorityLow, NULL);
}
```

---

## 8. 板级驱动框架

### 8.1 电机 PWM

文件: `App/board/motor_pwm.c`

```c
#include "board_config.h"
#include "tim.h"
#include "flight_types.h"

static float clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

static uint32_t duty_to_ccr(float duty)
{
    duty = clamp01(duty);
    return (uint32_t)(duty * (float)(MOTOR_PWM_PERIOD + 1U));
}

static void motor_write_physical(motor_physical_t phy, float duty)
{
    uint32_t ccr = duty_to_ccr(duty);

    switch (phy) {
    case MOTOR_PHYSICAL_PA8_CN6:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
        break;
    case MOTOR_PHYSICAL_PA11_CN4:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr);
        break;
    case MOTOR_PHYSICAL_PB6_CN3:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
        break;
    case MOTOR_PHYSICAL_PB7_CN1:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccr);
        break;
    default:
        break;
    }
}

void Motors_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    Motors_SetAll(0.0f);
}

void Motors_SetLogical(const motor_output_t *out)
{
    motor_write_physical(LOGICAL_MOTOR1_PHY, out->m[0]);
    motor_write_physical(LOGICAL_MOTOR2_PHY, out->m[1]);
    motor_write_physical(LOGICAL_MOTOR3_PHY, out->m[2]);
    motor_write_physical(LOGICAL_MOTOR4_PHY, out->m[3]);
}

void Motors_SetAll(float duty)
{
    motor_output_t out = { .m = { duty, duty, duty, duty } };
    Motors_SetLogical(&out);
}
```

### 8.2 电池 ADC

```c
float Battery_ReadVoltage(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 2);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    float vadc = ((float)raw / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE;
    return vadc * BATTERY_DIVIDER_RATIO;
}
```

### 8.3 LED

低电平亮:

```c
void Led1_Set(bool on)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Led2_Set(bool on)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
```

---

## 9. CRSF 接收机模块

### 9.1 CRSF 设计

USART2:

```text
420000 baud
DMA circular RX
IDLE line interrupt
```

处理流程:

```text
USART2 IDLE IRQ
  -> 记录 DMA 当前写位置
  -> 通知 CrsfTask

CrsfTask
  -> 从 ring buffer 取数据
  -> 查找 CRSF 帧
  -> CRC 校验
  -> 解析 RC_CHANNELS_PACKED
  -> 更新 rc_input_t topic
```

Failsafe:

```text
now_us - rc.last_frame_us > 100000 us
  -> rc.failsafe = true
  -> SafetyTask 进入 FAILSAFE
```

### 9.2 通道语义

建议:

| CRSF 通道 | 用途 |
| --- | --- |
| CH1 | Roll |
| CH2 | Pitch |
| CH3 | Throttle |
| CH4 | Yaw |
| CH5 | Arm |
| CH6 | Angle mode |
| CH7 | Baro mode |

缩放:

```c
float rc_norm(uint16_t us)
{
    float x = ((float)us - 1500.0f) / 500.0f;
    if (x < -1.0f) x = -1.0f;
    if (x >  1.0f) x =  1.0f;
    return x;
}

float rc_throttle(uint16_t us)
{
    float x = ((float)us - 1000.0f) / 1000.0f;
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x;
}
```

---

## 10. MPU6050 驱动与姿态估计

### 10.1 MPU6050 初始化

基本寄存器:

```text
PWR_MGMT_1 = 0x00
SMPLRT_DIV = 根据目标采样设置
CONFIG = DLPF 42Hz 或 98Hz 起步
GYRO_CONFIG = +/- 2000 dps 或 500 dps
ACCEL_CONFIG = +/- 4g 或 8g
```

建议第一版:

```text
gyro range = +/- 500 dps
accel range = +/- 4g
DLPF = 42Hz
control loop = 500Hz
```

校准:

```text
上电静止 2 秒
采样 1000 次 gyro
gyro_bias = average
accel 可先只做零偏近似，后面再六面校准
```

### 10.2 姿态估计

第一版推荐互补滤波:

```text
gyro 积分提供短期快速响应
acc 计算 roll/pitch 提供长期修正
yaw 只靠 gyro 积分，会漂，但 Angle 模式主要靠 roll/pitch
```

更新:

```c
roll_gyro  += gx * dt;
pitch_gyro += gy * dt;
yaw        += gz * dt;

roll_acc  = atan2f(acc_y, acc_z);
pitch_acc = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z));

roll  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
```

`alpha` 起步:

```text
alpha = 0.98
```

第二版可换 Mahony filter。

---

## 11. BMP280 高度估计

### 11.1 BMP280 初始化

建议:

```text
pressure oversampling = x4 或 x8
temperature oversampling = x1
IIR filter = 8 或 16
ODR = 25-50Hz
```

### 11.2 高度计算

使用相对高度:

```text
上电稳定后，记录 p0
altitude = 44330 * (1 - (p / p0)^(1/5.255))
```

滤波:

```text
alt_lpf = alt_lpf + k_alt * (alt_raw - alt_lpf)
vel_raw = (alt_lpf - alt_lpf_prev) / dt
vel_lpf = vel_lpf + k_vel * (vel_raw - vel_lpf)
```

建议:

```text
baro rate = 25Hz
alt LPF time constant = 0.5s
vel LPF time constant = 0.8s
```

注意:

```text
BMP280 必须盖透气海绵
不能被桨风直吹
定高只在姿态稳定后启用
```

---

## 12. Commander: 遥控器到目标值

输入:

```text
rc_input_t
flight_state_t
```

输出:

```text
setpoint_t
```

建议限制:

```text
max angle = 20 deg 起步
max yaw rate = 90 deg/s 起步
```

逻辑:

```c
setpoint.roll_angle_sp_rad  = rc_roll  * deg2rad(20.0f);
setpoint.pitch_angle_sp_rad = rc_pitch * deg2rad(20.0f);
setpoint.yaw_rate_sp_rad_s  = rc_yaw   * deg2rad(90.0f);
setpoint.throttle           = rc_throttle;
setpoint.arm_switch         = ch5 > 1500;
setpoint.angle_mode         = ch6 > 1500;
setpoint.baro_mode          = ch7 > 1500;
```

第一版可以强制只支持 Angle:

```text
if not angle_mode -> still use angle mode
```

---

## 13. 姿态控制器

### 13.1 级联结构

```text
角度外环:
angle_error -> target_rate

角速度内环:
target_rate - gyro_rate -> axis correction
```

### 13.2 PID 类型

```c
typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float last_error;
    float i_limit;
    float out_limit;
} pid_t;
```

更新:

```c
float PID_Update(pid_t *pid, float error, float dt, bool allow_integrator)
{
    if (allow_integrator) {
        pid->integrator += error * pid->ki * dt;
        if (pid->integrator > pid->i_limit) pid->integrator = pid->i_limit;
        if (pid->integrator < -pid->i_limit) pid->integrator = -pid->i_limit;
    }

    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    float out = pid->kp * error + pid->integrator + pid->kd * derivative;
    if (out > pid->out_limit) out = pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;
    return out;
}
```

更鲁棒的 D 项:

```text
优先对 gyro rate 做低通后作为 D，不要直接对 angle error 做高噪声微分。
```

### 13.3 初始控制参数

单位不是 Betaflight 的数值，需按你的实现调。

建议从很小开始:

```text
Angle P roll/pitch = 4.0
Rate P roll/pitch  = 0.08
Rate I roll/pitch  = 0.02
Rate D roll/pitch  = 0.001

Yaw rate P = 0.08
Yaw rate I = 0.01
Yaw rate D = 0
```

安全限制:

```text
axis correction limit = +/-0.25 起步
I term limit = +/-0.15 起步
```

---

## 14. 高度控制器

只在 `ANGLE + BARO + ARMED + altitude valid` 时启用。

### 14.1 激活逻辑

BARO 从关闭到开启瞬间:

```c
alt_target = current_altitude;
hover_throttle = current_throttle;
```

### 14.2 油门杆解释

定高开启后，油门杆不再是直接电机功率，而是爬升/下降意图。

```text
油门在激活点附近 deadband 内 -> 保持高度
油门高于 deadband -> 目标高度慢慢升
油门低于 deadband -> 目标高度慢慢降
```

建议:

```text
deadband = +/-0.08
max climb rate = +/-0.3 m/s 起步
```

### 14.3 级联高度控制

```text
alt_error = alt_target - altitude
vel_sp = Kp_alt * alt_error + pilot_climb_rate
vel_sp limited

vel_error = vel_sp - velocity_z
throttle_correction = PID_vel(vel_error)

throttle = hover_throttle + throttle_correction
```

倾角补偿:

```c
float tilt_comp = cosf(roll) * cosf(pitch);
if (tilt_comp < 0.7f) tilt_comp = 0.7f;
throttle = throttle / tilt_comp;
```

限幅:

```text
throttle_correction limit = +/-0.15 起步
roll/pitch > 35 deg -> 退出或冻结定高修正
baro invalid -> 退出定高
```

---

## 15. 四轴混控

逻辑电机编号建议使用 Betaflight Quad X:

```text
机头朝前:

    M4       M2
      \   /
       \ /
       / \
      /   \
    M3       M1
```

初始混控:

```c
motor_output_t Mixer_QuadX(float throttle, float roll, float pitch, float yaw)
{
    motor_output_t out;

    out.m[0] = throttle - roll - pitch + yaw; // M1
    out.m[1] = throttle - roll + pitch - yaw; // M2
    out.m[2] = throttle + roll - pitch - yaw; // M3
    out.m[3] = throttle + roll + pitch + yaw; // M4

    Mixer_Normalize(&out);
    return out;
}
```

归一化:

```c
void Mixer_Normalize(motor_output_t *out)
{
    float min = out->m[0];
    float max = out->m[0];
    for (int i = 1; i < 4; i++) {
        if (out->m[i] < min) min = out->m[i];
        if (out->m[i] > max) max = out->m[i];
    }

    if (min < 0.0f) {
        for (int i = 0; i < 4; i++) out->m[i] -= min;
    }

    max = out->m[0];
    for (int i = 1; i < 4; i++) {
        if (out->m[i] > max) max = out->m[i];
    }

    if (max > 1.0f) {
        for (int i = 0; i < 4; i++) out->m[i] /= max;
    }

    for (int i = 0; i < 4; i++) {
        if (out->m[i] < 0.0f) out->m[i] = 0.0f;
        if (out->m[i] > 1.0f) out->m[i] = 1.0f;
    }
}
```

无桨验证:

```text
机身右倾 -> 左侧电机应增加
机身前倾 -> 后侧电机应增加
机头顺时针偏航 -> 反向电机组合应补偿
```

如果补偿方向反，先改控制轴符号，不要急着改 PID。

---

## 16. 安全状态机

状态:

```text
BOOT
CALIBRATING
DISARMED
ARMED
FAILSAFE
ERROR
```

解锁条件:

```text
RC 有效，最近 100ms 内有 CRSF 帧
油门低于 5%
IMU healthy
gyro 校准完成
roll/pitch 绝对值小于 30 deg
电池电压合理
没有 I2C 连续错误
```

失控条件:

```text
CRSF 超过 100ms 无有效帧
IMU 读取失败连续超过阈值
控制环超时
姿态估计 invalid
```

失控动作:

```text
Motors_SetAll(0)
清 PID 积分
状态进入 FAILSAFE
LED 快闪
```

未解锁:

```text
Motors_SetAll(0)
PID integrator reset
```

---

## 17. StabilizerTask 伪代码

```c
void StabilizerTask(void *argument)
{
    TickType_t last = xTaskGetTickCount();
    AttitudeEstimator_Init();
    Controllers_Init();

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ));

        uint32_t now = micros();

        imu_sample_t imu;
        if (!MPU6050_ReadScaled(&imu)) {
            Safety_ReportImuError();
            Motors_SetAll(0.0f);
            continue;
        }

        attitude_t att;
        AttitudeEstimator_Update(&imu, CONTROL_DT, &att);

        rc_input_t rc;
        altitude_estimate_t alt;
        battery_t batt;
        flight_state_t state;
        TOPIC_READ(&g_topic_rc, &rc);
        TOPIC_READ(&g_topic_altitude, &alt);
        TOPIC_READ(&g_topic_battery, &batt);
        TOPIC_READ(&g_topic_state, &state);

        setpoint_t sp;
        Commander_Update(&rc, state, &sp);

        if (state != FLIGHT_ARMED || rc.failsafe) {
            Controllers_Reset();
            Motors_SetAll(0.0f);
            continue;
        }

        control_t ctrl;
        Controller_AttitudeUpdate(&sp, &att, CONTROL_DT, &ctrl);

        float throttle = sp.throttle;
        if (sp.baro_mode && sp.angle_mode && alt.valid) {
            throttle = Controller_AltitudeUpdate(&sp, &att, &alt, CONTROL_DT);
        }

        motor_output_t motors = Mixer_QuadX(throttle, ctrl.roll, ctrl.pitch, ctrl.yaw);
        Motors_SetLogical(&motors);
    }
}
```

---

## 18. 调试与开发顺序

### 阶段 1: 板级验证

```text
1. LED1/LED2 闪烁
2. USART1 printf 正常
3. TIM1/TIM4 四路 PWM 输出 16kHz
4. 单独控制四个电机 duty 0..30%
5. ADC PA4 读电池电压
6. I2C 扫描得到 0x68 和 0x76
7. 读 MPU6050 WHO_AM_I
8. 读 BMP280 chip ID
9. USART2 收到 CRSF 通道
```

### 阶段 2: 安全和手动油门

```text
1. 未解锁电机永远 0
2. Arm 开关生效
3. 油门不低禁止解锁
4. CRSF 丢失立即停机
5. 油门控制四电机同步转
```

### 阶段 3: 姿态估计

```text
1. 打印 roll/pitch/yaw
2. 手动倾斜板子，方向正确
3. 静止时 roll/pitch 稳定
4. gyro bias 校准有效
```

### 阶段 4: 自稳

```text
1. 无桨倾斜补偿方向验证
2. 小 P 值短暂离地
3. 调 Rate P
4. 加少量 Rate I
5. 最后加很少 D
```

### 阶段 5: 定高

```text
1. BMP280 盖海绵
2. 高度曲线稳定
3. Angle 飞稳后再启用 Baro
4. throttle correction 限制 +/-15%
5. climb rate 限制 +/-0.3m/s
```

---

## 19. 起步参数建议

### 姿态限制

```text
max angle = 20 deg
max yaw rate = 90 deg/s
control rate = 500Hz
motor pwm = 16kHz
```

### PID 初始值

这是给你自写代码的归一化起点，不是 Betaflight 数字。

```text
Angle roll/pitch P = 4.0

Rate roll/pitch:
P = 0.08
I = 0.02
D = 0.001
I limit = 0.15
Output limit = 0.25

Yaw:
P = 0.08
I = 0.01
D = 0
Output limit = 0.20
```

### 定高初始值

```text
Alt P = 0.8
Vel P = 0.25
Vel I = 0.05
Vel D = 0.00 起步
Throttle correction limit = +/-0.15
Climb rate limit = +/-0.3m/s
```

---

## 20. 必须避免的坑

```text
不要一开始上完整 EKF
不要控制环里 printf
不要控制环里等待串口
不要无 failsafe 测试电机
不要带桨调试电机顺序
不要先做定高再做自稳
不要让 BMP280 被桨风直吹
不要忽略电池电压下降对有刷电机推力的影响
```

---

## 21. 最小可飞版本定义

第一版能飞只需要:

```text
MPU6050 姿态
ELRS CRSF 接收
四路有刷 PWM
Angle 模式
解锁/失控保护
电池电压读取
```

不要把 BMP280 定高放进首飞必需项。  
首飞目标是: 松杆能回平，轻微扰动能自稳，不乱转，不失控。

---

## 22. 后续增强

第二版:

```text
Mahony filter 替代互补滤波
串口调参协议
Flash 保存参数
低压补偿 throttle boost
更好的 D-term 低通
```

第三版:

```text
高度 1D Kalman: state = [z, vz]
油门-加速度模型
悬停油门在线学习
黑匣子日志
```

---

## 23. 参考架构

这些项目只借鉴结构，不建议直接移植:

- PX4: 传感器 -> 估计 -> 控制 -> 分配 -> 输出的分层思想
- ArduPilot: 角度外环/角速度内环、高度/速度级联控制、安全状态机
- Betaflight: 快速 gyro PID loop、mixer、电机输出和 F1 小板经验
- INAV: ALT HOLD 作为模式修饰器，油门杆作为爬升/下降意图
- Crazyflie: FreeRTOS 下 stabilizer task + commander + estimator + controller + power distribution

