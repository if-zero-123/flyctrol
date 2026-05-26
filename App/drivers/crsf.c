#include "crsf.h"

#if defined(__GNUC__)
#pragma GCC optimize ("Os")
#endif

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "board_config.h"
#include "flight_types.h"
#include "main.h"
#include "topic.h"

#define CRSF_DMA_BUFFER_SIZE 256U
#define CRSF_MAX_FRAME_SIZE  64U
#define CRSF_ADDRESS_FC      0xC8U
#define CRSF_ADDRESS_RADIO   0xEAU
#define CRSF_TYPE_RC_CHANNELS_PACKED 0x16U

static uint8_t s_dma_buf[CRSF_DMA_BUFFER_SIZE];
static uint16_t s_read_pos;
static uint8_t s_frame[CRSF_MAX_FRAME_SIZE];
static uint8_t s_frame_pos;
static uint8_t s_frame_len;
static TaskHandle_t s_task_handle;
static uint32_t s_last_frame_ms;

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t data)
{
  crc ^= data;
  for (uint8_t i = 0; i < 8U; i++)
  {
    crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0xD5U) : (uint8_t)(crc << 1);
  }
  return crc;
}

static uint8_t crsf_crc(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;
  for (uint8_t i = 0; i < len; i++)
  {
    crc = crc8_dvb_s2(crc, data[i]);
  }
  return crc;
}

static bool valid_address(uint8_t address)
{
  return (address == CRSF_ADDRESS_FC) || (address == CRSF_ADDRESS_RADIO) || (address == 0xECU) || (address == 0xEEU);
}

static int16_t normalize_stick(uint16_t raw)
{
  int32_t v = ((int32_t)raw - 992) * 1000 / 820;
  if ((v > -BOARD_RC_DEADBAND) && (v < BOARD_RC_DEADBAND))
  {
    v = 0;
  }
  if (v < -1000)
  {
    v = -1000;
  }
  if (v > 1000)
  {
    v = 1000;
  }
  return (int16_t)v;
}

static uint16_t normalize_throttle(uint16_t raw)
{
  int32_t v = ((int32_t)raw - 172) * 1000 / (1811 - 172);
  if (v < 0)
  {
    v = 0;
  }
  if (v > 1000)
  {
    v = 1000;
  }
  return (uint16_t)v;
}

static void unpack_channels(const uint8_t *p, uint16_t ch[16])
{
  ch[0] = ((uint16_t)p[0] | ((uint16_t)p[1] << 8)) & 0x07FFU;
  ch[1] = (((uint16_t)p[1] >> 3) | ((uint16_t)p[2] << 5)) & 0x07FFU;
  ch[2] = (((uint16_t)p[2] >> 6) | ((uint16_t)p[3] << 2) | ((uint16_t)p[4] << 10)) & 0x07FFU;
  ch[3] = (((uint16_t)p[4] >> 1) | ((uint16_t)p[5] << 7)) & 0x07FFU;
  ch[4] = (((uint16_t)p[5] >> 4) | ((uint16_t)p[6] << 4)) & 0x07FFU;
  ch[5] = (((uint16_t)p[6] >> 7) | ((uint16_t)p[7] << 1) | ((uint16_t)p[8] << 9)) & 0x07FFU;
  ch[6] = (((uint16_t)p[8] >> 2) | ((uint16_t)p[9] << 6)) & 0x07FFU;
  ch[7] = (((uint16_t)p[9] >> 5) | ((uint16_t)p[10] << 3)) & 0x07FFU;
  ch[8] = ((uint16_t)p[11] | ((uint16_t)p[12] << 8)) & 0x07FFU;
  ch[9] = (((uint16_t)p[12] >> 3) | ((uint16_t)p[13] << 5)) & 0x07FFU;
  ch[10] = (((uint16_t)p[13] >> 6) | ((uint16_t)p[14] << 2) | ((uint16_t)p[15] << 10)) & 0x07FFU;
  ch[11] = (((uint16_t)p[15] >> 1) | ((uint16_t)p[16] << 7)) & 0x07FFU;
  ch[12] = (((uint16_t)p[16] >> 4) | ((uint16_t)p[17] << 4)) & 0x07FFU;
  ch[13] = (((uint16_t)p[17] >> 7) | ((uint16_t)p[18] << 1) | ((uint16_t)p[19] << 9)) & 0x07FFU;
  ch[14] = (((uint16_t)p[19] >> 2) | ((uint16_t)p[20] << 6)) & 0x07FFU;
  ch[15] = (((uint16_t)p[20] >> 5) | ((uint16_t)p[21] << 3)) & 0x07FFU;
}

static void publish_rc(const uint16_t raw[16])
{
  app_rc_t rc = Topic_GetRc();
  for (uint8_t i = 0; i < APP_RC_CHANNELS; i++)
  {
    rc.ch[i] = (int16_t)raw[i];
  }
  rc.roll = normalize_stick(raw[0]);
  rc.pitch = normalize_stick(raw[1]);
  rc.throttle = normalize_throttle(raw[2]);
  rc.yaw = normalize_stick(raw[3]);
  rc.arm_switch = raw[4] > 1500U;
  rc.angle_mode = true;
  rc.baro_mode = raw[5] > 1500U;
  rc.connected = true;
  rc.failsafe = false;
  rc.last_update_ms = HAL_GetTick();
  s_last_frame_ms = rc.last_update_ms;
  Topic_PublishRc(&rc);
}

static void handle_frame(void)
{
  uint8_t len = s_frame[1];
  uint8_t expected_crc = s_frame[1U + len];
  uint8_t crc = crsf_crc(&s_frame[2], (uint8_t)(len - 1U));
  if (crc != expected_crc)
  {
    return;
  }

  uint8_t type = s_frame[2];
  const uint8_t *payload = &s_frame[3];
  uint8_t payload_len = (uint8_t)(len - 2U);

  if ((type == CRSF_TYPE_RC_CHANNELS_PACKED) && (payload_len == 22U))
  {
    uint16_t ch[16];
    unpack_channels(payload, ch);
    publish_rc(ch);
  }
}

static void parser_byte(uint8_t b)
{
  if (s_frame_pos == 0U)
  {
    if (!valid_address(b))
    {
      return;
    }
    s_frame[s_frame_pos++] = b;
    return;
  }

  if (s_frame_pos == 1U)
  {
    if ((b < 2U) || (b > (CRSF_MAX_FRAME_SIZE - 2U)))
    {
      s_frame_pos = 0U;
      return;
    }
    s_frame_len = b;
    s_frame[s_frame_pos++] = b;
    return;
  }

  s_frame[s_frame_pos++] = b;
  if (s_frame_pos >= (uint8_t)(s_frame_len + 2U))
  {
    handle_frame();
    s_frame_pos = 0U;
  }
}

bool Crsf_Init(void)
{
  s_read_pos = 0U;
  s_frame_pos = 0U;
  s_frame_len = 0U;
  s_last_frame_ms = 0U;
  memset(s_dma_buf, 0, sizeof(s_dma_buf));

  if (HAL_UART_Receive_DMA(&huart2, s_dma_buf, CRSF_DMA_BUFFER_SIZE) != HAL_OK)
  {
    return false;
  }
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  return true;
}

void Crsf_SetTaskHandle(void *task_handle)
{
  s_task_handle = (TaskHandle_t)task_handle;
}

void Crsf_ProcessRx(void)
{
  if (huart2.hdmarx == 0)
  {
    return;
  }

  uint16_t write_pos = (uint16_t)(CRSF_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
  while (s_read_pos != write_pos)
  {
    parser_byte(s_dma_buf[s_read_pos]);
    s_read_pos++;
    if (s_read_pos >= CRSF_DMA_BUFFER_SIZE)
    {
      s_read_pos = 0U;
    }
  }
}

void Crsf_UpdateLinkState(void)
{
  app_rc_t rc = Topic_GetRc();
  uint32_t now = HAL_GetTick();
  if ((s_last_frame_ms == 0U) || ((now - s_last_frame_ms) > BOARD_RC_TIMEOUT_MS))
  {
    rc.connected = false;
    rc.failsafe = true;
    Topic_PublishRc(&rc);
  }
}

void Crsf_OnUsartIdleIsr(UART_HandleTypeDef *huart)
{
  if ((huart == &huart2) && (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) &&
      (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) != RESET))
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    BaseType_t higher_woken = pdFALSE;
    if (s_task_handle != 0)
    {
      vTaskNotifyGiveFromISR(s_task_handle, &higher_woken);
      portYIELD_FROM_ISR(higher_woken);
    }
  }
}
