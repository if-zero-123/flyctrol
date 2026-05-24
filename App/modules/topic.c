#include "topic.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static app_rc_t s_rc;
static imu_sample_t s_imu;
static attitude_t s_attitude;
static baro_sample_t s_baro;
static battery_status_t s_battery;
static flight_status_t s_status;

static void copy_in(void *dst, const void *src, size_t size)
{
  taskENTER_CRITICAL();
  memcpy(dst, src, size);
  taskEXIT_CRITICAL();
}

static void copy_out(void *dst, const void *src, size_t size)
{
  taskENTER_CRITICAL();
  memcpy(dst, src, size);
  taskEXIT_CRITICAL();
}

void Topic_Init(void)
{
  memset(&s_rc, 0, sizeof(s_rc));
  memset(&s_imu, 0, sizeof(s_imu));
  memset(&s_attitude, 0, sizeof(s_attitude));
  memset(&s_baro, 0, sizeof(s_baro));
  memset(&s_battery, 0, sizeof(s_battery));
  memset(&s_status, 0, sizeof(s_status));
}

void Topic_PublishRc(const app_rc_t *value) { if (value != NULL) copy_in(&s_rc, value, sizeof(s_rc)); }
app_rc_t Topic_GetRc(void) { app_rc_t v; copy_out(&v, &s_rc, sizeof(v)); return v; }

void Topic_PublishImu(const imu_sample_t *value) { if (value != NULL) copy_in(&s_imu, value, sizeof(s_imu)); }
imu_sample_t Topic_GetImu(void) { imu_sample_t v; copy_out(&v, &s_imu, sizeof(v)); return v; }

void Topic_PublishAttitude(const attitude_t *value) { if (value != NULL) copy_in(&s_attitude, value, sizeof(s_attitude)); }
attitude_t Topic_GetAttitude(void) { attitude_t v; copy_out(&v, &s_attitude, sizeof(v)); return v; }

void Topic_PublishBaro(const baro_sample_t *value) { if (value != NULL) copy_in(&s_baro, value, sizeof(s_baro)); }
baro_sample_t Topic_GetBaro(void) { baro_sample_t v; copy_out(&v, &s_baro, sizeof(v)); return v; }

void Topic_PublishBattery(const battery_status_t *value) { if (value != NULL) copy_in(&s_battery, value, sizeof(s_battery)); }
battery_status_t Topic_GetBattery(void) { battery_status_t v; copy_out(&v, &s_battery, sizeof(v)); return v; }

void Topic_PublishStatus(const flight_status_t *value) { if (value != NULL) copy_in(&s_status, value, sizeof(s_status)); }
flight_status_t Topic_GetStatus(void) { flight_status_t v; copy_out(&v, &s_status, sizeof(v)); return v; }
