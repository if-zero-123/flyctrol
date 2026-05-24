#ifndef APP_TOPIC_H
#define APP_TOPIC_H

#include "flight_types.h"

void Topic_Init(void);

void Topic_PublishRc(const app_rc_t *value);
app_rc_t Topic_GetRc(void);

void Topic_PublishImu(const imu_sample_t *value);
imu_sample_t Topic_GetImu(void);

void Topic_PublishAttitude(const attitude_t *value);
attitude_t Topic_GetAttitude(void);

void Topic_PublishBaro(const baro_sample_t *value);
baro_sample_t Topic_GetBaro(void);

void Topic_PublishBattery(const battery_status_t *value);
battery_status_t Topic_GetBattery(void);

void Topic_PublishStatus(const flight_status_t *value);
flight_status_t Topic_GetStatus(void);

#endif /* APP_TOPIC_H */
