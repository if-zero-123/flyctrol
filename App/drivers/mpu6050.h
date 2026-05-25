#ifndef APP_MPU6050_H
#define APP_MPU6050_H

#include <stdbool.h>
#include "flight_types.h"

bool Mpu6050_Init(void);
bool Mpu6050_CalibrateGyro(uint16_t samples);
bool Mpu6050_CalibrateAccel(uint16_t samples);
bool Mpu6050_CalibrateImu(uint16_t samples);
void Mpu6050_ResetFilters(void);
bool Mpu6050_Read(imu_sample_t *out);
bool Mpu6050_IsHealthy(void);
void Mpu6050_GetReadStats(uint32_t *ok_count, uint32_t *fail_count);

#endif /* APP_MPU6050_H */
