#ifndef APP_ESTIMATOR_ATTITUDE_H
#define APP_ESTIMATOR_ATTITUDE_H

#include "flight_types.h"

void EstimatorAttitude_Init(void);
void EstimatorAttitude_Update(const imu_sample_t *imu, float dt_s, attitude_t *out);
void EstimatorAttitude_SetTrim(float roll_deg, float pitch_deg);
void EstimatorAttitude_GetTrim(float *roll_deg, float *pitch_deg);

#endif /* APP_ESTIMATOR_ATTITUDE_H */
