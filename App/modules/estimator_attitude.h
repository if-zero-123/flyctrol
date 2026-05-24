#ifndef APP_ESTIMATOR_ATTITUDE_H
#define APP_ESTIMATOR_ATTITUDE_H

#include "flight_types.h"

void EstimatorAttitude_Init(void);
void EstimatorAttitude_Update(const imu_sample_t *imu, float dt_s, attitude_t *out);

#endif /* APP_ESTIMATOR_ATTITUDE_H */
