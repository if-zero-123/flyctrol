#ifndef APP_ESTIMATOR_ALTITUDE_H
#define APP_ESTIMATOR_ALTITUDE_H

#include "flight_types.h"

void EstimatorAltitude_Init(void);
void EstimatorAltitude_ResetDynamic(void);
void EstimatorAltitude_PredictImu(const imu_sample_t *imu,
                                  const attitude_t *attitude,
                                  float dt_s);
void EstimatorAltitude_Update(const baro_sample_t *baro, baro_sample_t *out);

#endif /* APP_ESTIMATOR_ALTITUDE_H */
