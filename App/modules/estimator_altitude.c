#include "estimator_altitude.h"

#include <stddef.h>

static bool s_has_baseline;
static int32_t s_baseline_pa;
static int32_t s_filtered_cm;

void EstimatorAltitude_Init(void)
{
  s_has_baseline = false;
  s_baseline_pa = 101325;
  s_filtered_cm = 0;
}

void EstimatorAltitude_Update(const baro_sample_t *baro, baro_sample_t *out)
{
  if ((baro == NULL) || (out == NULL))
  {
    return;
  }

  *out = *baro;
  if (!baro->healthy || (baro->pressure_pa <= 0))
  {
    out->healthy = false;
    return;
  }

  if (!s_has_baseline)
  {
    s_baseline_pa = baro->pressure_pa;
    s_filtered_cm = 0;
    s_has_baseline = true;
  }

  int32_t raw_cm = ((s_baseline_pa - baro->pressure_pa) * 25) / 3;
  s_filtered_cm = ((s_filtered_cm * 7) + raw_cm) / 8;
  out->altitude_cm = s_filtered_cm;
}
