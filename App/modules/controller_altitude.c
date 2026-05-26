#include "controller_altitude.h"

#include <string.h>

#include "board_config.h"

static bool s_active;
static bool s_velocity_control;
static bool s_assist_reliable;
static bool s_baro_rejected;
static controller_altitude_state_t s_state;
static int32_t s_hold_altitude_cm;
static int32_t s_takeoff_origin_altitude_cm;
static int32_t s_last_baro_altitude_cm;
static int16_t s_stick_reference_permille;
static int16_t s_throttle_base_permille;
static int16_t s_last_output_permille;
static int16_t s_assist_correction_limit_permille;
static uint8_t s_baro_reject_samples;
static bool s_has_last_baro_altitude;
static uint32_t s_last_baro_timestamp_ms;
static controller_altitude_debug_t s_debug;

static int32_t abs_i32(int32_t v)
{
  return (v < 0) ? -v : v;
}

static float clampf_local(float v, float min_v, float max_v)
{
  if (v < min_v)
  {
    return min_v;
  }
  if (v > max_v)
  {
    return max_v;
  }
  return v;
}

static int32_t clamp_i32(int32_t v, int32_t min_v, int32_t max_v)
{
  if (v < min_v)
  {
    return min_v;
  }
  if (v > max_v)
  {
    return max_v;
  }
  return v;
}

static int16_t clamp_i16(int32_t v, int32_t min_v, int32_t max_v)
{
  return (int16_t)clamp_i32(v, min_v, max_v);
}

static int16_t float_to_i16(float v)
{
  if (v > 32767.0f)
  {
    return 32767;
  }
  if (v < -32768.0f)
  {
    return -32768;
  }
  return (int16_t)v;
}

static int32_t output_min_permille(void)
{
  return (int32_t)BOARD_MOTOR_IDLE_PERMILLE;
}

static int32_t output_max_permille(void)
{
  return (int32_t)BOARD_MOTOR_MAX_PERMILLE;
}

static int32_t clamp_output_permille(int32_t output_permille)
{
  return clamp_i32(output_permille, output_min_permille(), output_max_permille());
}

static int32_t stick_to_target_velocity_cms(uint16_t throttle_permille,
                                            int32_t climb_max_cms,
                                            int32_t descend_max_cms)
{
  const int32_t center = (int32_t)BOARD_ALT_STICK_CENTER_PERMILLE;
  const int32_t deadband = (int32_t)BOARD_ALT_HOLD_DB_PERMILLE;
  int32_t delta = (int32_t)throttle_permille - center;

  if (delta > deadband)
  {
    int32_t range = 1000 - center - deadband;
    if (range < 1)
    {
      range = 1;
    }
    return ((delta - deadband) * climb_max_cms) / range;
  }
  if (delta < -deadband)
  {
    int32_t range = center - deadband;
    if (range < 1)
    {
      range = 1;
    }
    return -(((-delta) - deadband) * descend_max_cms) / range;
  }
  return 0;
}

static bool stick_above_takeoff_threshold(uint16_t throttle_permille)
{
  return ((int32_t)throttle_permille - (int32_t)BOARD_ALT_STICK_CENTER_PERMILLE) >
         (int32_t)BOARD_ALT_HOLD_DB_PERMILLE;
}

static bool stick_in_hold_deadband(uint16_t throttle_permille)
{
  int32_t delta = (int32_t)throttle_permille - (int32_t)BOARD_ALT_STICK_CENTER_PERMILLE;
  return abs_i32(delta) <= (int32_t)BOARD_ALT_HOLD_DB_PERMILLE;
}

static int32_t stick_to_pilot_base_permille(uint16_t throttle_permille)
{
  const int32_t center = (int32_t)BOARD_ALT_STICK_CENTER_PERMILLE;
  const int32_t deadband = (int32_t)BOARD_ALT_HOLD_DB_PERMILLE;
  const int32_t base = (int32_t)s_throttle_base_permille;
  int32_t delta = (int32_t)throttle_permille - center;

  if (delta > deadband)
  {
    int32_t range = 1000 - center - deadband;
    int32_t headroom = output_max_permille() - base;
    if (range < 1)
    {
      range = 1;
    }
    if (headroom < 0)
    {
      headroom = 0;
    }
    return clamp_output_permille(base + (((delta - deadband) * headroom) / range));
  }

  if (delta < -deadband)
  {
    int32_t range = center - deadband;
    int32_t low_limit = base - (int32_t)BOARD_ALT_TOY_DESCEND_HEADROOM_PERMILLE;
    if (low_limit < (int32_t)BOARD_ALT_BASE_MIN_PERMILLE)
    {
      low_limit = (int32_t)BOARD_ALT_BASE_MIN_PERMILLE;
    }
    if (range < 1)
    {
      range = 1;
    }
    return clamp_output_permille(base - (((-delta - deadband) * (base - low_limit)) / range));
  }

  return clamp_output_permille(base);
}

static int16_t slew_output_toward(int32_t desired_total, int32_t up_step, int32_t down_step)
{
  desired_total = clamp_output_permille(desired_total);
  int32_t output_delta = desired_total - (int32_t)s_last_output_permille;
  if (output_delta > up_step)
  {
    output_delta = up_step;
  }
  else if (output_delta < -down_step)
  {
    output_delta = -down_step;
  }
  s_last_output_permille = (int16_t)clamp_output_permille((int32_t)s_last_output_permille + output_delta);
  return s_last_output_permille;
}

static bool update_baro_rejection(const baro_sample_t *baro, int32_t pilot_base_permille)
{
  bool sample_bad = false;

  if (s_has_last_baro_altitude &&
      (abs_i32(baro->altitude_cm - s_last_baro_altitude_cm) > BOARD_ALT_TOY_ALT_JUMP_REJECT_CM))
  {
    sample_bad = true;
  }
  if (abs_i32((int32_t)baro->velocity_cms) > BOARD_ALT_TOY_VEL_REJECT_CMS)
  {
    sample_bad = true;
  }
  if (((pilot_base_permille >= (int32_t)s_throttle_base_permille) ||
       (s_last_output_permille >= s_throttle_base_permille)) &&
      (baro->velocity_cms <= BOARD_ALT_TOY_NEG_VEL_REJECT_CMS))
  {
    sample_bad = true;
  }

  if (sample_bad)
  {
    s_baro_reject_samples = BOARD_ALT_TOY_BAD_BARO_HOLD_SAMPLES;
  }
  else if (s_baro_reject_samples > 0U)
  {
    s_baro_reject_samples--;
  }

  s_last_baro_altitude_cm = baro->altitude_cm;
  s_has_last_baro_altitude = true;
  return s_baro_reject_samples > 0U;
}

static float toy_baro_correction(bool in_deadband,
                                 bool baro_rejected,
                                 int32_t altitude_error_cm,
                                 int16_t measured_velocity_cms,
                                 int16_t *correction_limit_permille)
{
  int16_t limit = baro_rejected ? (int16_t)BOARD_ALT_TOY_BAD_BARO_LIMIT_PERMILLE :
                                  (int16_t)BOARD_ALT_TOY_CORR_LIMIT_PERMILLE;
  float correction = 0.0f;

  if (correction_limit_permille != 0)
  {
    *correction_limit_permille = limit;
  }
  if (!in_deadband)
  {
    return 0.0f;
  }

  correction = ((float)altitude_error_cm * BOARD_ALT_TOY_HOLD_POS_P);
  if (!baro_rejected)
  {
    correction -= (float)measured_velocity_cms * BOARD_ALT_TOY_VEL_DAMPING;
  }
  else if (correction > 0.0f)
  {
    correction = 0.0f;
  }

  return clampf_local(correction, -(float)limit, (float)limit);
}

static void learn_hover_throttle(bool in_deadband,
                                 bool baro_rejected,
                                 int32_t altitude_error_cm,
                                 int16_t measured_velocity_cms,
                                 float correction)
{
  if ((s_state != CONTROLLER_ALTITUDE_STATE_TOY_ASSIST) || !in_deadband || baro_rejected)
  {
    return;
  }
  if ((abs_i32(altitude_error_cm) > (BOARD_ALT_HOLD_DB_CM * 2)) ||
      (abs_i32(measured_velocity_cms) > BOARD_ALT_HOVER_LEARN_VEL_CMS))
  {
    return;
  }

  if ((correction > 1.0f) &&
      (s_throttle_base_permille < (int16_t)BOARD_ALT_TOY_HOVER_MAX_PERMILLE))
  {
    s_throttle_base_permille++;
  }
  else if ((correction < -1.0f) &&
           (s_throttle_base_permille > (int16_t)BOARD_ALT_TOY_HOVER_MIN_PERMILLE))
  {
    s_throttle_base_permille--;
  }
}

static void publish_debug(int32_t altitude_error_cm,
                          int32_t target_velocity_cms,
                          int16_t velocity_cms,
                          float correction)
{
  s_debug.active = s_active;
  s_debug.velocity_control = s_velocity_control;
  s_debug.state = s_state;
  s_debug.hold_altitude_cm = s_hold_altitude_cm;
  s_debug.altitude_error_cm = clamp_i16(altitude_error_cm, -32768, 32767);
  s_debug.velocity_cms = velocity_cms;
  s_debug.target_velocity_cms = clamp_i16(target_velocity_cms, -32768, 32767);
  s_debug.stick_reference_permille = s_stick_reference_permille;
  s_debug.throttle_base_permille = s_throttle_base_permille;
  s_debug.correction_permille = float_to_i16(correction);
  s_debug.output_permille = s_last_output_permille;
  s_debug.assist_reliable = s_assist_reliable;
  s_debug.baro_rejected = s_baro_rejected;
  s_debug.assist_correction_limit_permille = s_assist_correction_limit_permille;
}

void ControllerAltitude_Init(void)
{
  ControllerAltitude_Reset();
}

void ControllerAltitude_Reset(void)
{
  s_active = false;
  s_velocity_control = false;
  s_assist_reliable = false;
  s_baro_rejected = false;
  s_state = CONTROLLER_ALTITUDE_STATE_INACTIVE;
  s_hold_altitude_cm = 0;
  s_takeoff_origin_altitude_cm = 0;
  s_last_baro_altitude_cm = 0;
  s_stick_reference_permille = 0;
  s_throttle_base_permille = 0;
  s_last_output_permille = 0;
  s_assist_correction_limit_permille = 0;
  s_baro_reject_samples = 0U;
  s_has_last_baro_altitude = false;
  s_last_baro_timestamp_ms = 0U;
  memset(&s_debug, 0, sizeof(s_debug));
}

int16_t ControllerAltitude_Update(const baro_sample_t *baro,
                                  const control_setpoint_t *setpoint,
                                  bool active,
                                  float dt_s)
{
  (void)dt_s;

  if ((baro == 0) || (setpoint == 0) || !active || !setpoint->baro_hold || !baro->healthy)
  {
    ControllerAltitude_Reset();
    return 0;
  }

  bool just_activated = false;
  if (!s_active)
  {
    s_active = true;
    just_activated = true;
    s_velocity_control = false;
    s_assist_reliable = true;
    s_baro_rejected = false;
    s_state = CONTROLLER_ALTITUDE_STATE_GROUND_IDLE;
    s_hold_altitude_cm = baro->altitude_cm;
    s_takeoff_origin_altitude_cm = baro->altitude_cm;
    s_last_baro_altitude_cm = baro->altitude_cm;
    s_stick_reference_permille = (int16_t)BOARD_ALT_STICK_CENTER_PERMILLE;
    s_throttle_base_permille = clamp_i16(BOARD_ALT_HOVER_THRUST_PERMILLE,
                                         BOARD_ALT_TOY_HOVER_MIN_PERMILLE,
                                         BOARD_ALT_TOY_HOVER_MAX_PERMILLE);
    s_last_output_permille = (int16_t)BOARD_MOTOR_IDLE_PERMILLE;
    s_assist_correction_limit_permille = (int16_t)BOARD_ALT_TOY_CORR_LIMIT_PERMILLE;
    s_baro_reject_samples = 0U;
    s_has_last_baro_altitude = true;
    s_last_baro_timestamp_ms = 0U;
  }

  if (!just_activated && (baro->timestamp_ms == s_last_baro_timestamp_ms))
  {
    return s_last_output_permille;
  }
  s_last_baro_timestamp_ms = baro->timestamp_ms;

  bool above_takeoff = stick_above_takeoff_threshold(setpoint->throttle_permille);
  bool in_deadband = stick_in_hold_deadband(setpoint->throttle_permille);
  int32_t target_velocity_cms = stick_to_target_velocity_cms(setpoint->throttle_permille,
                                                             BOARD_ALT_TOY_CLIMB_MAX_CMS,
                                                             BOARD_ALT_TOY_DESCEND_MAX_CMS);
  int32_t altitude_error_cm = 0;
  float correction = 0.0f;

  if (s_state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE)
  {
    s_hold_altitude_cm = baro->altitude_cm;
    s_takeoff_origin_altitude_cm = baro->altitude_cm;
    s_last_baro_altitude_cm = baro->altitude_cm;
    s_velocity_control = false;
    s_assist_reliable = true;
    s_baro_rejected = false;
    s_baro_reject_samples = 0U;
    s_last_output_permille = (int16_t)BOARD_MOTOR_IDLE_PERMILLE;
    s_assist_correction_limit_permille = (int16_t)BOARD_ALT_TOY_CORR_LIMIT_PERMILLE;
    if (!above_takeoff)
    {
      publish_debug(0, 0, baro->velocity_cms, 0.0f);
      return s_last_output_permille;
    }
    s_state = CONTROLLER_ALTITUDE_STATE_TOY_TAKEOFF;
  }

  if (s_state == CONTROLLER_ALTITUDE_STATE_TOY_TAKEOFF)
  {
    int32_t takeoff_altitude_cm = baro->altitude_cm - s_takeoff_origin_altitude_cm;
    bool liftoff_detected = (takeoff_altitude_cm >= BOARD_ALT_LIFTOFF_ALT_CM) ||
                            (baro->velocity_cms >= BOARD_ALT_LIFTOFF_VEL_CMS);
    bool takeoff_committed = s_last_output_permille >= (int16_t)BOARD_ALT_TOY_HOVER_MIN_PERMILLE;
    bool assist_ready = s_last_output_permille >=
                        (int16_t)(s_throttle_base_permille + BOARD_ALT_TOY_ASSIST_ENGAGE_HEADROOM_PERMILLE);

    if (!above_takeoff && !liftoff_detected && !takeoff_committed)
    {
      s_state = CONTROLLER_ALTITUDE_STATE_GROUND_IDLE;
      s_hold_altitude_cm = baro->altitude_cm;
      s_last_output_permille = (int16_t)BOARD_MOTOR_IDLE_PERMILLE;
      s_assist_reliable = true;
      s_baro_rejected = false;
      publish_debug(0, 0, baro->velocity_cms, 0.0f);
      return s_last_output_permille;
    }

    if (liftoff_detected || assist_ready || (!above_takeoff && takeoff_committed))
    {
      s_throttle_base_permille = clamp_i16(s_last_output_permille,
                                           BOARD_ALT_TOY_HOVER_MIN_PERMILLE,
                                           BOARD_ALT_TOY_HOVER_MAX_PERMILLE);
      s_state = CONTROLLER_ALTITUDE_STATE_TOY_ASSIST;
      s_hold_altitude_cm = baro->altitude_cm;
      s_velocity_control = !in_deadband;
    }
    else
    {
      int32_t desired_total = stick_to_pilot_base_permille(setpoint->throttle_permille);
      if (desired_total < (int32_t)s_throttle_base_permille)
      {
        desired_total = s_throttle_base_permille;
      }
      if (desired_total > (int32_t)BOARD_ALT_TOY_HOVER_MAX_PERMILLE)
      {
        desired_total = (int32_t)BOARD_ALT_TOY_HOVER_MAX_PERMILLE;
      }
      (void)slew_output_toward(desired_total,
                               BOARD_ALT_TAKEOFF_SLEW_PER_SAMPLE,
                               BOARD_ALT_OUTPUT_SLEW_DOWN_PER_SAMPLE);
      s_velocity_control = true;
      s_assist_reliable = true;
      s_baro_rejected = false;
      s_assist_correction_limit_permille = (int16_t)BOARD_ALT_TOY_BAD_BARO_LIMIT_PERMILLE;
      if (baro->velocity_cms <= BOARD_ALT_TOY_NEG_VEL_REJECT_CMS)
      {
        s_baro_rejected = true;
        s_assist_reliable = false;
      }
      publish_debug(0, target_velocity_cms, baro->velocity_cms, 0.0f);
      return s_last_output_permille;
    }
  }

  int32_t pilot_base = stick_to_pilot_base_permille(setpoint->throttle_permille);
  s_baro_rejected = update_baro_rejection(baro, pilot_base);
  s_assist_reliable = !s_baro_rejected;

  if (!in_deadband)
  {
    s_hold_altitude_cm = baro->altitude_cm;
    s_velocity_control = true;
    s_assist_correction_limit_permille = s_baro_rejected ?
                                        (int16_t)BOARD_ALT_TOY_BAD_BARO_LIMIT_PERMILLE :
                                        (int16_t)BOARD_ALT_TOY_CORR_LIMIT_PERMILLE;
    correction = 0.0f;
  }
  else
  {
    if (s_velocity_control)
    {
      s_hold_altitude_cm = baro->altitude_cm;
      s_velocity_control = false;
    }

    altitude_error_cm = s_hold_altitude_cm - baro->altitude_cm;
    if (abs_i32(altitude_error_cm) <= BOARD_ALT_HOLD_DB_CM)
    {
      altitude_error_cm = 0;
    }
    correction = toy_baro_correction(in_deadband,
                                     s_baro_rejected,
                                     altitude_error_cm,
                                     baro->velocity_cms,
                                     &s_assist_correction_limit_permille);
  }

  int32_t desired_total = pilot_base + (int32_t)correction;
  (void)slew_output_toward(desired_total,
                           BOARD_ALT_OUTPUT_SLEW_UP_PER_SAMPLE,
                           BOARD_ALT_OUTPUT_SLEW_DOWN_PER_SAMPLE);

  learn_hover_throttle(in_deadband,
                       s_baro_rejected,
                       altitude_error_cm,
                       baro->velocity_cms,
                       correction);
  publish_debug(altitude_error_cm, target_velocity_cms, baro->velocity_cms, correction);
  return s_last_output_permille;
}

void ControllerAltitude_GetDebug(controller_altitude_debug_t *out)
{
  if (out != 0)
  {
    *out = s_debug;
  }
}

bool ControllerAltitude_IsActive(void)
{
  return s_active;
}
