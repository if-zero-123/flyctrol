#include "controller_altitude.h"

#include <string.h>

#include "board_config.h"

static bool s_active;
static bool s_velocity_control;
static controller_altitude_state_t s_state;
static int32_t s_hold_altitude_cm;
static int32_t s_takeoff_origin_altitude_cm;
static int16_t s_stick_reference_permille;
static int16_t s_throttle_base_permille;
static float s_integrator_permille;
static int16_t s_last_output_permille;
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

static bool climb_overspeed(int32_t target_velocity_cms, int16_t measured_velocity_cms)
{
  int32_t allowed_velocity_cms = target_velocity_cms;
  if (allowed_velocity_cms < 0)
  {
    allowed_velocity_cms = 0;
  }
  allowed_velocity_cms += BOARD_ALT_ASCENT_OVERSPEED_MARGIN_CMS;

  return (((int32_t)measured_velocity_cms > allowed_velocity_cms) &&
          (measured_velocity_cms > BOARD_BARO_VEL_DEADBAND_CMS));
}

static int32_t apply_climb_overspeed_brake(int32_t desired_total,
                                           int32_t target_velocity_cms,
                                           int16_t measured_velocity_cms,
                                           bool *brake_active)
{
  if (brake_active != 0)
  {
    *brake_active = false;
  }

  if (climb_overspeed(target_velocity_cms, measured_velocity_cms))
  {
    int32_t brake_total = (int32_t)s_throttle_base_permille -
                          (int32_t)BOARD_ALT_ASCENT_BRAKE_PERMILLE;
    if (s_integrator_permille > 0.0f)
    {
      s_integrator_permille = 0.0f;
    }
    if (desired_total > brake_total)
    {
      desired_total = brake_total;
    }
    if (brake_active != 0)
    {
      *brake_active = true;
    }
  }

  return clamp_output_permille(desired_total);
}

static void learn_hover_throttle(bool in_deadband, int32_t altitude_error_cm, int16_t measured_velocity_cms)
{
  if ((s_state != CONTROLLER_ALTITUDE_STATE_FLYING) || !in_deadband)
  {
    return;
  }
  if ((abs_i32(altitude_error_cm) > (BOARD_ALT_HOLD_DB_CM * 2)) ||
      (abs_i32(measured_velocity_cms) > BOARD_ALT_HOVER_LEARN_VEL_CMS))
  {
    return;
  }

  if ((s_integrator_permille > 1.0f) &&
      (s_throttle_base_permille < (int16_t)BOARD_ALT_BASE_MAX_PERMILLE))
  {
    s_throttle_base_permille++;
    s_integrator_permille -= 1.0f;
  }
  else if ((s_integrator_permille < -1.0f) &&
           (s_throttle_base_permille > (int16_t)BOARD_MOTOR_IDLE_PERMILLE))
  {
    s_throttle_base_permille--;
    s_integrator_permille += 1.0f;
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
}

void ControllerAltitude_Init(void)
{
  ControllerAltitude_Reset();
}

void ControllerAltitude_Reset(void)
{
  s_active = false;
  s_velocity_control = false;
  s_state = CONTROLLER_ALTITUDE_STATE_INACTIVE;
  s_hold_altitude_cm = 0;
  s_takeoff_origin_altitude_cm = 0;
  s_stick_reference_permille = 0;
  s_throttle_base_permille = 0;
  s_integrator_permille = 0.0f;
  s_last_output_permille = 0;
  s_last_baro_timestamp_ms = 0U;
  memset(&s_debug, 0, sizeof(s_debug));
}

int16_t ControllerAltitude_Update(const baro_sample_t *baro,
                                  const control_setpoint_t *setpoint,
                                  bool active,
                                  float dt_s)
{
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
    s_state = CONTROLLER_ALTITUDE_STATE_GROUND_IDLE;
    s_hold_altitude_cm = baro->altitude_cm;
    s_takeoff_origin_altitude_cm = baro->altitude_cm;
    s_stick_reference_permille = (int16_t)BOARD_ALT_STICK_CENTER_PERMILLE;
    s_throttle_base_permille = clamp_i16(BOARD_ALT_HOVER_THRUST_PERMILLE,
                                         BOARD_MOTOR_IDLE_PERMILLE,
                                         BOARD_ALT_BASE_MAX_PERMILLE);
    s_integrator_permille = 0.0f;
    s_last_output_permille = (int16_t)BOARD_MOTOR_IDLE_PERMILLE;
    s_last_baro_timestamp_ms = 0U;
  }

  if (!just_activated && (baro->timestamp_ms == s_last_baro_timestamp_ms))
  {
    return s_last_output_permille;
  }

  float baro_dt_s = dt_s;
  if ((baro_dt_s <= 0.0f) || (baro_dt_s > 0.150f))
  {
    baro_dt_s = 0.025f;
  }
  if (!just_activated && (baro->timestamp_ms > s_last_baro_timestamp_ms))
  {
    baro_dt_s = (float)(baro->timestamp_ms - s_last_baro_timestamp_ms) / 1000.0f;
    baro_dt_s = clampf_local(baro_dt_s, 0.010f, 0.150f);
  }
  s_last_baro_timestamp_ms = baro->timestamp_ms;

  bool above_takeoff = stick_above_takeoff_threshold(setpoint->throttle_permille);
  bool in_deadband = stick_in_hold_deadband(setpoint->throttle_permille);
  int32_t target_velocity_cms = 0;
  int32_t altitude_error_cm = 0;

  if (s_state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE)
  {
    s_hold_altitude_cm = baro->altitude_cm;
    s_takeoff_origin_altitude_cm = baro->altitude_cm;
    s_velocity_control = false;
    s_integrator_permille = 0.0f;
    s_last_output_permille = (int16_t)BOARD_MOTOR_IDLE_PERMILLE;
    if (!above_takeoff)
    {
      publish_debug(0, 0, baro->velocity_cms, 0.0f);
      return s_last_output_permille;
    }
    s_state = CONTROLLER_ALTITUDE_STATE_TAKEOFF;
  }

  if (s_state == CONTROLLER_ALTITUDE_STATE_TAKEOFF)
  {
    int32_t takeoff_altitude_cm = baro->altitude_cm - s_takeoff_origin_altitude_cm;
    bool liftoff_detected = (takeoff_altitude_cm >= BOARD_ALT_LIFTOFF_ALT_CM) ||
                            (baro->velocity_cms >= BOARD_ALT_LIFTOFF_VEL_CMS);
    bool takeoff_committed = s_last_output_permille >= (int16_t)BOARD_ALT_BASE_MIN_PERMILLE;
    if (!above_takeoff && !liftoff_detected && !takeoff_committed)
    {
      s_state = CONTROLLER_ALTITUDE_STATE_GROUND_IDLE;
      s_hold_altitude_cm = baro->altitude_cm;
      s_integrator_permille = 0.0f;
      s_last_output_permille = (int16_t)BOARD_MOTOR_IDLE_PERMILLE;
      publish_debug(0, 0, baro->velocity_cms, 0.0f);
      return s_last_output_permille;
    }

    if (liftoff_detected || (!above_takeoff && takeoff_committed))
    {
      s_throttle_base_permille = clamp_i16(s_last_output_permille,
                                           BOARD_ALT_BASE_MIN_PERMILLE,
                                           BOARD_ALT_BASE_MAX_PERMILLE);
      s_state = CONTROLLER_ALTITUDE_STATE_FLYING;
      s_hold_altitude_cm = baro->altitude_cm;
      s_velocity_control = false;
      s_integrator_permille = 0.0f;
    }
    else
    {
      int32_t measured_velocity_cms = baro->velocity_cms;
      if (measured_velocity_cms < 0)
      {
        measured_velocity_cms = 0;
      }
      target_velocity_cms = stick_to_target_velocity_cms(setpoint->throttle_permille,
                                                         BOARD_ALT_TAKEOFF_CLIMB_MAX_CMS,
                                                         0);
      if (target_velocity_cms < 0)
      {
        target_velocity_cms = 0;
      }
      int32_t velocity_error = target_velocity_cms - measured_velocity_cms;
      float correction = BOARD_ALT_VEL_P * (float)velocity_error;
      correction = clampf_local(correction, 0.0f, (float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE);
      int32_t desired_total = (int32_t)BOARD_ALT_HOVER_THRUST_PERMILLE +
                              (target_velocity_cms * (int32_t)BOARD_ALT_TAKEOFF_THRUST_PER_CMS) +
                              (int32_t)correction;
      bool brake_active = false;
      desired_total = apply_climb_overspeed_brake(desired_total,
                                                  target_velocity_cms,
                                                  baro->velocity_cms,
                                                  &brake_active);
      if (brake_active)
      {
        s_last_output_permille = (int16_t)desired_total;
      }
      else
      {
        (void)slew_output_toward(desired_total,
                                 BOARD_ALT_TAKEOFF_SLEW_PER_SAMPLE,
                                 BOARD_ALT_OUTPUT_SLEW_DOWN_PER_SAMPLE);
      }
      s_velocity_control = true;
      s_hold_altitude_cm = baro->altitude_cm;
      publish_debug(0, target_velocity_cms, baro->velocity_cms, correction);
      return s_last_output_permille;
    }
  }

  target_velocity_cms = stick_to_target_velocity_cms(setpoint->throttle_permille,
                                                     BOARD_ALT_STICK_CLIMB_MAX_CMS,
                                                     BOARD_ALT_STICK_DESCEND_MAX_CMS);
  if (!in_deadband)
  {
    s_hold_altitude_cm = baro->altitude_cm;
    s_velocity_control = true;
  }
  else
  {
    if (s_velocity_control)
    {
      s_hold_altitude_cm = baro->altitude_cm;
      s_velocity_control = false;
      s_integrator_permille = 0.0f;
    }

    altitude_error_cm = s_hold_altitude_cm - baro->altitude_cm;
    if (abs_i32(altitude_error_cm) <= BOARD_ALT_HOLD_DB_CM)
    {
      altitude_error_cm = 0;
    }
    target_velocity_cms = (int32_t)((float)altitude_error_cm * BOARD_ALT_HOLD_POS_P);
    target_velocity_cms = clamp_i32(target_velocity_cms,
                                    -BOARD_ALT_HOLD_MAX_VEL_CMS,
                                    BOARD_ALT_HOLD_MAX_VEL_CMS);
  }

  int32_t velocity_error = target_velocity_cms - baro->velocity_cms;
  float p_term = BOARD_ALT_VEL_P * (float)velocity_error;
  bool brake_active = climb_overspeed(target_velocity_cms, baro->velocity_cms);
  float output_before_i = p_term + s_integrator_permille;
  bool saturated_high = (output_before_i > ((float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE * 0.90f)) && (velocity_error > 0);
  bool saturated_low = (output_before_i < (-(float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE * 0.90f)) && (velocity_error < 0);

  if (!brake_active && !saturated_high && !saturated_low)
  {
    s_integrator_permille += BOARD_ALT_VEL_I * (float)velocity_error * baro_dt_s;
    s_integrator_permille = clampf_local(s_integrator_permille,
                                         -BOARD_ALT_I_LIMIT_PERMILLE,
                                         BOARD_ALT_I_LIMIT_PERMILLE);
  }
  else if (brake_active && (s_integrator_permille > 0.0f))
  {
    s_integrator_permille = 0.0f;
  }

  float correction = p_term + s_integrator_permille;
  correction = clampf_local(correction,
                            -(float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE,
                            (float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE);

  int32_t desired_total = (int32_t)s_throttle_base_permille + (int32_t)correction;
  desired_total = apply_climb_overspeed_brake(desired_total,
                                              target_velocity_cms,
                                              baro->velocity_cms,
                                              &brake_active);
  if (brake_active)
  {
    s_last_output_permille = (int16_t)desired_total;
  }
  else
  {
    (void)slew_output_toward(desired_total,
                             BOARD_ALT_OUTPUT_SLEW_UP_PER_SAMPLE,
                             BOARD_ALT_OUTPUT_SLEW_DOWN_PER_SAMPLE);
  }

  learn_hover_throttle(in_deadband, altitude_error_cm, baro->velocity_cms);
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
