#include "controller_altitude.h"

#if defined(__GNUC__)
#pragma GCC optimize ("Os")
#endif

#include <string.h>

#include "board_config.h"

static bool s_active;
static bool s_velocity_control;
static bool s_assist_reliable;
static bool s_baro_rejected;
static controller_altitude_state_t s_state;
static int32_t s_hold_altitude_cm;
static int32_t s_takeoff_origin_altitude_cm;
static int16_t s_stick_reference_permille;
static int16_t s_hover_permille;
static int16_t s_base_permille;
static int16_t s_last_correction_permille;
static int16_t s_last_output_permille;
static float s_velocity_integrator_permille;
static float s_hover_learn_accum;
static uint16_t s_freeze_samples;
static uint8_t s_freeze_reason;
static mixer_feedback_t s_last_feedback;
static controller_altitude_debug_t s_debug;

static int32_t abs_i32(int32_t v)
{
  return (v < 0) ? -v : v;
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

static int32_t hover_min_permille(void)
{
  return (int32_t)BOARD_ALT_BASE_MIN_PERMILLE;
}

static int32_t hover_max_permille(void)
{
  return (int32_t)BOARD_ALT_BASE_MAX_PERMILLE;
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

static int32_t stick_to_takeoff_base_permille(uint16_t throttle_permille)
{
  const int32_t threshold = (int32_t)BOARD_ALT_TAKEOFF_THROTTLE;
  int32_t delta = (int32_t)throttle_permille - threshold;
  int32_t range = 1000 - threshold;
  int32_t headroom = (int32_t)BOARD_ALT_BASE_MAX_PERMILLE - (int32_t)BOARD_MOTOR_IDLE_PERMILLE;

  if (delta < 0)
  {
    delta = 0;
  }
  if (range < 1)
  {
    range = 1;
  }
  if (headroom < 0)
  {
    headroom = 0;
  }
  return clamp_output_permille((int32_t)BOARD_MOTOR_IDLE_PERMILLE +
                               ((delta * headroom) / range));
}

static int16_t slew_i16(int16_t current, int32_t desired, int32_t up_step, int32_t down_step)
{
  desired = clamp_output_permille(desired);
  int32_t delta = desired - (int32_t)current;
  if (delta > up_step)
  {
    delta = up_step;
  }
  else if (delta < -down_step)
  {
    delta = -down_step;
  }
  return clamp_i16((int32_t)current + delta, output_min_permille(), output_max_permille());
}

static uint16_t freeze_max_samples(void)
{
  uint32_t samples = ((uint32_t)BOARD_ALT_FREEZE_HOLD_MS * (uint32_t)BOARD_CONTROL_LOOP_HZ) / 1000U;
  if (samples < 1U)
  {
    samples = 1U;
  }
  if (samples > 65535U)
  {
    samples = 65535U;
  }
  return (uint16_t)samples;
}

static bool feedback_blocks_up(const mixer_feedback_t *feedback)
{
  if (feedback == 0)
  {
    return false;
  }
  return feedback->saturated_high ||
         feedback->attitude_scaled ||
         (feedback->motor_max_permille >= (uint16_t)(BOARD_MOTOR_MAX_PERMILLE - 5U));
}

static bool feedback_blocks_down(const mixer_feedback_t *feedback)
{
  if (feedback == 0)
  {
    return false;
  }
  return feedback->saturated_low ||
         (feedback->motor_min_permille <= (uint16_t)(BOARD_MOTOR_IDLE_PERMILLE + 5U));
}

static void remember_feedback(const mixer_feedback_t *feedback)
{
  if (feedback != 0)
  {
    s_last_feedback = *feedback;
  }
  else
  {
    memset(&s_last_feedback, 0, sizeof(s_last_feedback));
    s_last_feedback.attitude_scale_permille = 1000U;
  }
}

static void publish_debug(int32_t altitude_error_cm,
                          int32_t target_velocity_cms,
                          int16_t velocity_cms)
{
  s_debug.active = s_active;
  s_debug.velocity_control = s_velocity_control;
  s_debug.state = s_state;
  s_debug.hold_altitude_cm = s_hold_altitude_cm;
  s_debug.altitude_error_cm = clamp_i16(altitude_error_cm, -32768, 32767);
  s_debug.velocity_cms = velocity_cms;
  s_debug.estimated_velocity_cms = velocity_cms;
  s_debug.target_velocity_cms = clamp_i16(target_velocity_cms, -32768, 32767);
  s_debug.stick_reference_permille = s_stick_reference_permille;
  s_debug.throttle_base_permille = s_base_permille;
  s_debug.base_permille = s_base_permille;
  s_debug.hover_permille = s_hover_permille;
  s_debug.correction_permille = s_last_correction_permille;
  s_debug.output_permille = s_last_output_permille;
  s_debug.assist_reliable = s_assist_reliable;
  s_debug.baro_rejected = s_baro_rejected;
  s_debug.assist_correction_limit_permille = (int16_t)BOARD_ALT_OUTPUT_LIMIT_PERMILLE;
  s_debug.freeze_reason = s_freeze_reason;
  s_debug.baro_quality = s_baro_rejected ? 0U : (s_assist_reliable ? 100U : 40U);
  s_debug.imu_predict_enabled = (BOARD_ALT_IMU_PREDICT_ENABLE != 0U);
  s_debug.sat_hi = s_last_feedback.saturated_high;
  s_debug.sat_lo = s_last_feedback.saturated_low;
  s_debug.motor_min_permille = s_last_feedback.motor_min_permille;
  s_debug.motor_max_permille = s_last_feedback.motor_max_permille;
  s_debug.motor_spread_permille = s_last_feedback.motor_spread_permille;
  s_debug.attitude_scale_permille = s_last_feedback.attitude_scale_permille;
}

static void set_output_from_base_and_correction(int32_t base_permille, int32_t correction_permille)
{
  int32_t total = base_permille + correction_permille;
  if ((s_state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD) && (total < (int32_t)BOARD_ALT_BASE_MIN_PERMILLE))
  {
    total = (int32_t)BOARD_ALT_BASE_MIN_PERMILLE;
  }
  total = clamp_output_permille(total);
  base_permille = clamp_output_permille(base_permille);
  s_base_permille = (int16_t)base_permille;
  s_last_correction_permille = clamp_i16(total - base_permille,
                                         -(int32_t)BOARD_ALT_OUTPUT_LIMIT_PERMILLE,
                                         (int32_t)BOARD_ALT_OUTPUT_LIMIT_PERMILLE);
  s_last_output_permille = clamp_i16(base_permille + s_last_correction_permille,
                                     output_min_permille(),
                                     output_max_permille());
}

static void learn_hover(bool in_deadband,
                        bool saturated,
                        int32_t altitude_error_cm,
                        int16_t velocity_cms)
{
  if ((s_state != CONTROLLER_ALTITUDE_STATE_ALT_HOLD) || !in_deadband || saturated)
  {
    return;
  }
  if ((abs_i32(altitude_error_cm) > (BOARD_ALT_HOLD_DB_CM * 2)) ||
      (abs_i32((int32_t)velocity_cms) > BOARD_ALT_HOVER_LEARN_VEL_CMS) ||
      (abs_i32((int32_t)s_last_correction_permille) < 3))
  {
    return;
  }

  s_hover_learn_accum += (float)s_last_correction_permille * 0.0005f;
  if ((s_hover_learn_accum >= 1.0f) && (s_hover_permille < (int16_t)hover_max_permille()))
  {
    s_hover_permille++;
    s_velocity_integrator_permille -= 1.0f;
    s_hover_learn_accum -= 1.0f;
  }
  else if ((s_hover_learn_accum <= -1.0f) && (s_hover_permille > (int16_t)hover_min_permille()))
  {
    s_hover_permille--;
    s_velocity_integrator_permille += 1.0f;
    s_hover_learn_accum += 1.0f;
  }
}

static void start_active(const baro_sample_t *baro)
{
  s_active = true;
  s_velocity_control = false;
  s_assist_reliable = true;
  s_baro_rejected = false;
  s_state = CONTROLLER_ALTITUDE_STATE_GROUND_IDLE;
  s_hold_altitude_cm = baro->altitude_cm;
  s_takeoff_origin_altitude_cm = baro->altitude_cm;
  s_stick_reference_permille = (int16_t)BOARD_ALT_STICK_CENTER_PERMILLE;
  s_hover_permille = clamp_i16(BOARD_ALT_HOVER_THRUST_PERMILLE,
                               hover_min_permille(),
                               hover_max_permille());
  s_base_permille = (int16_t)BOARD_MOTOR_IDLE_PERMILLE;
  s_last_correction_permille = 0;
  s_last_output_permille = (int16_t)BOARD_MOTOR_IDLE_PERMILLE;
  s_velocity_integrator_permille = 0.0f;
  s_hover_learn_accum = 0.0f;
  s_freeze_samples = 0U;
  s_freeze_reason = CONTROLLER_ALTITUDE_FREEZE_NONE;
}

static int16_t freeze_or_exit(const control_setpoint_t *setpoint,
                              uint8_t freeze_reason,
                              uint16_t *base_throttle_permille)
{
  if (s_active && (s_freeze_samples < freeze_max_samples()))
  {
    s_state = CONTROLLER_ALTITUDE_STATE_FROZEN;
    s_freeze_samples++;
    s_freeze_reason = freeze_reason;
    s_assist_reliable = false;
    s_baro_rejected = (freeze_reason & CONTROLLER_ALTITUDE_FREEZE_BARO) != 0U;
    if (base_throttle_permille != 0)
    {
      *base_throttle_permille = (uint16_t)s_base_permille;
    }
    publish_debug(0, 0, s_debug.velocity_cms);
    return s_last_correction_permille;
  }

  ControllerAltitude_Reset();
  if (base_throttle_permille != 0)
  {
    *base_throttle_permille = (setpoint != 0) ? setpoint->throttle_permille : 0U;
  }
  return 0;
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
  s_stick_reference_permille = 0;
  s_hover_permille = 0;
  s_base_permille = 0;
  s_last_correction_permille = 0;
  s_last_output_permille = 0;
  s_velocity_integrator_permille = 0.0f;
  s_hover_learn_accum = 0.0f;
  s_freeze_samples = 0U;
  s_freeze_reason = CONTROLLER_ALTITUDE_FREEZE_NONE;
  memset(&s_last_feedback, 0, sizeof(s_last_feedback));
  s_last_feedback.attitude_scale_permille = 1000U;
  memset(&s_debug, 0, sizeof(s_debug));
}

int16_t ControllerAltitude_Update(const baro_sample_t *baro,
                                  const control_setpoint_t *setpoint,
                                  bool requested,
                                  bool ready,
                                  uint8_t freeze_reason,
                                  const mixer_feedback_t *feedback,
                                  float dt_s,
                                  uint16_t *base_throttle_permille)
{
  if (base_throttle_permille != 0)
  {
    *base_throttle_permille = (setpoint != 0) ? setpoint->throttle_permille : 0U;
  }
  remember_feedback(feedback);

  if ((baro == 0) || (setpoint == 0) || !requested || !setpoint->baro_hold)
  {
    ControllerAltitude_Reset();
    return 0;
  }

  if ((dt_s <= 0.0f) || (dt_s > 0.050f))
  {
    dt_s = 0.002f;
  }

  if (!ready || !baro->healthy)
  {
    if (!baro->healthy)
    {
      freeze_reason |= CONTROLLER_ALTITUDE_FREEZE_BARO;
    }
    return freeze_or_exit(setpoint, freeze_reason, base_throttle_permille);
  }

  if (!s_active)
  {
    start_active(baro);
  }

  s_freeze_samples = 0U;
  s_freeze_reason = CONTROLLER_ALTITUDE_FREEZE_NONE;
  s_assist_reliable = true;
  s_baro_rejected = false;
  if (s_state == CONTROLLER_ALTITUDE_STATE_FROZEN)
  {
    s_state = CONTROLLER_ALTITUDE_STATE_ALT_HOLD;
    s_hold_altitude_cm = baro->altitude_cm;
    s_velocity_control = false;
  }

  bool above_takeoff = stick_above_takeoff_threshold(setpoint->throttle_permille);
  bool in_deadband = stick_in_hold_deadband(setpoint->throttle_permille);
  int32_t target_velocity_cms = stick_to_target_velocity_cms(setpoint->throttle_permille,
                                                             BOARD_ALT_STICK_CLIMB_MAX_CMS,
                                                             BOARD_ALT_STICK_DESCEND_MAX_CMS);
  int32_t altitude_error_cm = 0;

  if (s_state == CONTROLLER_ALTITUDE_STATE_GROUND_IDLE)
  {
    s_hold_altitude_cm = baro->altitude_cm;
    s_takeoff_origin_altitude_cm = baro->altitude_cm;
    s_velocity_control = false;
    s_velocity_integrator_permille = 0.0f;
    set_output_from_base_and_correction(BOARD_MOTOR_IDLE_PERMILLE, 0);
    if (base_throttle_permille != 0)
    {
      *base_throttle_permille = BOARD_MOTOR_IDLE_PERMILLE;
    }
    if (!above_takeoff)
    {
      publish_debug(0, 0, baro->velocity_cms);
      return 0;
    }
    s_state = CONTROLLER_ALTITUDE_STATE_TAKEOFF;
  }

  if (s_state == CONTROLLER_ALTITUDE_STATE_TAKEOFF)
  {
    int32_t takeoff_altitude_cm = baro->altitude_cm - s_takeoff_origin_altitude_cm;
    bool liftoff_detected = (takeoff_altitude_cm >= BOARD_ALT_LIFTOFF_ALT_CM) ||
                            (baro->velocity_cms >= BOARD_ALT_LIFTOFF_VEL_CMS);
    bool takeoff_committed = s_base_permille >= (int16_t)hover_min_permille();

    if (!above_takeoff && !liftoff_detected && !takeoff_committed)
    {
      s_state = CONTROLLER_ALTITUDE_STATE_GROUND_IDLE;
      set_output_from_base_and_correction(BOARD_MOTOR_IDLE_PERMILLE, 0);
      if (base_throttle_permille != 0)
      {
        *base_throttle_permille = BOARD_MOTOR_IDLE_PERMILLE;
      }
      publish_debug(0, 0, baro->velocity_cms);
      return 0;
    }

    int32_t takeoff_target_velocity = stick_to_target_velocity_cms(setpoint->throttle_permille,
                                                                   BOARD_ALT_TAKEOFF_CLIMB_MAX_CMS,
                                                                   0);
    int32_t desired_base = stick_to_takeoff_base_permille(setpoint->throttle_permille);
    if (desired_base > (int32_t)BOARD_ALT_BASE_MAX_PERMILLE)
    {
      desired_base = (int32_t)BOARD_ALT_BASE_MAX_PERMILLE;
    }
    s_base_permille = slew_i16(s_base_permille,
                               desired_base,
                               BOARD_ALT_TAKEOFF_SLEW_PER_SAMPLE,
                               BOARD_ALT_OUTPUT_SLEW_DOWN_PER_SAMPLE);
    set_output_from_base_and_correction(s_base_permille, 0);

    bool assist_ready = s_base_permille >=
                        (int16_t)(s_hover_permille + BOARD_ALT_TOY_ASSIST_ENGAGE_HEADROOM_PERMILLE);
    if (liftoff_detected || assist_ready || (!above_takeoff && takeoff_committed))
    {
      s_hover_permille = clamp_i16(s_base_permille, hover_min_permille(), hover_max_permille());
      s_state = CONTROLLER_ALTITUDE_STATE_ALT_HOLD;
      s_hold_altitude_cm = baro->altitude_cm;
      s_velocity_control = !in_deadband;
    }
    if (base_throttle_permille != 0)
    {
      *base_throttle_permille = (uint16_t)s_base_permille;
    }
    publish_debug(0, takeoff_target_velocity, baro->velocity_cms);
    return s_last_correction_permille;
  }

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
    }
    altitude_error_cm = s_hold_altitude_cm - baro->altitude_cm;
    if (abs_i32(altitude_error_cm) <= BOARD_ALT_HOLD_DB_CM)
    {
      altitude_error_cm = 0;
    }
    target_velocity_cms = (int32_t)((float)altitude_error_cm * BOARD_ALT_HOLD_POS_P);
    target_velocity_cms = clamp_i32(target_velocity_cms,
                                    -(int32_t)BOARD_ALT_HOLD_MAX_VEL_CMS,
                                    (int32_t)BOARD_ALT_HOLD_MAX_VEL_CMS);
  }

  bool block_up = feedback_blocks_up(feedback);
  bool block_down = feedback_blocks_down(feedback);
  int32_t velocity_error_cms = target_velocity_cms - (int32_t)baro->velocity_cms;
  bool block_integrator = (block_up && (velocity_error_cms > 0)) ||
                          (block_down && (velocity_error_cms < 0));
  if (!block_integrator)
  {
    s_velocity_integrator_permille += BOARD_ALT_VEL_I * (float)velocity_error_cms * dt_s;
  }
  else
  {
    s_velocity_integrator_permille *= BOARD_ITERM_DECAY;
  }
  s_velocity_integrator_permille = clampf_local(s_velocity_integrator_permille,
                                                -BOARD_ALT_I_LIMIT_PERMILLE,
                                                BOARD_ALT_I_LIMIT_PERMILLE);

  float correction = (BOARD_ALT_VEL_FF * (float)target_velocity_cms) +
                     (BOARD_ALT_VEL_P * (float)velocity_error_cms) +
                     s_velocity_integrator_permille;
  correction = clampf_local(correction,
                            -(float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE,
                            (float)BOARD_ALT_OUTPUT_LIMIT_PERMILLE);
  if (block_up && (correction > 0.0f) && (correction > (float)s_last_correction_permille))
  {
    correction = (float)s_last_correction_permille;
    if (correction < 0.0f)
    {
      correction = 0.0f;
    }
  }
  set_output_from_base_and_correction(s_hover_permille, float_to_i16(correction));
  if (base_throttle_permille != 0)
  {
    *base_throttle_permille = (uint16_t)s_base_permille;
  }
  learn_hover(in_deadband, block_up || block_down, altitude_error_cm, baro->velocity_cms);
  publish_debug(altitude_error_cm, target_velocity_cms, baro->velocity_cms);
  return s_last_correction_permille;
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

bool ControllerAltitude_IsFlying(void)
{
  return s_active && (s_state == CONTROLLER_ALTITUDE_STATE_ALT_HOLD);
}
