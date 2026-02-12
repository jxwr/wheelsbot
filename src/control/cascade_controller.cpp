#include "cascade_controller.h"
#include <math.h>
#include <string.h>

namespace wheelsbot {
namespace control {


CascadeController::CascadeController() { reset(); }

void CascadeController::reset() {
  velocity_.reset();
  angle_.reset();
  state_ = STATE_DISABLED;
  fault_flags_ = 0;
  enable_time_ = 0.0f;
  enable_gain_ = 0.0f;
  was_enabled_ = false;
  sensor_bad_time_ = 0.0f;
}

void CascadeController::setMaxTilt(float max_tilt_rad) {
  max_tilt_ = max_tilt_rad;
}

void CascadeController::setRampTime(float ramp_time_s) {
  ramp_time_ = ramp_time_s;
}

void CascadeController::setPitchOffset(float offset_rad) {
  pitch_offset_ = offset_rad;
}

void CascadeController::getDebug(CascadeDebug& out) const {
  out = debug_;
}

void CascadeController::getFrequencyStats(FrequencyStats& out) const {
  out.outer_hz = measured_outer_hz_;
  out.inner_hz = measured_inner_hz_;
}

void CascadeController::getLimitStatus(LimitStatus& out) const {
  out.velocity_saturated = velocity_saturated_;
  out.angle_saturated = angle_saturated_;
  out.motor_saturated = motor_saturated_;
}

void CascadeController::getRuntimeStats(RuntimeStats& out) const {
  out = runtime_stats_;
}

void CascadeController::resetRuntimeStats() {
  runtime_stats_ = RuntimeStats();
}

void CascadeController::setParams(const Params& p) {
  // Apply angle loop parameters
  angle_.setGains(p.angle_kp, p.angle_ki, p.angle_kd);
  angle_.setDFilterAlpha(p.angle_d_alpha);
  angle_.setOutputLimits(-p.angle_max_out, p.angle_max_out);
  angle_.setIntegralLimit(p.angle_integrator_limit);

  // Apply velocity loop parameters (PI only, no D)
  velocity_.setGains(p.velocity_kp, p.velocity_ki, 0.0f);
  velocity_.setMaxTiltCommand(p.velocity_max_tilt);

  // Apply cascade-level parameters
  max_tilt_ = p.max_tilt;
  ramp_time_ = p.ramp_time;
  pitch_offset_ = p.pitch_offset;
}

void CascadeController::getParams(Params& p) const {
  // Get angle loop parameters
  p.angle_kp = angle_.pid().getKp();
  p.angle_ki = angle_.pid().getKi();
  p.angle_kd = angle_.pid().getKd();
  p.angle_d_alpha = angle_.pid().getDFilterAlpha();
  p.angle_max_out = angle_.pid().getOutputMax();
  p.angle_integrator_limit = angle_.pid().getIntegralLimit();

  // Get velocity loop parameters
  p.velocity_kp = velocity_.pid().getKp();
  p.velocity_ki = velocity_.pid().getKi();
  p.velocity_max_tilt = velocity_.getMaxTiltCommand();

  // Get cascade-level parameters
  p.max_tilt = max_tilt_;
  p.ramp_time = ramp_time_;
  p.pitch_offset = pitch_offset_;
}

bool CascadeController::step(const CascadeInput& in, CascadeOutput& out) {
  // Default output
  out.left_motor = 0.0f;
  out.right_motor = 0.0f;
  out.pitch_cmd = 0.0f;
  out.valid = false;
  out.fault_flags = CASCADE_FAULT_NONE;

  // Clear debug output defaults
  debug_.velocity_error = 0.0f;
  debug_.velocity_integrator = 0.0f;
  debug_.pitch_error = 0.0f;
  debug_.pitch_integrator = 0.0f;
  debug_.pitch_rate_used = 0.0f;
  debug_.motor_output = 0.0f;
  debug_.pitch_cmd = 0.0f;
  debug_.enable_gain = enable_gain_;
  debug_.fault_flags = fault_flags_;
  debug_.running = false;
  debug_.pitch = in.pitch_measurement;  // Record current pitch for telemetry

  // Validate dt
  if (!(in.dt > 0.0f && in.dt < 0.1f)) {
    state_ = STATE_FAULT;
    fault_flags_ |= CASCADE_FAULT_BAD_DT;
    out.fault_flags = fault_flags_;
    debug_.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;  // Handled (fault state)
  }

  // Check disabled
  if (!in.enabled) {
    state_ = STATE_DISABLED;
    fault_flags_ = 0;
    was_enabled_ = false;
    enable_gain_ = 0.0f;
    enable_time_ = 0.0f;
    out.fault_flags = CASCADE_FAULT_DISABLED;
    debug_.fault_flags = CASCADE_FAULT_DISABLED;
    return true;
  }

  // ==== RAMP-IN: Check rising edge BEFORE any control computation ====
  if (in.enabled && !was_enabled_) {
    // Rising edge - reset everything for clean start
    enable_time_ = 0.0f;
    enable_gain_ = 0.0f;
    velocity_.reset();
    angle_.reset();
    sensor_bad_time_ = 0.0f;
  }

  // Update ramp timer
  enable_time_ += in.dt;
  if (ramp_time_ > 1e-3f) {
    enable_gain_ = clamp(enable_time_ / ramp_time_, 0.0f, 1.0f);
  } else {
    enable_gain_ = 1.0f;
  }
  debug_.enable_gain = enable_gain_;

  // ==== SAFETY: Check sensor validity ====
  if (!in.sensors_valid) {
    sensor_bad_time_ += in.dt;
  } else {
    sensor_bad_time_ = 0.0f;
  }

  if (sensor_bad_time_ > sensor_timeout_) {
    state_ = STATE_FAULT;
    fault_flags_ |= CASCADE_FAULT_SENSOR_LOST;
    out.fault_flags = fault_flags_;
    debug_.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;
  }

  // ==== SAFETY: Tilt protection - BEFORE control computation ====
  if (fabsf(in.pitch_measurement) > max_tilt_) {
    state_ = STATE_FAULT;
    fault_flags_ |= CASCADE_FAULT_TILT_TOO_LARGE;
    out.fault_flags = fault_flags_;
    debug_.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;
  }

  // ==== FREQUENCY TRACKING ====
  inner_step_count_++;
  uint32_t now_ms = in.timestamp_ms;
  if (last_time_ms_ == 0) last_time_ms_ = now_ms;
  uint32_t elapsed_ms = now_ms - last_time_ms_;
  if (elapsed_ms >= 1000) {  // Update every second
    measured_inner_hz_ = inner_step_count_ * 1000.0f / elapsed_ms;
    measured_outer_hz_ = outer_step_count_ * 1000.0f / elapsed_ms;
    inner_step_count_ = 0;
    outer_step_count_ = 0;
    last_time_ms_ = now_ms;
  }

  // ==== OUTER LOOP: Velocity → Pitch Command (decimated) ====
  step_counter_++;
  bool run_outer = (step_counter_ % outer_decimation_) == 0;

  float pitch_reference;
  if (run_outer) {
    outer_step_count_++;

    ControlInput vel_in;
    vel_in.reference = in.velocity_reference;
    vel_in.measurement = in.velocity_measurement;
    vel_in.measurement_rate = 0.0f;  // No direct acceleration measurement
    vel_in.dt = in.dt * outer_decimation_;  // Scale dt for outer loop

    ControlOutput vel_out;
    bool vel_ok = velocity_.step(vel_in, vel_out);

    // Check if velocity loop saturated
    float max_tilt_cmd = velocity_.getMaxTiltCommand();
    velocity_saturated_ = fabsf(vel_out.control) >= max_tilt_cmd * 0.99f;

    // Debug: capture velocity loop state
    debug_.velocity_error = vel_in.reference - vel_in.measurement;
    debug_.velocity_integrator = velocity_.pid().getIntegral();

    if (!vel_ok) {
      vel_out.control = 0.0f;
    }

    // The velocity output is the pitch command (tilt to achieve velocity)
    last_pitch_cmd_ = vel_out.control + pitch_offset_;
  }
  pitch_reference = last_pitch_cmd_;
  debug_.pitch_cmd = pitch_reference;

  // ==== INNER LOOP: Angle → Motor Command (runs every cycle) ====
  ControlInput ang_in;
  ang_in.reference = pitch_reference;
  ang_in.measurement = in.pitch_measurement;
  ang_in.measurement_rate = in.pitch_rate;  // Use gyro rate for D-term
  ang_in.dt = in.dt;

  ControlOutput ang_out;
  bool ang_ok = angle_.step(ang_in, ang_out);

  // Check if angle loop saturated
  float angle_out_max = angle_.pid().getOutputMax();
  angle_saturated_ = fabsf(ang_out.control) >= angle_out_max * 0.99f;

  // Debug: capture angle loop state
  debug_.pitch_error = ang_in.reference - ang_in.measurement;
  debug_.pitch_integrator = angle_.pid().getIntegral();
  debug_.pitch_rate_used = in.pitch_rate;

  if (!ang_ok) {
    state_ = STATE_FAULT;
    out.fault_flags = fault_flags_;
    debug_.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;
  }

  // Apply ramp to output
  float motor_cmd = ang_out.control * enable_gain_;

  // Check if motor output saturated (at voltage limit)
  float motor_limit = angle_out_max;  // Same as angle loop limit
  motor_saturated_ = fabsf(motor_cmd) >= motor_limit * 0.99f;

  debug_.motor_output = motor_cmd;

  // ==== RUNTIME STATISTICS ====
  if (elapsed_ms >= 1000) {
    runtime_stats_.total_runtime_sec++;
    if (fault_flags_ != 0) {
      runtime_stats_.fault_count_total++;
    }
  }
  // Track min/max pitch
  if (in.pitch_measurement > runtime_stats_.max_pitch_ever) {
    runtime_stats_.max_pitch_ever = in.pitch_measurement;
  }
  if (in.pitch_measurement < runtime_stats_.min_pitch_ever) {
    runtime_stats_.min_pitch_ever = in.pitch_measurement;
  }

  // ==== OUTPUT ====
  out.left_motor = motor_cmd;
  out.right_motor = motor_cmd;  // Same command both sides (differential added elsewhere)
  out.pitch_cmd = pitch_reference;
  out.valid = true;
  out.fault_flags = 0;

  state_ = STATE_RUNNING;
  was_enabled_ = in.enabled;
  debug_.running = true;
  debug_.fault_flags = 0;

  return true;
}

}  // namespace control
}  // namespace wheelsbot
