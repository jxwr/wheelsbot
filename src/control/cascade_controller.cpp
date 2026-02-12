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

  // ==== OUTER LOOP: Velocity → Pitch Command ====
  ControlInput vel_in;
  vel_in.reference = in.velocity_reference;
  vel_in.measurement = in.velocity_measurement;
  vel_in.measurement_rate = 0.0f;  // No direct acceleration measurement
  vel_in.dt = in.dt;

  ControlOutput vel_out;
  bool vel_ok = velocity_.step(vel_in, vel_out);

  // Debug: capture velocity loop state
  debug_.velocity_error = vel_in.reference - vel_in.measurement;
  debug_.velocity_integrator = velocity_.pid().getIntegral();

  if (!vel_ok) {
    // Outer loop failure - fall back to zero velocity command
    vel_out.control = 0.0f;
  }

  // The velocity output is the pitch command (tilt to achieve velocity)
  float pitch_reference = vel_out.control + pitch_offset_;
  debug_.pitch_cmd = pitch_reference;

  // ==== INNER LOOP: Angle → Motor Command ====
  ControlInput ang_in;
  ang_in.reference = pitch_reference;
  ang_in.measurement = in.pitch_measurement;
  ang_in.measurement_rate = in.pitch_rate;  // Use gyro rate for D-term
  ang_in.dt = in.dt;

  ControlOutput ang_out;
  bool ang_ok = angle_.step(ang_in, ang_out);

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
  debug_.motor_output = motor_cmd;

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
