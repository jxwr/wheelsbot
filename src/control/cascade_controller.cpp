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

bool CascadeController::step(const CascadeInput& in, CascadeOutput& out) {
  // Default output
  out.left_motor = 0.0f;
  out.right_motor = 0.0f;
  out.pitch_cmd = 0.0f;
  out.valid = false;
  out.fault_flags = CASCADE_FAULT_NONE;

  // Validate dt
  if (!(in.dt > 0.0f && in.dt < 0.1f)) {
    state_ = STATE_FAULT;
    fault_flags_ |= CASCADE_FAULT_BAD_DT;
    out.fault_flags = fault_flags_;
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
    return true;
  }

  // Check sensor validity
  if (!in.sensors_valid) {
    sensor_bad_time_ += in.dt;
  } else {
    sensor_bad_time_ = 0.0f;
  }

  if (sensor_bad_time_ > sensor_timeout_) {
    state_ = STATE_FAULT;
    fault_flags_ |= CASCADE_FAULT_SENSOR_LOST;
    out.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;
  }

  // ==== OUTER LOOP: Velocity → Pitch Command ====
  ControlInput vel_in;
  vel_in.reference = in.velocity_reference;
  vel_in.measurement = in.velocity_measurement;
  vel_in.dt = in.dt;

  ControlOutput vel_out;
  bool vel_ok = velocity_.step(vel_in, vel_out);

  if (!vel_ok) {
    // Outer loop failure - fall back to zero velocity command
    vel_out.control = 0.0f;
  }

  // The velocity output is the pitch command (tilt to achieve velocity)
  float pitch_reference = vel_out.control + pitch_offset_;

  // ==== INNER LOOP: Angle → Motor Command ====
  ControlInput ang_in;
  ang_in.reference = pitch_reference;
  ang_in.measurement = in.pitch_measurement;
  ang_in.dt = in.dt;

  ControlOutput ang_out;
  bool ang_ok = angle_.step(ang_in, ang_out);

  if (!ang_ok) {
    state_ = STATE_FAULT;
    out.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;
  }

  // ==== SAFETY: Tilt Protection ====
  if (fabsf(in.pitch_measurement) > max_tilt_) {
    state_ = STATE_FAULT;
    fault_flags_ |= CASCADE_FAULT_TILT_TOO_LARGE;
    out.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;
  }

  // ==== RAMP-IN (Soft Start) ====
  if (in.enabled && !was_enabled_) {
    // Rising edge - reset ramp
    enable_time_ = 0.0f;
    enable_gain_ = 0.0f;
    velocity_.reset();
    angle_.reset();
  }

  enable_time_ += in.dt;
  if (ramp_time_ > 1e-3f) {
    enable_gain_ = clamp(enable_time_ / ramp_time_, 0.0f, 1.0f);
  } else {
    enable_gain_ = 1.0f;
  }

  // Apply ramp to output
  float motor_cmd = ang_out.control * enable_gain_;

  // ==== OUTPUT ====
  out.left_motor = motor_cmd;
  out.right_motor = motor_cmd;  // Same command both sides (differential added elsewhere)
  out.pitch_cmd = pitch_reference;
  out.valid = true;
  out.fault_flags = 0;

  state_ = STATE_RUNNING;
  was_enabled_ = in.enabled;

  return true;
}

}  // namespace control
}  // namespace wheelsbot
