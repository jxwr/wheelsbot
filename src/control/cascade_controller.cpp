#include "cascade_controller.h"
#include <math.h>
#include <string.h>

namespace wheelsbot {
namespace control {


CascadeController::CascadeController() { reset(); }

void CascadeController::reset() {
  position_.reset();
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
  out.position_hz = measured_position_hz_;
  out.velocity_hz = measured_velocity_hz_;
  out.angle_hz = measured_angle_hz_;
}

void CascadeController::getLimitStatus(LimitStatus& out) const {
  out.position_saturated = position_saturated_;
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

  // Apply position loop parameters (PI only, no D)
  position_.setGains(p.position_kp, p.position_ki, 0.0f);
  position_.setMaxVelocity(p.position_max_vel);

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

  // Get position loop parameters
  p.position_kp = position_.pid().getKp();
  p.position_ki = position_.pid().getKi();
  p.position_max_vel = position_.getMaxVelocity();

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
  out.velocity_cmd = 0.0f;
  out.valid = false;
  out.fault_flags = CASCADE_FAULT_NONE;

  // Clear debug output defaults
  debug_.position_error = 0.0f;
  debug_.position_integrator = 0.0f;
  debug_.velocity_cmd = 0.0f;
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
  debug_.pitch = in.pitch_measurement;

  // Validate dt
  if (!(in.dt > 0.0f && in.dt < 0.1f)) {
    state_ = STATE_FAULT;
    fault_flags_ |= CASCADE_FAULT_BAD_DT;
    out.fault_flags = fault_flags_;
    debug_.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;
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
    enable_time_ = 0.0f;
    enable_gain_ = 0.0f;
    position_.reset();
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

  // ==== SAFETY: Tilt protection ====
  if (fabsf(in.pitch_measurement) > max_tilt_) {
    state_ = STATE_FAULT;
    fault_flags_ |= CASCADE_FAULT_TILT_TOO_LARGE;
    out.fault_flags = fault_flags_;
    debug_.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;
  }

  // ==== FREQUENCY TRACKING ====
  angle_step_count_++;
  uint32_t now_ms = in.timestamp_ms;
  if (last_time_ms_ == 0) last_time_ms_ = now_ms;
  uint32_t elapsed_ms = now_ms - last_time_ms_;
  if (elapsed_ms >= 1000) {
    measured_angle_hz_ = angle_step_count_ * 1000.0f / elapsed_ms;
    measured_velocity_hz_ = velocity_step_count_ * 1000.0f / elapsed_ms;
    measured_position_hz_ = position_step_count_ * 1000.0f / elapsed_ms;
    angle_step_count_ = 0;
    velocity_step_count_ = 0;
    position_step_count_ = 0;
    last_time_ms_ = now_ms;
  }

  // ==== OUTERMOST LOOP: Position -> Velocity Command (decimated) ====
  step_counter_++;
  bool run_position = (step_counter_ % position_decimation_) == 0;

  float velocity_reference;
  // If external velocity reference is provided (non-zero), use it directly (remote control mode)
  // Otherwise, use position loop output (autonomous position holding mode)
  if (fabsf(in.velocity_reference) > 0.001f) {
    velocity_reference = in.velocity_reference;
    debug_.position_error = 0.0f;
    debug_.position_integrator = 0.0f;
  } else if (run_position) {
    position_step_count_++;

    ControlInput pos_in;
    pos_in.reference = in.position_reference;
    pos_in.measurement = in.position_measurement;
    pos_in.measurement_rate = 0.0f;
    pos_in.dt = in.dt * position_decimation_;

    ControlOutput pos_out;
    bool pos_ok = position_.step(pos_in, pos_out);

    float max_vel = position_.getMaxVelocity();
    position_saturated_ = fabsf(pos_out.control) >= max_vel * 0.99f;

    debug_.position_error = pos_in.reference - pos_in.measurement;
    debug_.position_integrator = position_.pid().getIntegral();

    if (!pos_ok) {
      pos_out.control = 0.0f;
    }

    last_velocity_cmd_ = pos_out.control;
    velocity_reference = last_velocity_cmd_;
  } else {
    velocity_reference = last_velocity_cmd_;
  }
  debug_.velocity_cmd = velocity_reference;

  // ==== OUTER LOOP: Velocity -> Pitch Command (decimated) ====
  bool run_velocity = (step_counter_ % velocity_decimation_) == 0;

  float pitch_reference;
  if (run_velocity) {
    velocity_step_count_++;

    ControlInput vel_in;
    vel_in.reference = velocity_reference;
    vel_in.measurement = in.velocity_measurement;
    vel_in.measurement_rate = 0.0f;
    vel_in.dt = in.dt * velocity_decimation_;

    ControlOutput vel_out;
    bool vel_ok = velocity_.step(vel_in, vel_out);

    float max_tilt_cmd = velocity_.getMaxTiltCommand();
    velocity_saturated_ = fabsf(vel_out.control) >= max_tilt_cmd * 0.99f;

    debug_.velocity_error = vel_in.reference - vel_in.measurement;
    debug_.velocity_integrator = velocity_.pid().getIntegral();

    if (!vel_ok) {
      vel_out.control = 0.0f;
    }

    last_pitch_cmd_ = vel_out.control + pitch_offset_;
  }
  pitch_reference = last_pitch_cmd_;
  debug_.pitch_cmd = pitch_reference;

  // ==== INNER LOOP: Angle -> Motor Command (runs every cycle) ====
  ControlInput ang_in;
  ang_in.reference = pitch_reference;
  ang_in.measurement = in.pitch_measurement;
  ang_in.measurement_rate = in.pitch_rate;
  ang_in.dt = in.dt;

  ControlOutput ang_out;
  bool ang_ok = angle_.step(ang_in, ang_out);

  float angle_out_max = angle_.pid().getOutputMax();
  angle_saturated_ = fabsf(ang_out.control) >= angle_out_max * 0.99f;

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

  float motor_limit = angle_out_max;
  motor_saturated_ = fabsf(motor_cmd) >= motor_limit * 0.99f;

  debug_.motor_output = motor_cmd;

  // ==== RUNTIME STATISTICS ====
  if (elapsed_ms >= 1000) {
    runtime_stats_.total_runtime_sec++;
    if (fault_flags_ != 0) {
      runtime_stats_.fault_count_total++;
    }
  }
  if (in.pitch_measurement > runtime_stats_.max_pitch_ever) {
    runtime_stats_.max_pitch_ever = in.pitch_measurement;
  }
  if (in.pitch_measurement < runtime_stats_.min_pitch_ever) {
    runtime_stats_.min_pitch_ever = in.pitch_measurement;
  }

  // ==== OUTPUT ====
  out.left_motor = motor_cmd;
  out.right_motor = motor_cmd;
  out.pitch_cmd = pitch_reference;
  out.velocity_cmd = velocity_reference;
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
