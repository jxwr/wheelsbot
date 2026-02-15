#include "balance_controller.h"
#include <Arduino.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

namespace wheelsbot {
namespace control {

BalanceController::BalanceController() {
  reset();
}

void BalanceController::reset() {
  // SimpleFOC PID: angle D-term set to 0 (we use manual gyro damping)
  pid_velocity_ = PIDController(0.15f, 0.02f, 0.0f, 100000, 0.14f);
  pid_angle_ = PIDController(1.5f, 0.0f, 0.0f, 100000, 6.0f);

  state_ = STATE_DISABLED;
  fault_flags_ = 0;
  enable_time_ = 0.0f;
  enable_gain_ = 0.0f;
  was_enabled_ = false;
  sensor_bad_time_ = 0.0f;
  step_counter_ = 0;
  last_pitch_cmd_ = 0.0f;
  d_filtered_ = 0.0f;
  debug_ = {};
}

void BalanceController::setParams(const Params& p) {
  pid_velocity_.P = p.velocity_kp;
  pid_velocity_.I = p.velocity_ki;
  pid_velocity_.D = p.velocity_kd;
  pid_velocity_.limit = p.velocity_max_tilt;

  pid_angle_.P = p.angle_kp;
  pid_angle_.I = p.angle_ki;
  pid_angle_.D = 0.0f;  // Always 0 - we use manual gyro damping
  pid_angle_.limit = p.angle_max_out;

  angle_gyro_kd_ = p.angle_gyro_kd;
  angle_d_alpha_ = p.angle_d_alpha;
  yaw_kd_ = p.yaw_kd;

  max_tilt_ = p.max_tilt;
  ramp_time_ = p.ramp_time;
  pitch_offset_ = p.pitch_offset;
  pitch_cmd_rate_limit_ = p.pitch_cmd_rate_limit;
  sensor_timeout_ = p.sensor_timeout;
  velocity_decimation_ = p.velocity_decimation;
}

void BalanceController::getParams(Params& p) const {
  p.velocity_kp = pid_velocity_.P;
  p.velocity_ki = pid_velocity_.I;
  p.velocity_kd = pid_velocity_.D;
  p.velocity_max_tilt = pid_velocity_.limit;

  p.angle_kp = pid_angle_.P;
  p.angle_ki = pid_angle_.I;
  p.angle_gyro_kd = angle_gyro_kd_;
  p.angle_d_alpha = angle_d_alpha_;
  p.angle_max_out = pid_angle_.limit;

  p.yaw_kd = yaw_kd_;

  p.max_tilt = max_tilt_;
  p.ramp_time = ramp_time_;
  p.pitch_offset = pitch_offset_;
  p.pitch_cmd_rate_limit = pitch_cmd_rate_limit_;
  p.sensor_timeout = sensor_timeout_;
  p.velocity_decimation = velocity_decimation_;
}

bool BalanceController::step(const BalanceInput& in, BalanceOutput& out) {
  uint32_t timestamp_ms = millis();

  // Default output
  out.left_motor = 0.0f;
  out.right_motor = 0.0f;
  out.pitch_cmd = 0.0f;
  out.valid = false;
  out.fault_flags = BALANCE_FAULT_NONE;

  // Clear debug
  debug_.velocity_error = 0.0f;
  debug_.velocity_integrator = 0.0f;
  debug_.pitch_error = 0.0f;
  debug_.pitch_integrator = 0.0f;
  debug_.pitch_rate_used = 0.0f;
  debug_.motor_output = 0.0f;
  debug_.yaw_output = 0.0f;
  debug_.d_filtered = 0.0f;
  debug_.enable_gain = enable_gain_;
  debug_.fault_flags = fault_flags_;
  debug_.running = false;
  debug_.pitch = in.pitch;

  // Validate dt
  if (!(in.dt > 0.0f && in.dt < 0.1f)) {
    state_ = STATE_FAULT;
    fault_flags_ |= BALANCE_FAULT_BAD_DT;
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
    out.fault_flags = BALANCE_FAULT_DISABLED;
    debug_.fault_flags = BALANCE_FAULT_DISABLED;
    return true;
  }

  // Check sensors with timeout tolerance (original cascade logic)
  if (!in.sensors_valid) {
    sensor_bad_time_ += in.dt;
  } else {
    sensor_bad_time_ = 0.0f;
  }

  if (sensor_bad_time_ > sensor_timeout_) {
    state_ = STATE_FAULT;
    fault_flags_ |= BALANCE_FAULT_SENSOR_LOST;
    out.fault_flags = fault_flags_;
    debug_.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;
  }

  // Rising edge: reset
  if (in.enabled && !was_enabled_) {
    enable_time_ = 0.0f;
    enable_gain_ = 0.0f;
    resetPid(pid_velocity_);
    resetPid(pid_angle_);
    d_filtered_ = 0.0f;
    sensor_bad_time_ = 0.0f;
  }

  // Update ramp
  enable_time_ += in.dt;
  if (ramp_time_ > 1e-3f) {
    enable_gain_ = clamp(enable_time_ / ramp_time_, 0.0f, 1.0f);
  } else {
    enable_gain_ = 1.0f;
  }

  // Tilt protection
  if (fabsf(in.pitch) > max_tilt_) {
    state_ = STATE_FAULT;
    fault_flags_ |= BALANCE_FAULT_TILT_TOO_LARGE;
    out.fault_flags = fault_flags_;
    debug_.fault_flags = fault_flags_;
    was_enabled_ = false;
    return true;
  }

  state_ = STATE_RUNNING;
  fault_flags_ = 0;
  was_enabled_ = true;

  // === Outer Loop: Velocity Control (with decimation) ===
  float velocity_error = 0.0f;
  float pitch_cmd = last_pitch_cmd_;

  if (step_counter_ % velocity_decimation_ == 0) {
    // Convert m/s to rad/s for wheel velocity
    float target_wheel_vel = in.target_velocity / 0.035f;  // WHEEL_RADIUS
    velocity_error = target_wheel_vel - in.wheel_velocity;

    // Velocity PID outputs pitch command
    float pitch_cmd_new = pid_velocity_(velocity_error);
    pitch_cmd_new = clamp(pitch_cmd_new, -max_tilt_, max_tilt_);

    // Slew rate limiter: smooth pitch command changes
    float dt_velocity = in.dt * velocity_decimation_;
    float max_delta = pitch_cmd_rate_limit_ * dt_velocity;
    float delta = clamp(pitch_cmd_new - last_pitch_cmd_, -max_delta, max_delta);
    pitch_cmd = last_pitch_cmd_ + delta;
    last_pitch_cmd_ = pitch_cmd;

    // Check velocity saturation
    limit_status_.velocity_saturated = fabsf(pitch_cmd_new) >= pid_velocity_.limit * 0.99f;

    debug_.velocity_error = velocity_error;
    velocity_step_count_++;
  }

  step_counter_++;

  // === Inner Loop: Angle Control ===
  // Pitch error: current - (commanded + offset)
  float pitch_error = in.pitch - (pitch_cmd + pitch_offset_);

  // Angle PID: P + I only (SimpleFOC D-term is disabled)
  float motor_output = pid_angle_(pitch_error);

  // Manual D-term: gyro damping with low-pass filter
  // Filter: higher alpha = more smoothing (preserves more history)
  float d_raw = in.pitch_rate;
  d_filtered_ = angle_d_alpha_ * d_filtered_ + (1.0f - angle_d_alpha_) * d_raw;
  motor_output += angle_gyro_kd_ * d_filtered_;

  // Check angle saturation (before ramp)
  float motor_pre_ramp = motor_output;
  limit_status_.angle_saturated = fabsf(motor_pre_ramp) >= pid_angle_.limit * 0.99f;

  // Apply ramp-in gain
  motor_output *= enable_gain_;

  // Check motor saturation (after ramp)
  limit_status_.motor_saturated = fabsf(motor_output) >= pid_angle_.limit * 0.99f;

  debug_.pitch_error = pitch_error;
  debug_.pitch_rate_used = in.pitch_rate;
  debug_.d_filtered = d_filtered_;
  debug_.motor_output = motor_output;
  debug_.pitch_cmd = pitch_cmd;
  debug_.enable_gain = enable_gain_;
  angle_step_count_++;

  // === Yaw Control (Disabled) ===
  // Temporarily disabled - no differential steering
  float yaw_output = 0.0f;

  debug_.yaw_output = yaw_output;

  // === Differential Output ===
  // Note: No 0.5 scaling - each motor gets full forward command
  // Yaw differential is applied directly for turning
  out.left_motor = -(motor_output + yaw_output);
  out.right_motor = -(motor_output - yaw_output);
  out.pitch_cmd = pitch_cmd;
  out.valid = true;
  debug_.running = true;

  // Update statistics
  updateFrequencyStats(timestamp_ms);
  updateRuntimeStats(in.pitch, timestamp_ms);

  return true;
}

void BalanceController::getFrequencyStats(FrequencyStats& out) const {
  out.velocity_hz = measured_velocity_hz_;
  out.angle_hz = measured_angle_hz_;
}

void BalanceController::getLimitStatus(LimitStatus& out) const {
  out = limit_status_;
}

void BalanceController::updateFrequencyStats(uint32_t timestamp_ms) {
  if (last_time_ms_ == 0) {
    last_time_ms_ = timestamp_ms;
    return;
  }

  uint32_t dt_ms = timestamp_ms - last_time_ms_;
  if (dt_ms >= 1000) {  // Update every second
    float dt_sec = dt_ms / 1000.0f;
    measured_velocity_hz_ = velocity_step_count_ / dt_sec;
    measured_angle_hz_ = angle_step_count_ / dt_sec;

    velocity_step_count_ = 0;
    angle_step_count_ = 0;
    last_time_ms_ = timestamp_ms;
  }
}

void BalanceController::updateRuntimeStats(float pitch, uint32_t timestamp_ms) {
  // Update min/max pitch
  if (pitch > runtime_stats_.max_pitch_ever) {
    runtime_stats_.max_pitch_ever = pitch;
  }
  if (pitch < runtime_stats_.min_pitch_ever) {
    runtime_stats_.min_pitch_ever = pitch;
  }

  // Update runtime counter (rough, in seconds)
  if (timestamp_ms - last_stats_update_ms_ >= 1000) {
    runtime_stats_.total_runtime_sec++;
    last_stats_update_ms_ = timestamp_ms;
  }

  // Count faults
  if (fault_flags_ != 0) {
    runtime_stats_.fault_count_total++;
  }
}

void BalanceController::resetPid(PIDController& pid) {
  // SimpleFOC doesn't expose reset, recreate with same params
  float p = pid.P, i = pid.I, d = pid.D;
  float ramp = pid.output_ramp, lim = pid.limit;
  pid.~PIDController();
  new (&pid) PIDController(p, i, d, ramp, lim);
}

}  // namespace control
}  // namespace wheelsbot
