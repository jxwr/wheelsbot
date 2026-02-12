#include "pid_controller.h"
#include <math.h>

namespace wheelsbot {
namespace control {

PidController::PidController(const char* name, float kp, float ki, float kd)
    : name_(name), kp_(kp), ki_(ki), kd_(kd) {}

void PidController::reset() {
  integral_ = 0.0f;
  last_error_ = 0.0f;
  d_filtered_ = 0.0f;
}

void PidController::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PidController::setOutputLimits(float min_val, float max_val) {
  out_min_ = min_val;
  out_max_ = max_val;
}

void PidController::setIntegralLimit(float limit) {
  integral_limit_ = fabsf(limit);
}

bool PidController::step(const ControlInput& in, ControlOutput& out) {
  // Validate dt
  if (!(in.dt > 0.0f && in.dt < 0.1f)) {
    out.valid = false;
    out.control = 0.0f;
    out.error = 0.0f;
    return false;
  }

  // Compute error
  float error = in.reference - in.measurement;

  // Proportional term
  float p_term = kp_ * error;

  // Integral term with anti-windup
  if (ki_ != 0.0f) {
    integral_ += error * in.dt;
    integral_ = clamp(integral_, -integral_limit_, integral_limit_);
  } else {
    integral_ = 0.0f;
  }
  float i_term = ki_ * integral_;

  // Derivative term with low-pass filter
  float d_raw = (error - last_error_) / in.dt;
  d_filtered_ = d_alpha_ * d_raw + (1.0f - d_alpha_) * d_filtered_;
  float d_term = kd_ * d_filtered_;

  // Total output
  float output = p_term + i_term + d_term;

  // Output saturation (without integrator windup protection for now)
  output = clamp(output, out_min_, out_max_);

  // Store state
  last_error_ = error;

  // Fill output
  out.control = output;
  out.valid = true;
  out.error = error;

  return true;
}

}  // namespace control
}  // namespace wheelsbot
