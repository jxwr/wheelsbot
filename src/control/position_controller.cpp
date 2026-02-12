#include "position_controller.h"
#include <math.h>

namespace wheelsbot {
namespace control {

PositionController::PositionController() : pid_("PositionPID") {
  // Default gains for outermost loop (slow, conservative)
  // Position control should be gentle to avoid oscillation
  pid_.setGains(1.0f, 0.0f, 0.0f);  // Start with P-only
  pid_.setOutputLimits(-max_velocity_, max_velocity_);
  pid_.setIntegralLimit(1.0f);  // 1 meter max integral
  pid_.setDFilterAlpha(0.5f);
}

bool PositionController::step(const ControlInput& in, ControlOutput& out) {
  // Compute PID output
  ControlOutput pid_out;
  bool ok = pid_.step(in, pid_out);

  if (!ok) {
    out.valid = false;
    out.control = 0.0f;
    return false;
  }

  // PID output is the velocity command
  float vel_cmd = pid_out.control;

  // Safety clamp (redundant with PID limits, but explicit)
  if (vel_cmd > max_velocity_) vel_cmd = max_velocity_;
  if (vel_cmd < -max_velocity_) vel_cmd = -max_velocity_;

  out.control = vel_cmd;  // This is the velocity reference for inner loop
  out.valid = true;
  out.error = pid_out.error;

  return true;
}

void PositionController::reset() { pid_.reset(); }

void PositionController::setGains(float kp, float ki, float kd) {
  pid_.setGains(kp, ki, kd);
}

void PositionController::setOutputLimits(float min_val, float max_val) {
  // Store as max_velocity for clarity
  max_velocity_ = fminf(fabsf(max_val), fabsf(min_val));
  pid_.setOutputLimits(-max_velocity_, max_velocity_);
}

void PositionController::setIntegralLimit(float limit) {
  pid_.setIntegralLimit(limit);
}

}  // namespace control
}  // namespace wheelsbot
