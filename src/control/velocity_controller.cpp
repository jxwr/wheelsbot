#include "velocity_controller.h"
#include <math.h>

namespace wheelsbot {
namespace control {

VelocityController::VelocityController() : pid_("VelocityPID") {
  // Default gains for outer loop (slower, gentler)
  // Start with conservative values - needs tuning
  pid_.setGains(0.0f, 0.0f, 0.0f);  // Disabled by default for safe tuning
  pid_.setOutputLimits(-max_tilt_, max_tilt_);
  pid_.setIntegralLimit(1.0f);
  pid_.setDFilterAlpha(0.5f);
}

bool VelocityController::step(const ControlInput& in, ControlOutput& out) {
  // Compute PID output
  ControlOutput pid_out;
  bool ok = pid_.step(in, pid_out);

  if (!ok) {
    out.valid = false;
    out.control = 0.0f;
    return false;
  }

  // PID output is the pitch command
  // Negative sign: positive velocity -> pitch forward (negative pitch)
  // This is handled by the sign of Kp
  float pitch_cmd = pid_out.control;

  // Safety clamp (redundant with PID limits, but explicit)
  if (pitch_cmd > max_tilt_) pitch_cmd = max_tilt_;
  if (pitch_cmd < -max_tilt_) pitch_cmd = -max_tilt_;

  out.control = pitch_cmd;  // This is the pitch reference for inner loop
  out.valid = true;
  out.error = pid_out.error;

  return true;
}

void VelocityController::reset() { pid_.reset(); }

void VelocityController::setGains(float kp, float ki, float kd) {
  pid_.setGains(kp, ki, kd);
}

void VelocityController::setOutputLimits(float min_val, float max_val) {
  // Store as max_tilt for clarity
  max_tilt_ = fminf(fabsf(max_val), fabsf(min_val));
  pid_.setOutputLimits(-max_tilt_, max_tilt_);
}

void VelocityController::setIntegralLimit(float limit) {
  pid_.setIntegralLimit(limit);
}

}  // namespace control
}  // namespace wheelsbot
