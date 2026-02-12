#include "angle_controller.h"

namespace wheelsbot {
namespace control {

AngleController::AngleController() : pid_("AnglePID") {
  // Default gains for inner loop (faster, tighter)
  // These will need tuning after separation
  pid_.setGains(6.0f, 0.0f, 0.6f);
  pid_.setOutputLimits(-6.0f, 6.0f);  // Voltage limit
  pid_.setIntegralLimit(2.0f);
  pid_.setDFilterAlpha(0.7f);
}

bool AngleController::step(const ControlInput& in, ControlOutput& out) {
  // Pass through to PID
  // Input: pitch error (reference - measured_pitch)
  // Output: motor command
  return pid_.step(in, out);
}

void AngleController::reset() { pid_.reset(); }

void AngleController::setGains(float kp, float ki, float kd) {
  pid_.setGains(kp, ki, kd);
}

void AngleController::setOutputLimits(float min_val, float max_val) {
  pid_.setOutputLimits(min_val, max_val);
}

void AngleController::setIntegralLimit(float limit) {
  pid_.setIntegralLimit(limit);
}

}  // namespace control
}  // namespace wheelsbot
