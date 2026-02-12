#pragma once

#include "control_loop_interface.h"
#include "pid_controller.h"

namespace wheelsbot {
namespace control {

// ============================================================
// Velocity Controller (Outer Loop)
// Input: wheel velocity error (rad/s)
// Output: pitch angle command (rad)
//
// Controls robot velocity by commanding a tilt angle.
// Positive velocity error -> pitch forward (negative pitch command)
// The angle command feeds into the inner AngleController.
// ============================================================

class VelocityController : public ControlLoop {
 public:
  VelocityController();

  bool step(const ControlInput& in, ControlOutput& out) override;
  void reset() override;
  void setGains(float kp, float ki, float kd) override;
  void setOutputLimits(float min_val, float max_val) override;
  void setIntegralLimit(float limit) override;
  const char* name() const override { return "VelocityController"; }

  // Access underlying PID for direct tuning
  PidController& pid() { return pid_; }

  // Set maximum tilt command (safety limit)
  void setMaxTiltCommand(float max_tilt_rad) { max_tilt_ = max_tilt_rad; }

 private:
  PidController pid_;
  float max_tilt_ = 0.3f;  // ~17 degrees max tilt command
};

}  // namespace control
}  // namespace wheelsbot
