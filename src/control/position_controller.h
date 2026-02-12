#pragma once

#include "control_loop_interface.h"
#include "pid_controller.h"

namespace wheelsbot {
namespace control {

// ============================================================
// Position Controller (Outermost Loop)
// Input: position error (m or rad)
// Output: velocity command (m/s or rad/s)
//
// Controls robot position by commanding a velocity.
// The velocity command feeds into the VelocityController.
// ============================================================

class PositionController : public ControlLoop {
 public:
  PositionController();

  bool step(const ControlInput& in, ControlOutput& out) override;
  void reset() override;
  void setGains(float kp, float ki, float kd) override;
  void setOutputLimits(float min_val, float max_val) override;
  void setIntegralLimit(float limit) override;
  const char* name() const override { return "PositionController"; }

  // Access underlying PID for direct tuning
  PidController& pid() { return pid_; }
  const PidController& pid() const { return pid_; }

  // Set maximum velocity command (safety limit)
  void setMaxVelocity(float max_vel) { max_velocity_ = max_vel; }
  float getMaxVelocity() const { return max_velocity_; }

 private:
  PidController pid_;
  float max_velocity_ = 0.5f;  // m/s max velocity command
};

}  // namespace control
}  // namespace wheelsbot
