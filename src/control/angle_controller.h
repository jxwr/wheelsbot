#pragma once

#include "control_loop_interface.h"
#include "pid_controller.h"

namespace wheelsbot {
namespace control {

// ============================================================
// Angle Controller (Inner Loop)
// Input: pitch angle error (rad)
// Output: motor torque/voltage command
//
// Controls robot pitch to maintain balance.
// The reference pitch comes from outer velocity/position loops.
// ============================================================

class AngleController : public ControlLoop {
 public:
  AngleController();

  bool step(const ControlInput& in, ControlOutput& out) override;
  void reset() override;
  void setGains(float kp, float ki, float kd) override;
  void setOutputLimits(float min_val, float max_val) override;
  void setIntegralLimit(float limit) override;
  const char* name() const override { return "AngleController"; }

  // D-term filter for noise rejection
  void setDFilterAlpha(float alpha) { pid_.setDFilterAlpha(alpha); }

  // Access underlying PID for direct tuning
  PidController& pid() { return pid_; }
  const PidController& pid() const { return pid_; }

 private:
  PidController pid_;
};

}  // namespace control
}  // namespace wheelsbot
