#pragma once

#include "control_loop_interface.h"

namespace wheelsbot {
namespace control {

// ============================================================
// PID Controller Implementation
// Features: D-term low-pass filter, anti-windup, output saturation
// ============================================================

class PidController : public ControlLoop {
 public:
  PidController(const char* name, float kp = 0.0f, float ki = 0.0f,
                float kd = 0.0f);

  bool step(const ControlInput& in, ControlOutput& out) override;
  void reset() override;
  void setGains(float kp, float ki, float kd) override;
  void setOutputLimits(float min_val, float max_val) override;
  void setIntegralLimit(float limit) override;
  const char* name() const override { return name_; }

  // D-term filter coefficient (0..1, higher = less filtering)
  void setDFilterAlpha(float alpha) { d_alpha_ = alpha; }

  // Direct access to internal state (for debugging)
  float getIntegral() const { return integral_; }
  float getLastError() const { return last_error_; }

  // Getters for persistence
  float getKp() const { return kp_; }
  float getKi() const { return ki_; }
  float getKd() const { return kd_; }
  float getDFilterAlpha() const { return d_alpha_; }
  float getOutputMax() const { return out_max_; }
  float getOutputMin() const { return out_min_; }
  float getIntegralLimit() const { return integral_limit_; }

 private:
  const char* name_;

  // Gains
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;

  // Limits
  float out_min_ = -1e6f;
  float out_max_ = 1e6f;
  float integral_limit_ = 1e6f;

  // State
  float integral_ = 0.0f;
  float last_error_ = 0.0f;
  float d_filtered_ = 0.0f;

  // D-term filter coefficient
  float d_alpha_ = 0.7f;

  static inline float clamp(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }
};

}  // namespace control
}  // namespace wheelsbot
