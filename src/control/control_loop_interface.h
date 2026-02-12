#pragma once

#include <stdint.h>

namespace wheelsbot {
namespace control {

// ============================================================
// Control Loop Interface
// Abstract base for all controllers (PID, LQR, MPC, etc.)
// ============================================================

struct ControlInput {
  float reference;           // Setpoint/target value
  float measurement;         // Current sensor value
  float measurement_rate;    // Derivative of measurement (e.g., gyro rate) - optional, used for D-term
  float dt;                  // Time delta in seconds
};

struct ControlOutput {
  float control;      // Controller output
  bool valid;         // Output validity
  float error;        // Debug: tracking error
};

class ControlLoop {
 public:
  virtual ~ControlLoop() = default;

  // Main control step: compute control output from input
  virtual bool step(const ControlInput& in, ControlOutput& out) = 0;

  // Reset internal state (integrator, filters, etc.)
  virtual void reset() = 0;

  // Set PID gains (common interface, implementations may ignore unused)
  virtual void setGains(float kp, float ki, float kd) = 0;

  // Set output limits (saturation)
  virtual void setOutputLimits(float min_val, float max_val) = 0;

  // Set integral term limits (anti-windup)
  virtual void setIntegralLimit(float limit) = 0;

  // Get controller name
  virtual const char* name() const = 0;

  // Check if controller is initialized/ready
  virtual bool isReady() const { return true; }
};

}  // namespace control
}  // namespace wheelsbot
