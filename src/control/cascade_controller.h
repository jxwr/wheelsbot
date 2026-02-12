#pragma once

#include "control_loop_interface.h"
#include "angle_controller.h"
#include "velocity_controller.h"

namespace wheelsbot {
namespace control {

// ============================================================
// Cascade Controller
// Composes outer + inner loops: Velocity → Angle
//
// Execution order:
//   1. Outer loop (velocity) generates pitch reference
//   2. Inner loop (angle) tracks pitch reference
//   3. Output is motor command
//
// Safety checks performed at cascade level:
//   - Tilt protection (max physical tilt)
//   - Sensor timeout
//   - Output ramp-in
// ============================================================

struct CascadeInput {
  // Velocity loop inputs
  float velocity_reference;   // Target wheel velocity (rad/s)
  float velocity_measurement; // Current wheel velocity (rad/s)

  // Angle loop inputs
  float pitch_measurement;    // Current pitch angle (rad)
  float pitch_rate;           // Pitch angular velocity (rad/s) - optional

  // Metadata
  float dt;
  bool enabled;               // Master enable
  bool sensors_valid;         // IMU + encoders healthy
};

struct CascadeOutput {
  float left_motor;   // Left motor command (voltage/torque)
  float right_motor;  // Right motor command
  float pitch_cmd;    // Commanded pitch (debug)
  bool valid;         // Output valid
  uint32_t fault_flags;
};

enum CascadeFault {
  CASCADE_FAULT_NONE = 0,
  CASCADE_FAULT_TILT_TOO_LARGE = 1u << 0,
  CASCADE_FAULT_SENSOR_LOST = 1u << 1,
  CASCADE_FAULT_BAD_DT = 1u << 2,
  CASCADE_FAULT_DISABLED = 1u << 3,
};

// ============================================================
// Debug structure for telemetry and tuning
// ============================================================
struct CascadeDebug {
  // Outer loop (velocity) state
  float velocity_error;      // Velocity tracking error (rad/s)
  float velocity_integrator; // Velocity PID integrator
  float pitch_cmd;           // Output: commanded pitch (rad)

  // Inner loop (angle) state
  float pitch_error;         // Pitch tracking error (rad)
  float pitch_integrator;    // Angle PID integrator
  float pitch_rate_used;     // D-term input (rad/s)

  // Output
  float motor_output;        // Final motor command (V)
  float enable_gain;         // Ramp-in gain (0..1)

  // Status
  uint32_t fault_flags;
  bool running;
  float pitch;               // Current pitch measurement (rad) - for telemetry
};

class CascadeController {
 public:
  CascadeController();

  // Main step: execute cascade
  bool step(const CascadeInput& in, CascadeOutput& out);

  // Reset all loops
  void reset();

  // Access individual controllers for tuning
  VelocityController& velocityLoop() { return velocity_; }
  AngleController& angleLoop() { return angle_; }

  // Configuration
  void setMaxTilt(float max_tilt_rad);          // Physical tilt limit (safety)
  void setRampTime(float ramp_time_s);          // Soft startup ramp
  void setPitchOffset(float offset_rad);        // Static pitch trim

  // State
  bool isRunning() const { return state_ == STATE_RUNNING; }
  uint32_t getFaultFlags() const { return fault_flags_; }

  // Debug: get internal state for telemetry
  void getDebug(CascadeDebug& out) const;

  // Parameters for serialization (persist to flash)
  struct Params {
    float angle_kp = 6.0f, angle_ki = 0.0f, angle_kd = 0.6f;
    float angle_d_alpha = 0.7f;
    float angle_max_out = 6.0f;
    float angle_integrator_limit = 2.0f;
    float velocity_kp = 0.0f;  // Outer loop P only
    float velocity_max_tilt = 0.3f;  // ~17 degrees
    float max_tilt = 35.0f * (3.14159f / 180.0f);  // Safety limit
    float ramp_time = 0.5f;
    float pitch_offset = 0.0f;
  };

  void setParams(const Params& p);
  void getParams(Params& p) const;
  // Persistence handled at application layer (wifi_debug.cpp)

 private:
  VelocityController velocity_;
  AngleController angle_;

  // Safety/state
  enum State { STATE_DISABLED, STATE_RUNNING, STATE_FAULT };
  State state_ = STATE_DISABLED;
  uint32_t fault_flags_ = 0;

  // Parameters
  float max_tilt_ = 35.0f * (3.14159f / 180.0f);  // 35 degrees
  float ramp_time_ = 0.5f;
  float pitch_offset_ = 0.0f;

  // Ramp state
  float enable_time_ = 0.0f;
  float enable_gain_ = 0.0f;
  bool was_enabled_ = false;

  // Sensor timeout tracking
  float sensor_bad_time_ = 0.0f;
  float sensor_timeout_ = 0.2f;

  // Debug state (updated each step)
  mutable CascadeDebug debug_;  // mutable to allow updates in const getDebug()

  static inline float clamp(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }
};

}  // namespace control
}  // namespace wheelsbot
