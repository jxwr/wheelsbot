#pragma once

#include "control_loop_interface.h"
#include "angle_controller.h"
#include "velocity_controller.h"

namespace wheelsbot {
namespace control {

// ============================================================
// Cascade Controller
// Composes two loops: Velocity -> Angle
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
  uint32_t timestamp_ms;      // millis() for frequency tracking
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

// ============================================================
// Frequency statistics for monitoring loop execution
// ============================================================
struct FrequencyStats {
  float velocity_hz = 0.0f;  // Velocity loop actual frequency
  float angle_hz = 0.0f;     // Angle loop actual frequency
};

// ============================================================
// Limit status for detecting saturation
// ============================================================
struct LimitStatus {
  bool velocity_saturated = false;  // Velocity loop output at limit
  bool angle_saturated = false;     // Angle loop output at limit
  bool motor_saturated = false;     // Final motor command at limit
};

// ============================================================
// Runtime statistics (reset on power cycle)
// ============================================================
struct RuntimeStats {
  uint32_t total_runtime_sec = 0;   // Total running time in seconds
  uint32_t fault_count_total = 0;   // Total fault occurrences
  float max_pitch_ever = 0.0f;      // Maximum pitch ever recorded (rad)
  float min_pitch_ever = 0.0f;      // Minimum pitch ever recorded (rad)
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

  // Statistics getters
  void getFrequencyStats(FrequencyStats& out) const;
  void getLimitStatus(LimitStatus& out) const;
  void getRuntimeStats(RuntimeStats& out) const;
  void resetRuntimeStats();  // Reset runtime statistics

  // Parameters for serialization (persist to flash)
  struct Params {
    // Angle loop parameters
    float angle_kp = 6.0f, angle_ki = 0.0f, angle_kd = 0.6f;
    float angle_d_alpha = 0.7f;
    float angle_max_out = 6.0f;
    float angle_integrator_limit = 2.0f;

    // Velocity loop parameters
    float velocity_kp = 0.0f;
    float velocity_ki = 0.0f;
    float velocity_max_tilt = 0.14f;  // ~8 degrees

    // Cascade-level parameters
    float max_tilt = 35.0f * (3.14159f / 180.0f);  // Safety limit
    float ramp_time = 0.5f;
    float pitch_offset = 0.0f;
  };

  void setParams(const Params& p);
  void getParams(Params& p) const;
  // Persistence handled at application layer (wifi_debug.cpp)

  // Decimation configuration (inner / outer frequency ratio)
  void setVelocityDecimation(uint32_t ratio) { velocity_decimation_ = ratio; }
  uint32_t getVelocityDecimation() const { return velocity_decimation_; }

 private:
  VelocityController velocity_;  // Outer loop
  AngleController angle_;        // Inner loop

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

  // Decimation (frequency separation)
  uint32_t velocity_decimation_ = 4;   // 4:1 = 50Hz velocity / 200Hz angle
  uint32_t step_counter_ = 0;
  float last_pitch_cmd_ = 0.0f;        // Last velocity loop output

  // Frequency tracking
  mutable uint32_t last_time_ms_ = 0;
  mutable uint32_t angle_step_count_ = 0;
  mutable uint32_t velocity_step_count_ = 0;
  mutable float measured_velocity_hz_ = 0.0f;
  mutable float measured_angle_hz_ = 0.0f;

  // Limit status tracking
  mutable bool velocity_saturated_ = false;
  mutable bool angle_saturated_ = false;
  mutable bool motor_saturated_ = false;

  // Runtime statistics
  mutable RuntimeStats runtime_stats_;
  mutable uint32_t last_stats_update_ms_ = 0;

  static inline float clamp(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }
};

}  // namespace control
}  // namespace wheelsbot

