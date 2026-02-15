#pragma once

#include <SimpleFOC.h>
#include <cmath>

namespace wheelsbot {
namespace control {

// ============================================================
// Simplified Cascade Controller
// Based on bd45f14, but using SimpleFOC PIDController
// Retains all debugging and tuning capabilities
// ============================================================

struct BalanceInput {
  // IMU data
  float pitch;           // Pitch angle (rad)
  float pitch_rate;      // Pitch angular velocity (rad/s)
  float yaw_rate;        // Yaw angular velocity (rad/s)

  // Wheel data
  float wheel_velocity;  // Average wheel velocity (rad/s)
  float wheel_position;  // Average wheel position (rad)

  // Commands
  float target_velocity; // Target linear velocity (m/s)
  float target_yaw_rate; // Target yaw rate (rad/s)

  // Metadata
  float dt;              // Time step (s)
  bool enabled;          // Master enable
  bool sensors_valid;    // IMU + encoders healthy
};

struct BalanceOutput {
  float left_motor;      // Left motor command (V)
  float right_motor;     // Right motor command (V)
  float pitch_cmd;       // Commanded pitch angle (rad) - for debug
  bool valid;            // Output valid
  uint32_t fault_flags;
};

enum BalanceFault {
  BALANCE_FAULT_NONE = 0,
  BALANCE_FAULT_TILT_TOO_LARGE = 1u << 0,
  BALANCE_FAULT_SENSOR_LOST = 1u << 1,
  BALANCE_FAULT_BAD_DT = 1u << 2,
  BALANCE_FAULT_DISABLED = 1u << 3,
};

// ============================================================
// Debug structure for telemetry and tuning
// ============================================================
struct BalanceDebug {
  // Outer loop (velocity) state
  float velocity_error;      // Velocity tracking error (rad/s)
  float velocity_integrator; // Velocity PID integrator
  float pitch_cmd;           // Output: commanded pitch (rad)

  // Inner loop (angle) state
  float pitch_error;         // Pitch tracking error (rad)
  float pitch_integrator;    // Angle PID integrator
  float pitch_rate_used;     // D-term input (rad/s)

  // Yaw control
  float yaw_output;          // Yaw differential output
  float d_filtered;          // Filtered D-term for angle loop

  // Output
  float motor_output;        // Final motor command (V)
  float enable_gain;         // Ramp-in gain (0..1)

  // Status
  uint32_t fault_flags;
  bool running;
  float pitch;               // Current pitch measurement
};

// ============================================================
// Statistics structures (from bd45f14)
// ============================================================
struct FrequencyStats {
  float velocity_hz = 0.0f;
  float angle_hz = 0.0f;
};

struct LimitStatus {
  bool velocity_saturated = false;
  bool angle_saturated = false;
  bool motor_saturated = false;
};

struct RuntimeStats {
  uint32_t total_runtime_sec = 0;
  uint32_t fault_count_total = 0;
  float max_pitch_ever = 0.0f;
  float min_pitch_ever = 0.0f;
};

// ============================================================
// Main Controller Class
// ============================================================
class BalanceController {
 public:
  BalanceController();

  // Main control step
  bool step(const BalanceInput& in, BalanceOutput& out);

  // Reset all loops
  void reset();

  // Access PID controllers for tuning
  PIDController& pidVelocity() { return pid_velocity_; }
  PIDController& pidAngle() { return pid_angle_; }
  // Note: Yaw control simplified - no PID object, direct damping only

  // Configuration
  void setMaxTilt(float max_tilt_rad) { max_tilt_ = max_tilt_rad; }
  void setRampTime(float ramp_time_s) { ramp_time_ = ramp_time_s; }
  void setPitchOffset(float offset_rad) { pitch_offset_ = offset_rad; }
  void setVelocityDecimation(uint32_t ratio) { velocity_decimation_ = ratio; }

  // State
  bool isRunning() const { return state_ == STATE_RUNNING; }
  uint32_t getFaultFlags() const { return fault_flags_; }

  // Debug and statistics
  void getDebug(BalanceDebug& out) const { out = debug_; }
  void getFrequencyStats(FrequencyStats& out) const;
  void getLimitStatus(LimitStatus& out) const;
  void getRuntimeStats(RuntimeStats& out) const { out = runtime_stats_; }
  void resetRuntimeStats() { runtime_stats_ = RuntimeStats(); }

  // Parameters for serialization
  struct Params {
    // Velocity loop (outer) - optimized for responsive control
    float velocity_kp = 0.15f;       // Increased for better response
    float velocity_ki = 0.02f;       // Small integral for steady-state accuracy
    float velocity_kd = 0.0f;
    float velocity_max_tilt = 0.25f;  // ~14 degrees - increased for stronger response

    // Angle loop (inner) - SimpleFOC PID P+I only, manual D term
    float angle_kp = 2.0f;           // Proportional gain - increased for stronger torque
    float angle_ki = 0.0f;           // No integral (prevents drift)
    float angle_gyro_kd = 1.0f;      // Gyro damping - increased to prevent oscillation
    float angle_d_alpha = 0.7f;      // D-term low-pass filter (0..1, higher=more smoothing)
    float angle_max_out = 12.0f;     // CRITICAL: Increased to allow full motor torque

    // Yaw control - simplified to rate damping only
    float yaw_kd = 1.2f;             // High damping for high-CoG robot

    // Cascade-level
    float max_tilt = 0.6f;           // Safety limit ~35 degrees
    float ramp_time = 0.5f;          // Soft startup
    float pitch_offset = 0.0f;       // Static trim
    float pitch_cmd_rate_limit = 0.5f; // Max pitch command change rate (rad/s)
    float sensor_timeout = 0.2f;     // Sensor loss tolerance (s)
    uint32_t velocity_decimation = 4; // 4:1 ratio (50Hz / 200Hz)
  };

  void setParams(const Params& p);
  void getParams(Params& p) const;

 private:
  // === PID Controllers (SimpleFOC) ===
  // Note: angle PID D-term set to 0 - we use manual gyro damping instead
  PIDController pid_velocity_{0.15f, 0.02f, 0.0f, 100000, 0.14f};
  PIDController pid_angle_{1.5f, 0.0f, 0.0f, 100000, 6.0f};
  //                       P     I    D=0 (manual D instead)

  // === Configuration ===
  float max_tilt_ = 0.6f;
  float ramp_time_ = 0.5f;
  float pitch_offset_ = 0.0f;
  float pitch_cmd_rate_limit_ = 0.5f;  // rad/s
  float sensor_timeout_ = 0.2f;         // s
  uint32_t velocity_decimation_ = 4;

  // Manual D-term for angle loop (gyro damping + filtering)
  float angle_gyro_kd_ = 0.6f;
  float angle_d_alpha_ = 0.7f;
  float d_filtered_ = 0.0f;

  // Yaw damping coefficient
  float yaw_kd_ = 1.2f;

  // === State ===
  enum State { STATE_DISABLED, STATE_RUNNING, STATE_FAULT };
  State state_ = STATE_DISABLED;
  uint32_t fault_flags_ = 0;

  // Ramp-in state
  float enable_time_ = 0.0f;
  float enable_gain_ = 0.0f;
  bool was_enabled_ = false;

  // Sensor timeout tracking
  float sensor_bad_time_ = 0.0f;

  // Decimation counter
  uint32_t step_counter_ = 0;
  float last_pitch_cmd_ = 0.0f;

  // === Debug ===
  BalanceDebug debug_ = {};

  // === Statistics ===
  mutable uint32_t last_time_ms_ = 0;
  mutable uint32_t angle_step_count_ = 0;
  mutable uint32_t velocity_step_count_ = 0;
  mutable float measured_velocity_hz_ = 0.0f;
  mutable float measured_angle_hz_ = 0.0f;
  mutable LimitStatus limit_status_ = {};
  mutable RuntimeStats runtime_stats_ = {};
  mutable uint32_t last_stats_update_ms_ = 0;

  // === Helpers ===
  static inline float clamp(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }

  void updateFrequencyStats(uint32_t timestamp_ms);
  void updateRuntimeStats(float pitch, uint32_t timestamp_ms);
  void resetPid(PIDController& pid);
};

}  // namespace control
}  // namespace wheelsbot
