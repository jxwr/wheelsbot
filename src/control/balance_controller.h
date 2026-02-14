#pragma once

#include <SimpleFOC.h>
#include <cmath>

namespace wheelsbot {
namespace control {

// ============================================================
// Input / Output / Debug structs
// ============================================================

struct BalanceInput {
  // IMU raw data
  float ax;              // Acceleration X (m/s^2)
  float ay;              // Acceleration Y (m/s^2)
  float az;              // Acceleration Z (m/s^2)
  float gx;              // Gyro X (rad/s)
  float gy;              // Gyro Y (rad/s)
  float gz;              // Gyro Z (rad/s)

  // IMU fused angles
  float pitch;             // Pitch angle (rad, positive = forward tilt)
  float pitch_rate;        // Pitch angular velocity (rad/s)
  float roll;             // Roll angle (rad)
  float yaw_rate;          // Yaw angular velocity from gyro (rad/s)

  // Wheel data
  float wheel_position;   // Average wheel position (rad) = (xL + xR) / 2
  float wheel_velocity;   // Average wheel velocity (rad/s) = (wL + wR) / 2
  float wL;               // Left wheel velocity (rad/s)
  float wR;               // Right wheel velocity (rad/s)
  float xL;               // Left wheel position (rad)
  float xR;               // Right wheel position (rad)

  // Commands
  float target_velocity;   // Joystick linear velocity target (m/s)
  float target_yaw_rate;   // Joystick yaw rate target (rad/s)

  // Meta
  float dt;                // Time step (s)
  bool enabled;            // Master enable
  bool sensors_valid;      // IMU + encoders healthy
};

struct BalanceOutput {
  float left_motor;        // Left motor voltage command
  float right_motor;       // Right motor voltage command
  bool valid;              // Output is valid
  uint32_t fault_flags;    // Fault bitmask
};

enum BalanceFault {
  BALANCE_FAULT_NONE           = 0,
  BALANCE_FAULT_TILT_TOO_LARGE = 1u << 0,
  BALANCE_FAULT_SENSOR_LOST    = 1u << 1,
  BALANCE_FAULT_DISABLED       = 1u << 2,
};

struct BalanceDebug {
  // IMU 原始数据
  float ax;              // 加速度 X (m/s^2)
  float ay;              // 加速度 Y (m/s^2)
  float az;              // 加速度 Z (m/s^2)
  float gx;              // 陀螺仪 X (rad/s)
  float gy;              // 陀螺仪 Y (rad/s)
  float gz;              // 陀螺仪 Z (rad/s)

  // IMU 融合角度
  float pitch;
  float pitch_rate;
  float roll;              // 横滚角 (rad)
  float yaw_rate;          // 偏航率 (rad/s)

  // 轮子数据
  float wheel_velocity;    // 平均轮速 (rad/s)
  float wL;                // 左轮速度 (rad/s)
  float wR;               // 右轮速度 (rad/s)
  float xL;               // 左轮位置 (rad)
  float xR;               // 右轮位置 (rad)

  // 各环贡献
  float angle_contribution;    // pid_angle output
  float gyro_contribution;     // pid_gyro output
  float distance_contribution; // pid_distance output
  float speed_contribution;    // pid_speed output
  float lqr_u_raw;             // Sum before compensation
  float lqr_u_compensated;     // After pid_lqr_u

  // 偏航
  float yaw_output;            // Yaw differential
  float heading;               // Accumulated heading (rad)

  // 零点和偏移
  float pitch_offset;          // Current CoG offset
  float distance_zeropoint;    // Current distance zero-point

  // 输出
  float left_motor;
  float right_motor;

  // 状态
  bool running;
  uint32_t fault_flags;
  bool wheel_lifted;           // Wheel lift detected this cycle
};

// ============================================================
// BalanceController
// Single class replacing the entire cascade control stack.
// Uses SimpleFOC PIDController and LowPassFilter.
// Control strategy borrowed from Micro-Wheeled_leg-Robot.
// ============================================================

class BalanceController {
 public:
  struct Params {
    // Angle PID (primary restoring torque)
    // Reduced for lightweight high-CoG robot to avoid over-reaction
    float angle_kp = 5.0f;

    // Gyro PID (pitch rate damping)
    // CRITICAL: Doubled for lightweight high-CoG robot to suppress oscillation
    float gyro_kp = 0.12f;

    // Distance PID (position hold / anti-drift)
    // Reduced for lightweight robot (lower inertia)
    float distance_kp = 0.35f;

    // Speed PID (velocity tracking)
    // Reduced for lightweight robot (faster response)
    float speed_kp = 0.35f;

    // Yaw angle PID (heading hold)
    float yaw_angle_kp = 1.0f;

    // Yaw gyro PID (yaw rate damping)
    float yaw_gyro_kp = 0.04f;

    // LQR output integral compensation (static friction)
    // Reduced Ki for lightweight robot (lower static friction)
    float lqr_u_kp = 1.0f;
    float lqr_u_ki = 10.0f;

    // Zero-point adaptation (CoG self-adjust)
    float zeropoint_kp = 0.01f;

    // Low-pass filter time constants
    float lpf_target_vel_tf = 0.2f;
    float lpf_zeropoint_tf  = 0.1f;

    // Safety
    // Reduced tilt threshold for high-CoG robot (harder to recover)
    float max_tilt_deg = 20.0f;      // Tilt protection threshold (degrees)
    float pitch_offset = 0.0f;       // Static pitch offset (degrees)

    // Output limits
    // Reduced for lightweight robot to prevent over-shooting
    float pid_limit = 6.0f;          // Individual PID output limit
    float pid_ramp  = 100000.0f;     // PID output ramp (effectively unlimited)

    // Wheel lift detection thresholds
    float lift_accel_thresh = 50.0f;  // Angular acceleration threshold (rad/s^2)
    float lift_vel_thresh   = 150.0f;  // Angular velocity threshold (rad/s)
  };

  BalanceController()
    : pid_angle_     {6.0f,   0, 0, 100000, 8},
      pid_gyro_      {0.06f,  0, 0, 100000, 8},
      pid_distance_  {0.5f,   0, 0, 100000, 8},
      pid_speed_     {0.7f,   0, 0, 100000, 8},
      pid_yaw_angle_ {1.0f,   0, 0, 100000, 8},
      pid_yaw_gyro_  {0.04f,  0, 0, 100000, 8},
      pid_lqr_u_     {1.0f,  15, 0, 100000, 8},
      pid_zeropoint_ {0.01f, 0, 0, 100000, 4},
      lpf_target_vel_{0.2f},
      lpf_zeropoint_ {0.1f}
  {}

  // Main control step
  void step(const BalanceInput& in, BalanceOutput& out) {
    out.fault_flags = BALANCE_FAULT_NONE;
    out.valid = false;
    out.left_motor = 0;
    out.right_motor = 0;

    // --- Disabled check ---
    if (!in.enabled) {
      out.fault_flags |= BALANCE_FAULT_DISABLED;
      resetIntegrators();
      debug_.running = false;
      debug_.fault_flags = out.fault_flags;
      return;
    }

    // --- Sensor check ---
    if (!in.sensors_valid) {
      out.fault_flags |= BALANCE_FAULT_SENSOR_LOST;
      resetIntegrators();
      debug_.running = false;
      debug_.fault_flags = out.fault_flags;
      return;
    }

    float pitch_deg = in.pitch * (180.0f / PI_F);

    // --- Tilt protection with recovery ---
    if (fabsf(pitch_deg) > params_.max_tilt_deg) {
      uncontrollable_ = 1;
    }
    if (uncontrollable_ != 0) {
      if (fabsf(pitch_deg) < (params_.max_tilt_deg * 0.4f)) {
        uncontrollable_++;
      }
      if (uncontrollable_ > 200) {  // ~1s at 200Hz
        uncontrollable_ = 0;
      }
    }
    if (uncontrollable_ != 0) {
      out.fault_flags |= BALANCE_FAULT_TILT_TOO_LARGE;
      resetIntegrators();
      debug_.running = false;
      debug_.fault_flags = out.fault_flags;
      debug_.pitch = in.pitch;
      debug_.pitch_rate = in.pitch_rate;
      return;
    }

    // --- LQR-decomposed balance ---
    float angle_error = pitch_deg - params_.pitch_offset;
    float angle_ctrl    = pid_angle_(angle_error);
    float gyro_ctrl     = pid_gyro_(in.pitch_rate);

    // --- Distance and speed ---
    float distance = in.wheel_position;
    float speed    = in.wheel_velocity;

    // Smooth joystick velocity input
    float target_vel_filtered = lpf_target_vel_(in.target_velocity);
    // Convert m/s to rad/s for wheel speed reference
    float target_wheel_vel = target_vel_filtered / WHEEL_RADIUS;

    // Distance zero-point management
    bool has_joystick_input = (fabsf(in.target_velocity) > 0.01f);

    if (has_joystick_input) {
      // Reset distance zero-point while moving
      distance_zeropoint_ = distance;
      resetPid(pid_lqr_u_);  // Clear output integral
    }

    // Stop-and-lock: after joystick release, wait for speed < 0.5 before locking
    if (had_joystick_input_ && !has_joystick_input) {
      move_stop_flag_ = 1;
    }
    if (move_stop_flag_ == 1 && fabsf(speed) < 0.5f) {
      distance_zeropoint_ = distance;
      move_stop_flag_ = 0;
    }

    // Fast push detection: reset if pushed hard
    if (fabsf(speed) > 30.0f) {
      distance_zeropoint_ = distance;
    }

    had_joystick_input_ = has_joystick_input;

    float distance_ctrl = pid_distance_(distance - distance_zeropoint_);
    // Note: target_wheel_vel is already in rad/s (converted from m/s using WHEEL_RADIUS)
    // No additional scaling needed (0.1f factor was for wheel-leg robot's raw joystick values)
    float speed_ctrl    = pid_speed_(speed - target_wheel_vel);

    // --- Wheel lift detection ---
    // 计算真实加速度 (rad/s^2)，需要除以 dt
    float speed_accel = fabsf(speed - speed_prev_) / in.dt;
    speed_prev_ = speed;
    // 轮子抬起检测：已禁用，始终认为轮子着地
    bool wheel_lifted = false;

    float lqr_u;
    if (wheel_lifted) {
      // Only angle + gyro when wheels are off ground
      distance_zeropoint_ = distance;
      lqr_u = angle_ctrl + gyro_ctrl;
      resetPid(pid_lqr_u_);
    } else {
      lqr_u = angle_ctrl + gyro_ctrl + distance_ctrl + speed_ctrl;
    }

    debug_.lqr_u_raw = lqr_u;

    // --- Output integral compensation + CoG self-adaptation ---
    if (fabsf(lqr_u) < 5.0f && !has_joystick_input
        && fabsf(speed) < 1.0f && !wheel_lifted) {
      lqr_u = pid_lqr_u_(lqr_u);
      // CoG adaptation: DISABLED due to unit mismatch bug
      // TODO: Fix unit conversion (distance_ctrl is voltage, pitch_offset is degrees)
      // params_.pitch_offset += pid_zeropoint_(lpf_zeropoint_(distance_ctrl));
    } else {
      resetPid(pid_lqr_u_);
    }

    // --- Yaw control (dual-loop: angle + rate) ---
    // Integrate current heading from IMU gyro
    heading_current_ += in.yaw_rate * in.dt;
    // Normalize to [-pi, pi]
    while (heading_current_ > PI_F)  heading_current_ -= 2.0f * PI_F;
    while (heading_current_ < -PI_F) heading_current_ += 2.0f * PI_F;

    // Integrate target heading from joystick command (use actual dt)
    heading_target_ += in.target_yaw_rate * in.dt;
    // Normalize to [-pi, pi]
    while (heading_target_ > PI_F)  heading_target_ -= 2.0f * PI_F;
    while (heading_target_ < -PI_F) heading_target_ += 2.0f * PI_F;

    // Calculate heading error (shortest path)
    float yaw_angle_error = heading_target_ - heading_current_;
    if (yaw_angle_error > PI_F)  yaw_angle_error -= 2.0f * PI_F;
    if (yaw_angle_error < -PI_F) yaw_angle_error += 2.0f * PI_F;

    // Dual-loop control: angle error + rate damping
    // Disable yaw control when wheels are lifted (no ground reference)
    float yaw_output = 0.0f;
    if (!wheel_lifted) {
      float yaw_angle_ctrl = pid_yaw_angle_(yaw_angle_error);
      float yaw_gyro_ctrl  = pid_yaw_gyro_(in.yaw_rate);
      yaw_output = yaw_angle_ctrl + yaw_gyro_ctrl;
    }

    // --- Differential output ---
    float left_cmd  = -0.5f * (lqr_u + yaw_output);
    float right_cmd = -0.5f * (lqr_u - yaw_output);

    out.left_motor  = left_cmd;
    out.right_motor = right_cmd;
    out.valid = true;

    // --- Update debug ---
    // Raw IMU data
    debug_.ax = in.ax;
    debug_.ay = in.ay;
    debug_.az = in.az;
    debug_.gx = in.gx;
    debug_.gy = in.gy;
    debug_.gz = in.gz;

    // Fused angles
    debug_.pitch = in.pitch;
    debug_.pitch_rate = in.pitch_rate;
    debug_.roll = in.roll;
    debug_.yaw_rate = in.yaw_rate;
    debug_.wheel_velocity = in.wheel_velocity;
    debug_.wL = in.wL;
    debug_.wR = in.wR;
    debug_.xL = in.xL;
    debug_.xR = in.xR;
    debug_.angle_contribution = angle_ctrl;
    debug_.gyro_contribution = gyro_ctrl;
    debug_.distance_contribution = distance_ctrl;
    debug_.speed_contribution = speed_ctrl;
    debug_.lqr_u_compensated = lqr_u;
    debug_.yaw_output = yaw_output;
    debug_.pitch_offset = params_.pitch_offset;
    debug_.distance_zeropoint = distance_zeropoint_;
    debug_.heading = heading_current_;  // Report current heading (IMU-integrated)
    debug_.left_motor = left_cmd;
    debug_.right_motor = right_cmd;
    debug_.running = true;
    debug_.fault_flags = out.fault_flags;
    debug_.wheel_lifted = wheel_lifted;
  }

  void reset() {
    resetIntegrators();
    heading_current_ = 0;
    heading_target_ = 0;
    distance_zeropoint_ = -256.0f;  // Sentinel: not yet initialized
    speed_prev_ = 0;
    uncontrollable_ = 0;
    move_stop_flag_ = 0;
    had_joystick_input_ = false;
    debug_ = {};
  }

  void getDebug(BalanceDebug& out) const { out = debug_; }

  void setParams(const Params& p) {
    params_ = p;
    pid_angle_.P = p.angle_kp;
    pid_gyro_.P = p.gyro_kp;
    pid_distance_.P = p.distance_kp;
    pid_speed_.P = p.speed_kp;
    pid_yaw_angle_.P = p.yaw_angle_kp;
    pid_yaw_gyro_.P = p.yaw_gyro_kp;
    pid_lqr_u_.P = p.lqr_u_kp;
    pid_lqr_u_.I = p.lqr_u_ki;
    pid_zeropoint_.P = p.zeropoint_kp;
    lpf_target_vel_.Tf = p.lpf_target_vel_tf;
    lpf_zeropoint_.Tf = p.lpf_zeropoint_tf;

    // Update limits
    float lim = p.pid_limit;
    pid_angle_.limit = lim;
    pid_gyro_.limit = lim;
    pid_distance_.limit = lim;
    pid_speed_.limit = lim;
    pid_yaw_angle_.limit = lim;
    pid_yaw_gyro_.limit = lim;
    pid_lqr_u_.limit = lim;
  }

  void getParams(Params& p) const { p = params_; }

  // Access PID instances for Commander serial tuning
  PIDController& pidAngle()     { return pid_angle_; }
  PIDController& pidGyro()      { return pid_gyro_; }
  PIDController& pidDistance()   { return pid_distance_; }
  PIDController& pidSpeed()     { return pid_speed_; }
  PIDController& pidYawAngle()  { return pid_yaw_angle_; }
  PIDController& pidYawGyro()   { return pid_yaw_gyro_; }
  PIDController& pidLqrU()      { return pid_lqr_u_; }
  PIDController& pidZeropoint() { return pid_zeropoint_; }
  LowPassFilter& lpfTargetVel() { return lpf_target_vel_; }
  LowPassFilter& lpfZeropoint() { return lpf_zeropoint_; }

 private:
  static constexpr float PI_F = 3.14159265358979f;
  static constexpr float WHEEL_RADIUS = 0.035f;  // 70mm diameter wheels

  // Reset a PIDController by reconstructing it in-place (clears all internal state).
  // SimpleFOC 2.3.2 keeps error_prev/integral_prev/output_prev as protected,
  // so we use placement-new to reset cleanly without modifying library source.
  // Risk: Depends on compiler optimization and memory layout.
  // Alternative: Patch SimpleFOC to expose a public reset() method.
  static void resetPid(PIDController& pid) {
    float p = pid.P, i = pid.I, d = pid.D;
    float ramp = pid.output_ramp, lim = pid.limit;
    pid.~PIDController();
    new (&pid) PIDController(p, i, d, ramp, lim);
  }

  void resetIntegrators() {
    resetPid(pid_angle_);
    resetPid(pid_gyro_);
    resetPid(pid_distance_);
    resetPid(pid_speed_);
    resetPid(pid_yaw_angle_);
    resetPid(pid_yaw_gyro_);
    resetPid(pid_lqr_u_);
    resetPid(pid_zeropoint_);
  }

  Params params_;

  // PID instances (SimpleFOC PIDController)
  PIDController pid_angle_;
  PIDController pid_gyro_;
  PIDController pid_distance_;
  PIDController pid_speed_;
  PIDController pid_yaw_angle_;
  PIDController pid_yaw_gyro_;
  PIDController pid_lqr_u_;
  PIDController pid_zeropoint_;

  // Low-pass filters (SimpleFOC LowPassFilter)
  LowPassFilter lpf_target_vel_;
  LowPassFilter lpf_zeropoint_;

  // State
  float heading_current_ = 0;  // Current heading from IMU gyro integration (rad)
  float heading_target_ = 0;   // Target heading from joystick command (rad)
  float distance_zeropoint_ = -256.0f;  // Sentinel value
  float speed_prev_ = 0;
  int uncontrollable_ = 0;
  int move_stop_flag_ = 0;
  bool had_joystick_input_ = false;

  // Debug (updated each step)
  BalanceDebug debug_ = {};
};

}  // namespace control
}  // namespace wheelsbot
