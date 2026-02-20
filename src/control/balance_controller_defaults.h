#pragma once

namespace wheelsbot {
namespace control {

// ============================================================
// Balance Controller Default Parameters
// These values are tuned for a 500g lightweight high-CoG robot
// with hard off-road tires.
// ============================================================

struct BalanceControllerDefaults {
    // Angle PID (primary restoring torque)
    // Tuned for lightweight high-CoG robot to avoid over-reaction
    static constexpr float angle_kp = 2.5f;

    // Gyro PID (pitch rate damping, input unit: deg/s)
    // Reference wheel-leg robot uses 0.06; slightly higher for heavier high-CoG body
    static constexpr float gyro_kp = 0.08f;

    // Distance PID (position hold / anti-drift)
    // Tuned for hard off-road tires - higher values for better position hold
    static constexpr float distance_kp = 1.0f;

    // Speed PID (velocity tracking)
    // Tuned for hard tires - helps stabilize stick-slip
    static constexpr float speed_kp = 1.0f;

    // Yaw angle PID (heading hold)
    static constexpr float yaw_angle_kp = 1.0f;

    // Yaw gyro PID (yaw rate damping)
    static constexpr float yaw_gyro_kp = 0.04f;

    // LQR output integral compensation (static friction compensation)
    // Tuned for hard tires - prevents stick-slip oscillation
    static constexpr float lqr_u_kp = 1.0f;
    static constexpr float lqr_u_ki = 3.0f;

    // Zero-point adaptation (CoG self-adjust)
    // Uses distance_ctrl (voltage) as input, not position error
    // Unit: degrees per volt - very small gain for slow adaptation
    static constexpr float zeropoint_kp = 0.002f;

    // Low-pass filter time constants
    static constexpr float lpf_target_vel_tf = 0.2f;
    static constexpr float lpf_zeropoint_tf  = 0.1f;

    // Safety
    // Reduced tilt threshold for high-CoG robot (harder to recover)
    static constexpr float max_tilt_deg = 20.0f;      // Tilt protection threshold (degrees)
    static constexpr float pitch_offset = 0.0f;       // Static pitch offset (degrees) - will be auto-adjusted

    // Output limits
    // Reduced for lightweight robot to prevent over-shooting
    static constexpr float pid_limit = 6.0f;          // Individual PID output limit
    static constexpr float pid_ramp  = 100000.0f;     // PID output ramp (effectively unlimited)

    // Wheel lift detection thresholds
    static constexpr float lift_accel_thresh = 50.0f;  // Angular acceleration threshold (rad/s^2)
    static constexpr float lift_vel_thresh   = 150.0f; // Angular velocity threshold (rad/s)

    // CoG adaptation deadband (prevent noise accumulation)
    static constexpr float cog_deadband = 0.05f;       // Only adapt when |distance_ctrl| > 0.06V
};

}  // namespace control
}  // namespace wheelsbot
