## Decision: Fix CoG Self-Adaptation Input Source

Date: 2026-02-20
Author: Claude Code

### Context

The CoG (Center of Gravity) self-adaptation logic in BalanceController was causing violent oscillation. After comparing with the reference implementation from Micro-Wheeled_leg-Robot, a critical bug was identified.

**Original (Buggy) Implementation:**
```cpp
float position_error_rad = distance - distance_zeropoint_;
float angle_adjustment_deg = pid_zeropoint_(lpf_zeropoint_(position_error_rad)) * (180.0f / PI_F);
params_.pitch_offset -= angle_adjustment_deg;
```

**Problem:** Using position error (radians) as input to zeropoint adaptation.

**Why this is wrong:**
1. Position error can be caused by transient disturbances (e.g., a push), not just CoG offset
2. Adapting to transient disturbances causes the controller to "chase" temporary errors
3. Results in continuous oscillation as the controller overreacts to every small perturbation

### Decision

Revert to the original LQR implementation's approach: use `distance_ctrl` (position PID output) as the adaptation input.

**Corrected Implementation:**
```cpp
// distance_ctrl reflects steady-state torque needed to hold position
// Persistent non-zero distance_ctrl indicates CoG offset (not transient disturbance)
float zeropoint_adjustment = pid_zeropoint_(lpf_zeropoint_(distance_ctrl));
params_.pitch_offset -= zeropoint_adjustment;
```

**Key Changes:**
1. **Input source**: `distance - distance_zeropoint_` (position error) → `distance_ctrl` (PID output)
2. **zeropoint_kp**: `0.05` → `0.002` (25x reduction, matching reference implementation)
3. **Remove**: Unit conversion (radians to degrees), no longer needed

### Physical Interpretation

**Why `distance_ctrl` is the correct input:**

In the LQR decomposition:
```
LQR_u = angle_ctrl + gyro_ctrl + distance_ctrl + speed_ctrl
```

When the robot is stationary but the center of gravity is not centered:
- Gravity pulls the robot in one direction
- The position loop must output continuous torque (`distance_ctrl`) to counteract gravity
- This `distance_ctrl` is proportional to the CoG offset
- By adjusting `pitch_offset`, we change the equilibrium angle so that gravity does the work instead of the motor

**Why transient disturbances don't affect this:**

The adaptation only activates when ALL of the following are true:
1. `|LQR_u| < 5` - Total output is small (steady state)
2. `!has_joystick_input` - No active user command
3. `|speed| < 1.0` - Nearly stationary
4. `!wheel_lifted` - Wheels on ground

A transient push would violate condition #3 (speed spike), preventing adaptation.

### Parameter Analysis

**zeropoint_kp = 0.002** (unit: degrees/volt)

- `distance_ctrl` range: typically ±2V in steady state
- Adaptation rate: ±2V × 0.002 = ±0.004° per control cycle
- At 200Hz: max ~0.8°/second adaptation rate
- This is intentionally very slow to avoid instability

The previous value of 0.05 was appropriate for position error (radians) input, but completely wrong for voltage input.

### Impact

**Files Modified:**
- `src/control/balance_controller.h` - CoG adaptation logic and default parameter

**Behavioral Changes:**
1. Eliminates violent oscillation caused by adapting to transient disturbances
2. CoG adaptation now only responds to persistent steady-state offsets
3. Adaptation is much slower (by design) for stability

**Testing Required:**
1. Verify robot no longer oscillates violently
2. Test that CoG adaptation still works (place weight on robot, observe slow compensation)
3. Check that adaptation pauses during motion and resumes when stopped

### References

- Reference implementation: `Micro-Wheeled_leg-Robot/3.Software/wl_pro_robot/wl_pro_robot.ino:372`
- Related ADR: `2026-02-14-fix-drift-and-pitch-offset-adaptation.md`
