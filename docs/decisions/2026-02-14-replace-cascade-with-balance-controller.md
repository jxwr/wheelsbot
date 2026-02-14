## Decision: Replace CascadeController with BalanceController

Date: 2026-02-14
Author: Claude Code

### Context

The cascade control layer had two problems:

1. **CascadeController was dead code**: `balanceTask` in `main.cpp` bypassed `ctx->cascade.step()` entirely, using hardcoded PD control (`-20.0f * pitch_rad - 0.5f * gy`) and open-loop velocity feedforward (`VEL_FF_GAIN = 5.0`). The cascade framework was unused at runtime.

2. **Over-engineered abstractions**: 863 lines across 9 files (`PidController`, `AngleController`, `VelocityController`, `CascadeController`, `ControlLoop` interface) that were either unused or trivially wrapping each other. The custom `PidController` duplicated SimpleFOC's built-in `PIDController`.

Meanwhile, the Micro-Wheeled_leg-Robot project (`wl_pro_robot.ino`) had a proven LQR-decomposed balance strategy running directly on SimpleFOC `PIDController` instances, with features like wheel lift detection, CoG self-adaptation, and output integral compensation for static friction.

### Decision

Replace the entire cascade control stack with a single header-only `BalanceController` (~200 lines) that:
- Uses SimpleFOC's built-in `PIDController` and `LowPassFilter` directly (no custom wrappers)
- Ports the control strategy from `wl_pro_robot.ino` with adaptations for the two-wheeled balance bot

### Alternatives considered

1. **Fix the cascade controller to actually be used** - Rejected: The cascade velocity-to-tilt architecture doesn't match what the robot needs. The LQR decomposition (angle + gyro + distance + speed as parallel P controllers) is simpler and proven on real hardware.

2. **Keep the cascade framework, add LQR inside it** - Rejected: The cascade framework's Velocity -> Angle composition model is fundamentally different from the LQR parallel decomposition. Forcing one into the other adds complexity for no benefit.

3. **Patch the local SimpleFOC copy to make `error_prev` public** - Rejected: The wheel-leg project did this (`Lyf` modification), but modifying library internals creates maintenance burden. Instead, used placement-new reconstruction to reset PID state cleanly.

### Impact

**Created files:**
- `src/control/balance_controller.h` - Single header-only controller with:
  - 8 PID instances: `pid_angle` (P=6), `pid_gyro` (P=0.06), `pid_distance` (P=0.5), `pid_speed` (P=0.7), `pid_yaw_angle` (P=1.0), `pid_yaw_gyro` (P=0.04), `pid_lqr_u` (P=1, I=15), `pid_zeropoint` (P=0.002)
  - 2 LPF instances: `lpf_target_vel` (Tf=0.2), `lpf_zeropoint` (Tf=0.1)
  - Tilt protection with 200-cycle recovery delay
  - Distance zero-point management with stop-and-lock
  - Wheel lift detection (suppresses distance/speed output)
  - Output integral compensation for static friction dead-zone
  - CoG self-adaptation via slow pitch offset adjustment
  - Yaw heading hold with PD control

**Deleted files (9):**
- `src/control/pid_controller.h` / `.cpp`
- `src/control/angle_controller.h` / `.cpp`
- `src/control/velocity_controller.h` / `.cpp`
- `src/control/cascade_controller.h` / `.cpp`
- `src/control/control_loop_interface.h`

**Modified files:**
- `src/app_context.h` - `CascadeController cascade` -> `BalanceController balance`; removed `heading` and `remote_mode` (now internal to controller)
- `src/main.cpp` - Rewrote `balanceTask` to build `BalanceInput` and call `balance.step()`; removed hardcoded PD control, VEL_FF_GAIN, heading integration, clamp helper
- `src/wifi_debug.h` - `loadCascadeParams` -> `loadBalanceParams`
- `src/wifi_debug.cpp` - New persistence format (16 params, file `/params/balance.json`), new telemetry with per-PID contributions, added `handleSetYaw` and `handleSetCompensation` handlers

**WebSocket protocol changes:**
```json
// Old telemetry
{"type":"telem","vel_error":...,"pitch_cmd":...,"motor":...}

// New telemetry (per-PID contributions visible)
{"type":"telem","angle":...,"gyro":...,"dist":...,"spd":...,"lqr_raw":...,"lqr_comp":...,"yaw":...,"heading":...,"lifted":...}

// Old params
{"type":"params","cascade":{"angle_kp":...,"velocity_kp":...}}

// New params
{"type":"params","balance":{"angle_kp":...,"gyro_kp":...,"distance_kp":...,"speed_kp":...,"yaw_angle_kp":...,"lqr_u_kp":...,"zeropoint_kp":...}}
```

**New commands:**
- `set_yaw` - Yaw PID parameters (`yaw_angle_kp`, `yaw_gyro_kp`)
- `set_compensation` - Output integral and CoG adaptation (`lqr_u_kp`, `lqr_u_ki`, `zeropoint_kp`, LPF time constants)

**Persistence migration:** Old `/params/cascade.json` will not be loaded (different key names). Robot starts with defaults on first boot after update.

### Follow-ups

- [ ] Update `data/index.html` parameter tuning panel for new field names
- [ ] Tune default PID gains on actual hardware (defaults from wheel-leg robot may need adjustment for different mass/geometry)
- [ ] Add SimpleFOC Commander serial tuning (PID accessors already exposed via `pidAngle()` etc.)
- [ ] Consider re-adding runtime statistics and frequency monitoring if needed for diagnostics
