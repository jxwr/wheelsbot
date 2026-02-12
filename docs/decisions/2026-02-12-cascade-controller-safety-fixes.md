## Decision: Cascade Controller Safety and D-term Fixes

Date: 2026-02-12
Author: Claude Code

### Context

The initial implementation of `CascadeController` had several critical issues discovered during code review:

1. **Tilt protection was performed AFTER control computation** - If the robot was already on the ground, the inner loop would still compute a (wrong) control output before the safety check caught it.

2. **Ramp-in reset happened AFTER the first control step** - On rising edge (enable), integrators were not cleared before the first PID step, leading to stale data affecting initial output.

3. **PID D-term used error derivative** - The formula `(error - last_error) / dt` produces large spikes when reference changes and is sensitive to measurement noise. The correct approach is to use `-measurement_rate` (gyro rate) for the D-term.

4. **No debug visibility** - The cascade controller had no way to expose internal state (velocity loop output, angle loop integrator, etc.) making tuning impossible.

### Decision

Fix all four issues in the cascade controller:

1. **Move safety checks to the beginning** of `step()` - Validate tilt and sensor timeout BEFORE any control computation.

2. **Move ramp-in reset to rising edge detection** - Reset integrators immediately when `enabled && !was_enabled`, before any PID step.

3. **Extend `ControlInput` with `measurement_rate`** - Use gyro rate for D-term when available, fallback to error derivative only when rate is unavailable.

4. **Add `CascadeDebug` structure and `getDebug()` method** - Expose all internal state for telemetry.

### Alternatives considered

1. **Keep existing order, rely on output clamping** - Rejected: Computing control when already faulted is wasteful and confusing for debugging.

2. **Use conditional compilation for debug** - Rejected: `mutable CascadeDebug debug_` member has negligible overhead, simpler to always have it.

3. **Keep D-term as error derivative only** - Rejected: Measurement derivative is standard practice in aerospace/robotics for better noise rejection.

### Impact

**Files modified:**
- `src/control/control_loop_interface.h` - Added `measurement_rate` to `ControlInput`
- `src/control/pid_controller.cpp` - Modified D-term calculation
- `src/control/cascade_controller.h/cpp` - Reordered safety checks, added debug structure
- `src/control/angle_controller.h` - Added `const pid()` accessor
- `src/control/velocity_controller.h` - Added `const pid()` and `getMaxTiltCommand()`
- `src/control/pid_controller.h` - Added getter methods for persistence

**Behavior changes:**
- Robot now enters FAULT immediately on tilt exceed (no spurious motor output)
- Clean startup with zeroed integrators
- D-term responds to gyro rate (smoother, less noisy)
- Full visibility of cascade internals via WebSocket

**API additions:**
```cpp
struct CascadeDebug {
    float velocity_error, velocity_integrator, pitch_cmd;
    float pitch_error, pitch_integrator, pitch_rate_used;
    float motor_output, enable_gain;
    uint32_t fault_flags;
    bool running;
    float pitch;  // Added later
};
```

### Follow-ups

- [x] Update WebSocket telemetry to use new debug interface
- [x] Verify D-term noise level with real IMU data
- [ ] Add unit tests for safety check ordering
- [ ] Document tuning procedure using new debug fields
