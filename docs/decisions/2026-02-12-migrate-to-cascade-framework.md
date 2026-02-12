## Decision: Complete Migration from balance_core to cascade_controller

Date: 2026-02-12
Author: Claude Code

### Context

The codebase had two competing control systems running simultaneously:

1. **balance_core** (legacy) - A monolithic single-loop PID controller with integrated complementary filter, bias learning, and direct motor control. ~250 lines, tightly coupled.

2. **cascade_controller** (new) - A layered cascade framework with Velocity → Angle composition, proper safety state machine, and HAL integration.

This dual-system state caused:
- Confusion about which system was active
- WiFi debug only showed balance_core data (new system was invisible)
- Parameter changes via WebSocket only affected legacy system
- Code bloat and maintenance burden

### Decision

Completely remove `balance_core` and migrate all functionality to `cascade_controller`.

**Scope:**
1. Delete `balance_core.h` and `balance_core.cpp`
2. Remove all references from `main.cpp`, `shared_state.h`
3. Rewrite `wifi_debug.cpp` to use new cascade API
4. Update frontend (`index.html`) for new parameter structure

### Alternatives considered

1. **Keep both, add switching logic** - Rejected: Adds complexity, no benefit. cascade_controller is strictly superior.

2. **Gradual migration (deprecation period)** - Rejected: Confusing for users, doubles testing effort.

3. **Maintain WebSocket protocol backward compatibility** - Rejected: Old parameter names (Kp/Ki/Kd) don't map cleanly to cascade structure (angle_kp vs velocity_kp). Better to break once and clean.

### Impact

**Deleted files:**
- `src/balance_core.h`
- `src/balance_core.cpp`

**Modified files:**
- `src/main.cpp` - Removed `bc_init()`, added cascade param loading
- `src/shared_state.h` - Removed `extern bc_ctx_t g_bc`
- `src/wifi_debug.cpp` - Complete rewrite for cascade protocol
- `src/wifi_debug.h` - Added `loadCascadeParams()` declaration
- `data/index.html` - New UI layout, new parameter names

**WebSocket protocol changes:**
```json
// Old (balance_core)
{"type":"set_bc","key":"Kp","value":6.0}
{"type":"params","bc":{"Kp":6.0,"Ki":0.0,...}}

// New (cascade)
{"type":"set_angle","key":"angle_kp","value":6.0}
{"type":"params","cascade":{"angle_kp":6.0,"angle_ki":0.0,...}}
```

**New commands:**
- `set_angle` - Angle loop parameters
- `set_velocity` - Velocity loop parameters
- `set_safety` - Safety limits (max_tilt, ramp_time)
- `save_params` - Persist to LittleFS

**Obsolete parameters (removed):**
- `bias_learn_k`, `v_gate`, `w_gate` - Replaced by velocity loop integrator
- `comp_alpha` - Now in HAL layer
- `pitch_gyro_axis/sign` - HAL responsibility
- `deadband`, `Kx` - Unused
- `v2speed_gain`, `yaw2diff_gain` - Handled in main.cpp

### Follow-ups

- [x] Update frontend to match new protocol
- [x] Test parameter persistence end-to-end
- [ ] Document new tuning workflow for users
- [ ] Add migration guide for users with saved parameters
- [ ] Consider parameter import tool (old format → new format)
