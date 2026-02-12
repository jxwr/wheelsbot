## Decision: Separate Cascade Control Loops (Velocity → Angle)

Date: 2026-02-12
Author: Claude Code

### Context

Current `balance_core.cpp` combines multiple control responsibilities:
- Inner loop: Angle PID (pitch stabilization)
- Outer loop: Velocity feedback (Kv * v → tilt command)
- Sensor fusion: Complementary filter (now moved to HAL)
- Safety: Tilt protection, sensor timeout

This violates single-responsibility principle and makes it hard to:
- Tune velocity loop independently from angle loop
- Add position loop (future) as another cascade stage
- Test individual loops in isolation
- Swap controller types (PID → LQR → MPC per CLAUDE.md vision)

### Decision

Refactor into modular ControlLoop interface with cascade composition:

```
VelocityController (outer) → AngleController (inner) → Motor Output
         ↑                          ↑
    wheel velocity              IMU pitch
```

**New Architecture:**

```
control/
├── control_loop_interface.h   # Abstract ControlLoop base
├── pid_controller.h/cpp       # PID implementation
├── angle_controller.h/cpp     # Inner loop: pitch → torque
├── velocity_controller.h/cpp  # Outer loop: velocity → pitch_cmd
└── cascade_controller.h/cpp   # Pipeline composition
```

**ControlLoop Interface:**
```cpp
class ControlLoop {
public:
    struct Input { float reference; float measurement; float dt; };
    struct Output { float control; bool valid; };

    virtual void reset() = 0;
    virtual bool step(const Input& in, Output& out) = 0;
    virtual void setGains(float kp, float ki, float kd) = 0;
    virtual void setLimits(float out_min, float out_max) = 0;
};
```

### Alternatives considered

1. **Keep monolithic balance_core**: Rejected - blocks future extensibility (LQR/MPC)
2. **Template-based composition**: Rejected - too complex for embedded, harder to debug
3. **Function pointers**: Rejected - loses state encapsulation, harder to reset/inspect
4. **Separate FreeRTOS tasks**: Rejected - adds complexity, current 200Hz unified rate works

### Impact

**Files modified:**
- `src/balance_core.cpp/h` - deprecated, functionality migrated to new structure
- `src/main.cpp` - use CascadeController instead of bc_step()
- `src/shared_state.h` - may need tuning parameter exposure

**New files:**
- `src/control/*.h/cpp` - modular control loop implementations

**Behavioral changes:**
- Angle loop and Velocity loop run sequentially in same 200Hz task
- Velocity loop outputs pitch command (tilt reference)
- Angle loop tracks pitch reference (not fixed zero)
- PID gains need retuning (separate Kp/Ki/Kd for each loop)

**Safety:**
- Tilt protection stays in outer coordinator (not in individual loops)
- Each loop has independent output saturation
- Anti-windup on integrator for each loop

### Follow-ups

1. Create ControlLoop interface and PID implementation
2. Implement AngleController (inner loop)
3. Implement VelocityController (outer loop)
4. Create CascadeController for sequential execution
5. Migrate main.cpp to new control structure
6. Retune PID gains for separated loops
7. Add position loop (future) using same interface
