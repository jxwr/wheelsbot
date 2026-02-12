## Discussion Memo: Cascade Controller Frequency and Architecture Review

Date: 2026-02-12
Author: Claude Code (during code review session)
Context: Reviewing control loop implementation for two-wheel balancing robot
Status: **REVISED** - Initial analysis contained errors, see Section 2 Correction

---

### 1. Initial Observation: Same Frequency for Inner/Outer Loops

**Observation**: Both velocity loop (outer) and angle loop (inner) run at 200Hz in `balanceTask`.

**Question Raised**: Does this violate the "bandwidth separation" principle of cascade control?

**Standard Rule**: Inner loop bandwidth should be 3-10x outer loop bandwidth.

**Current Implementation**:
```cpp
// Both loops execute every 5ms (200Hz)
velocity_.step(vel_in, vel_out);   // Outer: velocity → pitch_cmd
angle_.step(ang_in, ang_out);      // Inner: pitch → motor
```

**Potential Issue**: Zero-delay coupling between loops may cause phase-coupled oscillation.

---

### 2. Correction: Architecture Analysis Was Wrong

**Initial (Incorrect) Analysis**: Claimed that commanding pitch angle as setpoint is fundamentally wrong because "any non-zero pitch causes robot to fall."

**Corrected Understanding**: Thanks to user correction:

> "平衡车静止直立对应角度=0，但运动/加减速时，稳定工作点通常就是不等于0的小角度，或者可理解为：车通过轮子前进，把支撑点'追'到重心下面，从而不倒。"

**Correct Physics**:

| State | Pitch Angle | Physical Meaning |
|-------|-------------|------------------|
| Stationary | 0° | Center of mass directly above wheels |
| Accelerating | > 0° (forward lean) | Wheels chase center of mass forward |
| Decelerating | < 0° (backward lean) | Wheels slow down, let mass catch up |

**Key Insight**:
- Non-zero pitch is **not inherently unstable**
- It's a **dynamic equilibrium** sustained by appropriate wheel acceleration
- The cascade structure `Velocity → Pitch → Motor` is **physically valid**

---

### 3. Real Problems Identified

#### Problem 1: No Frequency Separation (Primary Issue)

**Rule Violated**: Outer loop should run slower than inner loop.

**Current**: Both at 200Hz (5ms period)
**Recommended**: Outer loop at 50-100Hz, inner loop at 200Hz

**Why It Matters**:
- Outer loop generates pitch commands
- Inner loop must track these commands faster than they change
- If same frequency, inner loop can't catch up → oscillation

#### Problem 2: Outer Loop Output Not Properly Constrained

`velocity_controller.cpp:33-34`:
```cpp
if (pitch_cmd > max_tilt_) pitch_cmd = max_tilt_;
if (pitch_cmd < -max_tilt_) pitch_cmd = -max_tilt_;
```

This clamps output, but:
- Default `max_tilt_` is large (17° = 0.3 rad)
- Should be smaller for stability (e.g., 8-10° max)
- Also affects integral windup

#### Problem 3: Velocity Loop Only Has P-term

`cascade_controller.cpp:46`:
```cpp
velocity_.setGains(p.velocity_kp, 0.0f, 0.0f);  // Ki forced to 0
```

For speed control, need integral term to eliminate steady-state error.

---

### 4. Corrected Architecture Assessment

```
Position Loop (future, optional)
    ↓ velocity_cmd
Velocity Loop (outer, 50-100Hz)
    ↓ pitch_cmd (constrained to ±8°)
Angle Loop (inner, 200Hz)
    ↓ motor_voltage
Motor Driver (1kHz FOC)
```

**This is correct**. The issues are implementation details, not architecture.

---

### 5. Required Modifications

#### Mod 1: Outer Loop Decimation

Implement in `CascadeController`:
```cpp
uint32_t outer_loop_decimation_ = 4;  // Run outer loop every 4 inner loops
uint32_t step_counter_ = 0;
float last_pitch_cmd_ = 0.0f;

bool step(...) {
    step_counter_++;
    bool run_outer = (step_counter_ % outer_loop_decimation_) == 0;

    if (run_outer) {
        velocity_.step(...);  // Update at 50Hz
        last_pitch_cmd_ = pitch_cmd;
    }
    // Use last_pitch_cmd_ for angle loop
    angle_.step(...);  // Always runs at 200Hz
}
```

#### Mod 2: Constrain Outer Loop Output

```cpp
// In VelocityController::step()
pitch_cmd = clamp(pid_output, -max_tilt_command_, max_tilt_command_);
// Default max_tilt_command should be 0.14 rad (8°), not 0.3 rad (17°)
```

#### Mod 3: Enable Velocity Loop Integral

```cpp
// In CascadeController::setParams()
velocity_.setGains(p.velocity_kp, p.velocity_ki, 0.0f);  // Allow Ki
```

Add `velocity_ki` to `Params` struct.

#### Mod 4: Add Anti-windup for Velocity Loop

When pitch command saturates at max tilt, stop integrating velocity error.

---

### 6. Summary

| Aspect | Initial Analysis | Corrected Analysis |
|--------|------------------|-------------------|
| Architecture | Wrong (claimed) | Actually correct ✓ |
| Frequency | Mentioned | Primary issue ✗ |
| Output limits | Not mentioned | Needs tightening ✗ |
| Integral term | Not mentioned | Should add Ki ✗ |

**Architecture is valid. Implementation needs refinement.**

---

### 7. Follow-up Actions

- [ ] Implement outer loop decimation (50Hz outer / 200Hz inner)
- [ ] Add velocity_ki parameter
- [ ] Implement anti-windup for velocity loop when tilt-limited
- [ ] Reduce default max_tilt_command (17° → 8°)
- [ ] Update unit tests for decimation
- [ ] Document cascade tuning procedure

---

### References

- `src/control/cascade_controller.cpp` lines 162-193
- `src/control/velocity_controller.cpp` lines 16-41
- `src/main.cpp` lines 100-144 (balanceTask)
- `docs/decisions/2026-02-12-cascade-control-separation.md`
