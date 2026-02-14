## Decision: Tune Default Parameters for Lightweight High-CoG Robot

Date: 2026-02-14
Author: Claude Code

### Context

After fixing the drift and pitch offset issues, user reported that the robot has specific physical characteristics that require different default parameters than the reference wheel-leg robot:

**Robot Specifications:**
- **Mass**: 500g (very lightweight)
- **Center of Gravity**: High (like "stilts" / "gāo qiāo")
- **Behavior**: Prone to front-back rocking oscillation

**Problem:**
The default parameters (borrowed from a heavier wheel-leg robot) caused instability:
- Insufficient damping → continuous front-back oscillation
- Over-aggressive restoring force → overshooting and ringing
- Too strong velocity tracking → "chase oscillation"

**Physics Analysis:**
1. **Low Inertia** (500g): Fast response, easily disturbed
2. **High CoG**: Large angular acceleration, like inverted pendulum
3. **Small Static Friction**: Less output integral compensation needed

### Decision

Adjust default parameters in `BalanceController::Params` for lightweight high-CoG characteristics:

**Primary Change: Double Gyro Damping (CRITICAL)**
```cpp
gyro_kp: 0.06f → 0.12f  (2× increase)
```
- Most important change for high-CoG robot
- Suppresses pitch_rate oscillation
- User specifically requested this

**Reduce Angle Restoring Force**
```cpp
angle_kp: 6.0f → 5.0f  (-17%)
```
- Avoid over-reaction and overshoot
- Combine with high damping for "gentle restore + strong damp" strategy

**Reduce Velocity/Position Tracking**
```cpp
speed_kp:    0.5f → 0.35f  (-30%)
distance_kp: 0.5f → 0.35f  (-30%)
```
- Lightweight robot has lower inertia
- Excessive gain causes chase oscillation

**Reduce Static Friction Compensation**
```cpp
lqr_u_ki: 15.0f → 10.0f  (-33%)
```
- Lightweight robot has lower static friction
- Excessive integral can introduce low-frequency wobble

**Tighten Safety Limits**
```cpp
max_tilt_deg: 25.0f → 20.0f  (-20%)
pid_limit:     8.0f →  6.0f  (-25%)
```
- High-CoG robot harder to recover beyond 20°
- Lower output limit prevents overshooting

### Alternatives Considered

**Alternative 1: Keep wheel-leg robot defaults**
- Rejected: Physical characteristics completely different (500g vs likely 2-3kg)

**Alternative 2: Only increase gyro_kp, keep others unchanged**
- Rejected: Other parameters also mismatched for lightweight robot

**Alternative 3: Use even higher gyro_kp (0.15 or 0.18)**
- Deferred: Start with 0.12 (2× increase), can increase further if needed

**Alternative 4: Add D-term to angle PID**
- Rejected: gyro PID already acts as derivative of angle, would be redundant

### Impact

**Files Modified:**
- `src/control/balance_controller.h` - Default values in `Params` struct

**Parameter Changes Summary:**

| Parameter | Old | New | Change | Reason |
|-----------|-----|-----|--------|--------|
| angle_kp | 6.0 | 5.0 | -17% | Reduce overshoot |
| gyro_kp ⭐ | 0.06 | 0.12 | +100% | Suppress oscillation |
| distance_kp | 0.5 | 0.35 | -30% | Lower inertia |
| speed_kp | 0.5 | 0.35 | -30% | Lower inertia |
| lqr_u_ki | 15.0 | 10.0 | -33% | Lower friction |
| max_tilt_deg | 25° | 20° | -20% | Harder to recover |
| pid_limit | 8.0 | 6.0 | -25% | Prevent overshoot |

**Behavioral Changes:**
1. **Oscillation suppression**: Much stronger damping (2× gyro_kp)
2. **Gentler response**: Reduced restoring force and output limits
3. **Earlier tilt protection**: Triggers at 20° instead of 25°
4. **Moderate tracking**: Velocity/position loops less aggressive

**Breaking Changes:**
- Existing saved parameters (`/params/balance.json`) will override these defaults
- Users need to delete saved params or manually update to use new defaults

### Follow-ups

- [ ] **Phase 1 Tuning**: Test gyro_kp (0.12-0.15) and angle_kp (4.5-5.5) for stable standing
- [ ] **Phase 2 Tuning**: Adjust speed_kp and distance_kp (0.25-0.45) for motion control
- [ ] **Phase 3 Tuning**: Fine-tune lqr_u_ki and other compensation parameters
- [ ] Add telemetry field for gyro_contribution to monitor damping effectiveness
- [ ] Consider adding parameter presets for different robot masses (500g, 1kg, 2kg)
- [ ] Document parameter tuning procedure in user guide

---

**Related ADRs:**
- `2026-02-14-fix-drift-and-pitch-offset-adaptation.md` - Previous parameter fixes
- `2026-02-14-replace-cascade-with-balance-controller.md` - Initial controller

**References:**
- User requirement: "500g lightweight, high CoG, like stilts"
- Physics: Inverted pendulum dynamics for high center of gravity
