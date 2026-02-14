# Project Memory

Persistent record of constraints, rules, and technical decisions.

---

## 2026-02-12

### Architecture Constraints

**Layered Control Stack (per CLAUDE.md):**
1. Hardware Abstraction Layer (HAL) - this iteration
2. Sensor Fusion / State Estimation
3. Cascade Control Framework
4. Safety and State Machine
5. High-level behaviors / external command interface

Each layer must only depend on the layer directly below it.

**HAL Design Principles:**
- Use abstract base classes with virtual interfaces
- Minimize virtual call overhead (acceptable for 200Hz-1kHz loops)
- Keep HAL headers Arduino-agnostic where possible
- Static allocation preferred over heap

### Control Loop Frequencies

| Loop | Frequency | Core | Priority |
|------|-----------|------|----------|
| FOC / Motor | 1kHz | 1 | 5 |
| Balance / Angle | 200Hz | 0 | 4 |
| IMU Read | 200Hz | 0 | 5 |
| Telemetry | ~10Hz | 0 | 1 |

Balance loop must run at >= 200Hz to maintain stability.

### Technical Debt

- IMU complementary filter now in HAL (MPU6050_HAL) - future: add Mahony/Madgwick fusion layer
- Control cascade implemented: Velocity → Angle
- Position loop not yet implemented (future: Position → Velocity → Angle)
- PID gains need retuning after cascade separation (conservative defaults set)
- D-term on angle loop uses filtered pitch rate - verify noise level in practice

### Control Architecture (New)

```
PositionController (future)
    ↓ pitch_cmd
VelocityController (outer, Kv)
    ↓ pitch_cmd
AngleController (inner, Kp/Ki/Kd)
    ↓ motor_voltage
Motor Driver
```

Current: Inner loop only (velocity loop disabled with Kv=0)
Tune inner loop first, then enable outer loop.

### Hardware Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Wheel diameter | 70mm | radius = 0.035m |
| Track width | 0.18m | wheel-to-wheel distance |
| Pole pairs | 7 | BLDC motor spec |
| Supply voltage | 12V | nominal |
| Max safe tilt | 35° | balance protection threshold |

---

## 2026-02-14

### Critical Controller Parameters (Post-Drift Fix)

**Speed Loop Gain:**
- `speed_kp = 0.5f` (minimum) - values below 0.3 cause robot drift
- Do NOT reduce below 0.3 without testing on hardware
- Reference: wheel-leg robot uses 0.7, we use 0.5 as conservative middle ground

**Distance Zero-Point Reset:**
- Fast-reset threshold: `speed > 30 rad/s` (~1 m/s)
- Threshold must be high enough to allow normal movement without disabling position control
- Values below 20 rad/s cause position hold to fail during commanded motion

**Pitch Offset Self-Adaptation:**
- Sign MUST be `pitch_offset +=` (not `-=`)
- Logic: forward drift → positive distance_ctrl → increase pitch_offset → lean back → correct drift
- Activation condition: `lqr_u < 5.0 && speed < 1.0 && no joystick input`
- Activates only when robot is nearly stationary with persistent drift

### Known Issues Fixed

- ✅ Robot drift (溜车) - caused by speed_kp too small (was 0.02, now 0.35)
- ✅ Pitch offset adaptation inverted sign - would amplify drift instead of correcting
- ✅ Position control disabled too easily - fast-reset at 15 rad/s was too low
- ✅ Adaptation never activated - condition `distance_ctrl < 4.0` was too strict

### Lightweight High-CoG Robot Parameters (500g, high center of gravity)

**Critical: Strong Damping Required**
- `gyro_kp = 0.12f` (doubled from 0.06) - MOST IMPORTANT for suppressing oscillation
- High-CoG robots behave like inverted pendulums with large angular acceleration
- Insufficient damping causes continuous front-back rocking

**Reduced Gains for Lightweight Body**
- `angle_kp = 5.0f` (reduced from 6.0) - avoid over-reaction
- `speed_kp = 0.35f` (reduced from 0.5) - lower inertia needs gentler tracking
- `distance_kp = 0.35f` (reduced from 0.5) - prevent chase oscillation
- `lqr_u_ki = 10.0f` (reduced from 15.0) - lower static friction

**Tighter Safety Limits**
- `max_tilt_deg = 20°` (reduced from 25°) - high-CoG harder to recover
- `pid_limit = 6.0` (reduced from 8.0) - prevent output overshoot

**Tuning Priority:**
1. First: gyro_kp (0.12-0.15) + angle_kp (4.5-5.5) for stable standing
2. Then: speed_kp + distance_kp (0.25-0.45) for motion control
3. Finally: lqr_u_ki and other compensation parameters
