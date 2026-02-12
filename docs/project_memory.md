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

- Current IMU uses complementary filter inline in `imuTask`
- Future: move sensor fusion to dedicated layer (Kalman/Mahony/Madgwick)
- Current balance controller combines angle + velocity control
- Future: separate into cascade: Position → Velocity → Angle → Torque

### Hardware Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Wheel diameter | 70mm | radius = 0.035m |
| Track width | 0.18m | wheel-to-wheel distance |
| Pole pairs | 7 | BLDC motor spec |
| Supply voltage | 12V | nominal |
| Max safe tilt | 35° | balance protection threshold |
