# Project Memory

Persistent record of constraints, rules, and technical decisions.

---

## 2026-02-15

### Current Architecture (Post-Refactor)

**Control Stack:**
```
Velocity Loop (Outer, 50Hz)
    ↓ pitch_cmd
Angle Loop (Inner, 200Hz)
    ↓ motor_voltage
Motor Driver (FOC, 1kHz)
```

**注意：** 位置环尚未实现（未来可能添加）

**Key Parameters:**
- `velocity_kp/ki/kd` - 速度环增益
- `angle_kp/ki` - 角度环P/I增益
- `angle_gyro_kd` - 手动D项（角速度阻尼）
- `angle_d_alpha` - D项低通滤波系数
- `yaw_kd` - 转向阻尼（纯速率控制）

**Yaw Control:** 当前简化为纯速率阻尼，无heading hold功能

---

## 2026-02-12

### Architecture Constraints

**Layered Control Stack (per CLAUDE.md):**
1. Hardware Abstraction Layer (HAL) - ✅ 已实现
2. Sensor Fusion / State Estimation - ⚠️ 部分（融合在HAL中）
3. Cascade Control Framework - ✅ 已实现（2层）
4. Safety and State Machine - ✅ 已实现
5. High-level behaviors / external command interface - ✅ 已实现

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

- ✅ ~~IMU complementary filter now in HAL (MPU6050_HAL)~~ - 已完成
- ✅ ~~Control cascade implemented: Velocity → Angle~~ - 已完成
- ⏳ Position loop not yet implemented (future: Position → Velocity → Angle)
- ⚠️ PID gains need retuning after cascade separation (conservative defaults set)
- ⚠️ D-term on angle loop uses filtered pitch rate - verify noise level in practice
- ⏳ Wheel lift detection removed during refactor - needs reimplementation

### Control Architecture (Current)

```
VelocityController (outer, Kv)
    ↓ pitch_cmd
AngleController (inner, Kp/Ki + gyro_kd)
    ↓ motor_voltage
Motor Driver
```

**Current:** 2-layer cascade (velocity + angle)
**Future:** May add Position loop as outermost layer

### Hardware Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Wheel diameter | 70mm | radius = 0.035m |
| Track width | 0.18m | wheel-to-wheel distance |
| Pole pairs | 7 | BLDC motor spec |
| Supply voltage | 12V | nominal |
| Max safe tilt | 35° | balance protection threshold |

---

## Legacy Notes (Pre-Refactor)

### Old Control Structure (Deprecated)

```
Monolithic balance_core.cpp
├── Angle PID (Kp, Ki, Kd)
├── Bias learning
├── Complementary filter (moved to HAL)
└── Direct velocity feedback (Kv)
```

**Migration completed:** See docs/decisions/2026-02-12-migrate-to-cascade-framework.md

---

## Documentation Status

| Document | Status | Notes |
|----------|--------|-------|
| CLAUDE.md | ✅ Current | Master guidelines |
| MEM.md | ✅ Current | Operating rules |
| project_memory.md | ✅ Current | This file |
| PARAMETER_TUNING_GUIDE.md | ✅ Updated v2.0 | Matches cascade params |
| CODE_REVIEW_ISSUES.md | ✅ Archived | Problems fixed |
| REFACTOR_PROPOSAL.md | ✅ Deleted | Migration completed |
| decisions/*.md | ✅ Preserved | ADR history |
