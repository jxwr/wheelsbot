# WheelsBot Development Guide

Two-wheel self-balancing robot running on ESP32-S3.

---

## 1. Project Overview

### 1.1 Core Architecture

```
src/
├── main.cpp           # Hardware instances + FreeRTOS tasks + Serial commands
├── robot.h            # Data structure definitions (config/state/debug)
├── balance.cpp        # Balance control algorithm
├── imu_mpu6050.cpp    # IMU reading
├── wifi_ctrl.cpp      # WiFi/WebSocket/OTA
└── pins.h             # GPIO definitions
```

**Design Principle**: Data in structs, algorithms as functions, global state visibility.

### 1.2 Tech Stack

| Component | Technology |
|-----------|------------|
| Platform | ESP32-S3 (Arduino Framework) |
| Motor Control | SimpleFOC (FOC brushless motor) |
| RTOS | FreeRTOS |
| IMU | MPU6050 (complementary filter) |
| Encoder | AS5600 (I2C magnetic encoder) |
| Communication | WiFi AP + WebSocket |
| Build System | PlatformIO |

---

## 2. Control System

### 2.1 Task Frequencies

| Task | Frequency | Core | Priority |
|------|-----------|------|----------|
| FOC Motor | 1kHz | 1 | 5 |
| Balance Control | 200Hz | 0 | 4 |
| IMU Reading | 200Hz | 0 | 5 |
| WiFi Telemetry | 20Hz | 0 | 1 |

### 2.2 Control Loop Structure

LQR-decomposed parallel PID control:

```
                    ┌─ angle_kp × angle_error ────┐
                    │                              │
                    ├─ gyro_kp × pitch_rate ──────┤
                    │                              │
lqr_u ──────────────┼─ distance_kp × pos_error ───┼──→ motor_output
                    │                              │
                    ├─ speed_kp × wheel_vel ──────┤
                    │                              │
                    └─ lqr_u_kp × lqr_u + ki × ∫ ─┘

differential:  motor_L = -0.5 × (lqr_u + yaw)
               motor_R = -0.5 × (lqr_u - yaw)
```

### 2.3 Key Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `angle_kp` | 1.0 | Angle loop gain |
| `gyro_kp` | 0.08 | Angular velocity damping |
| `distance_kp` | 0.3 | Position loop gain |
| `speed_kp` | 1.0 | Velocity loop gain |
| `zeropoint_kp` | 0.002 | CoG adaptation rate |
| `pitch_offset` | 0.0 | CoG offset compensation (°) |
| `max_tilt_deg` | 60.0 | Max tilt protection (°) |

### 2.4 CoG Self-Adaptation

Automatically adjusts `pitch_offset` when robot is stationary to compensate for center of gravity offset:

**Activation Conditions**:
- `|lqr_u| < 5.0` (small output)
- No joystick input
- `|speed| < 2.0 rad/s`
- Wheels not lifted

**Adjustment Direction**: `pitch_offset -= zeropoint_kp × distance_ctrl`

---

## 3. Wheel Direction Convention

**IMPORTANT**: Wheel velocity/position signs must be correct, otherwise control loop will be chaotic.

```c
// Correct configuration (consistent with main branch)
float wL_raw = sensor_left.getVelocity();   // Left wheel: no inversion
float wR_raw = -sensor_right.getVelocity(); // Right wheel: inverted

g_wheel.x_l = sensor_left.getAngle();    // Left wheel: no inversion
g_wheel.x_r = -sensor_right.getAngle();  // Right wheel: inverted
```

---

## 4. Serial Debugging

### 4.1 Command Format

```
param=value    # Set parameter
param?         # Query parameter
dump           # Show all parameters
telem          # Show telemetry data
save           # Save parameters to Flash
reset          # Reset controller
help           # Help
```

### 4.2 Debug Tools

```bash
cd tools
python3 debug_check_params.py      # Check parameters
python3 debug_monitor_cog.py       # Monitor CoG adaptation
python3 debug_analyze_oscillation.py  # Analyze oscillation
```

---

## 5. Naming Convention

### 5.1 Keep Full Words for Core Terms

`angle`, `gyro`, `distance`, `speed`, `yaw`, `pitch`, `roll`

### 5.2 Abbreviation Rules

| Full Word | Abbreviation |
|-----------|--------------|
| contribution | ctrl |
| enable | en |
| zeropoint | zero |
| velocity | vel |
| compensated | comp |
| adjustment | adj |
| joystick | joy |
| output | out |

### 5.3 Examples

```c
// Correct
float angle_ctrl, gyro_ctrl, distance_ctrl;
bool balance_en, motor_en;
float motor_l, motor_r, w_l, w_r;

// Wrong
float angle_contribution, wL, xR;
```

---

## 6. Hardware Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Wheel diameter | 60mm | radius = 0.03m |
| Pole pairs | 7 | BLDC motor |
| Supply voltage | 12V | nominal |
| I2C1 | SDA=3, SCL=9 | IMU + left encoder |
| I2C2 | SDA=2, SCL=1 | Right encoder |

---

## 7. Development Workflow

### 7.1 Explore-Plan-Code-Verify

1. **Explore**: Use `grep`/`Glob` to locate relevant code
2. **Plan**: Output modification plan, wait for confirmation
3. **Code**: Implement changes, maintain existing style
4. **Verify**: Build with `pio run -e esp32s3`

### 7.2 Commit Convention

- Use [Conventional Commits](https://www.conventionalcommits.org/)
- Types: `feat:`, `fix:`, `refactor:`, `chore:`
- Example: `fix(control): correct wheel velocity sign convention`

### 7.3 Branch Strategy

- `main` - Stable version
- `refactor/unix-c-style` - Current development version
- Feature branches created from main

---

## 8. Decision Records

Important decisions recorded in `docs/decisions/`, format:

```markdown
# Decision: <Title>

Date: YYYY-MM-DD

## Context
Background description

## Decision
Selected solution

## Impact
Affected scope
```

---

## 9. Troubleshooting

### 9.1 Robot Cannot Balance

1. Check wheel direction signs are correct
2. Check IMU initialized successfully (`telem` command)
3. Check parameters are reasonable (`dump` command)

### 9.2 Robot Drifting

1. Check `speed_kp` not too small (<0.3)
2. Check CoG adaptation is active (`debug_monitor_cog.py`)
3. Check `pitch_offset` has converged

### 9.3 Robot Oscillating

1. Increase `gyro_kp` (0.08 → 0.12)
2. Decrease `angle_kp`
3. Use `debug_analyze_oscillation.py` to locate oscillation source
