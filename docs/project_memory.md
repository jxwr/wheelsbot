# Project Memory

Persistent record of constraints, rules, and technical decisions.

---

## 2026-02-21: Unix C Style Refactor Complete

### Current Code Structure

```
src/
├── main.cpp           # Hardware + FreeRTOS tasks + Serial commands (574 lines)
├── robot.h            # Data structure definitions (189 lines)
├── balance.cpp        # Balance control algorithm (322 lines)
├── imu_mpu6050.cpp    # IMU reading (128 lines)
├── wifi_ctrl.cpp      # WiFi/WebSocket/OTA (479 lines)
└── pins.h             # GPIO definitions (43 lines)

Total: ~1700 lines, 6 files
```

### Critical Constraints

1. **Wheel Direction**:
   - Left wheel: no inversion, right wheel: inverted
   - Wrong signs will cause control loop chaos

2. **IMU Units**:
   - `g_imu.pitch` outputs angle (°)
   - `g_imu.gy` outputs angular velocity (rad/s)

3. **Control Frequency**:
   - Balance loop must be ≥200Hz
   - FOC loop must be =1kHz

---

## 2026-02-14: Control Parameter Tuning

### Parameter Lower Bounds

- `speed_kp >= 0.3` (below this causes drift)
- `gyro_kp = 0.08~0.12` (insufficient damping causes oscillation)

### CoG Self-Adaptation

- Activation: `|lqr_u| < 5 && |speed| < 2 && no joystick input`
- Adjustment direction: `pitch_offset -= zeropoint_kp × distance_ctrl`
- `zeropoint_kp = 0.002` (too high causes oscillation)

### High-CoG Robot Tuning

1. First: tune `gyro_kp` (0.12-0.15) + `angle_kp` (4.5-5.5) for stable standing
2. Then: tune `speed_kp` + `distance_kp` (0.25-0.45) for motion control
3. Finally: tune `lqr_u_ki` and other compensation parameters

---

## Hardware Parameters

| Parameter | Value |
|-----------|-------|
| Wheel radius | 0.03m |
| Pole pairs | 7 |
| Supply voltage | 12V |
| Max safe tilt | 60° |

---

## Technical Debt

- [ ] IMU gyro bias calibration (currently no startup calibration)
- [ ] Auto-load parameters from Flash (currently uses code defaults)
- [ ] Magnetometer fusion to eliminate yaw drift (long-term)
