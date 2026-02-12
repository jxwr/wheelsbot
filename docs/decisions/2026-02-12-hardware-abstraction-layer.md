## Decision: Hardware Abstraction Layer (HAL) for Sensor and Actuator Interfaces

Date: 2026-02-12
Author: Claude Code

### Context

Current codebase has hardcoded hardware dependencies:
- MPU6050 IMU directly accessed in `imuTask`
- AS5600 encoders directly accessed in `focTask`
- Hardware-specific code mixed with control logic in `main.cpp`

This prevents:
- Swapping IMU types (MPU6050 → ICM42688/BNO085) without modifying control logic
- Testing control algorithms in simulation
- Clean separation of concerns per CLAUDE.md Layered Control Stack

### Decision

Introduce a Hardware Abstraction Layer (HAL) with the following structure:

1. **Interface Classes** (pure virtual):
   - `IMUSensor` - provides acceleration (m/s²), gyroscope (rad/s), orientation
   - `WheelEncoder` - provides angle (rad), velocity (rad/s)

2. **Concrete Implementations**:
   - `MPU6050_HAL` - current IMU implementation
   - `AS5600_HAL` - current encoder implementation

3. **HardwareManager**:
   - Centralized hardware lifecycle management
   - Factory-like initialization

Directory structure:
```
src/hardware/
├── imu_interface.h         # IMUSensor abstract class
├── imu_mpu6050.h/cpp      # MPU6050 implementation
├── encoder_interface.h     # WheelEncoder abstract class
├── encoder_as5600.h/cpp   # AS5600 implementation
└── hardware_manager.h/cpp  # Hardware initialization
```

### Alternatives considered

1. **Template-based HAL**: Rejected - adds complexity, harder to debug, not idiomatic for embedded C++
2. **C-style function pointers**: Rejected - loses type safety, harder to extend
3. **Keep current structure**: Rejected - violates CLAUDE.md plugin-based hardware requirement
4. **Full dependency injection framework**: Rejected - overkill for current scope, can evolve later

### Impact

**Files modified:**
- `src/main.cpp` - uses HAL instead of direct hardware access
- `src/imu_mpu6050.cpp` - deprecated, functionality moved to HAL

**New files:**
- `src/hardware/*.h/cpp` - HAL interface and implementations

**API changes:**
- `imuTask` uses `IMUSensor::read()` instead of direct `mpu_read14()`
- `focTask` uses `WheelEncoder::read()` instead of direct `getVelocity()`

**Runtime impact:**
- Minimal: virtual function dispatch adds ~2-4 cycles per call
- FOC loop remains at 1kHz, balance loop at 200Hz

### Follow-ups

1. Create HAL interfaces and implementations
2. Migrate `main.cpp` to use HAL
3. Verify real-time performance (no jitter in control loops)
4. Add HAL support for future motor driver abstraction
5. Document HAL extension pattern for new hardware types
