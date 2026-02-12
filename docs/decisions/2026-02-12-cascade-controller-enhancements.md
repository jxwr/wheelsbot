## Decision: Cascade Controller Enhancements

Date: 2026-02-12
Author: Claude Code
Status: Implemented

---

### Summary

Enhanced the cascade controller with frequency separation, velocity Ki support, and comprehensive telemetry for better tuning and debugging.

---

### Changes Made

#### 1. Outer Loop Decimation (Frequency Separation)

**Problem**: Inner and outer loops both ran at 200Hz, violating bandwidth separation principle.

**Solution**: Implemented 4:1 decimation ratio (outer loop 50Hz, inner loop 200Hz).

**Implementation**:
```cpp
// In CascadeController
uint32_t outer_decimation_ = 4;  // Run outer every 4 inner cycles
uint32_t step_counter_ = 0;
float last_pitch_cmd_ = 0.0f;    // Hold last outer output

bool step(...) {
    step_counter_++;
    bool run_outer = (step_counter_ % outer_decimation_) == 0;
    if (run_outer) {
        velocity_.step(...);       // Update at 50Hz
        last_pitch_cmd_ = pitch_cmd;
    }
    // Use last_pitch_cmd_ for inner loop (always 200Hz)
}
```

**Files Modified**:
- `src/control/cascade_controller.h` - Added decimation members
- `src/control/cascade_controller.cpp` - Implemented decimation logic

---

#### 2. Velocity Loop Ki Support

**Problem**: Velocity loop only had P-term, no integral action for steady-state error elimination.

**Solution**: Added `velocity_ki` parameter to Params struct.

**Changes**:
- Added `float velocity_ki = 0.0f;` to `Params` struct
- Updated `setParams()` to apply Ki gain
- Updated `getParams()` to read Ki gain
- Updated JSON serialization/deserialization in `wifi_debug.cpp`
- Maintains backward compatibility (defaults to 0)

**Files Modified**:
- `src/control/cascade_controller.h` - Added velocity_ki field
- `src/control/cascade_controller.cpp` - Apply Ki in setParams
- `src/wifi_debug.cpp` - JSON serialization for velocity_ki

---

#### 3. Reduced Default Max Tilt

**Problem**: Default `velocity_max_tilt` was 0.3 rad (~17°), too large for stability.

**Solution**: Reduced to 0.14 rad (~8°).

**File Modified**:
- `src/control/velocity_controller.h` - Changed default from 0.3f to 0.14f

---

#### 4. Frequency Statistics

**Added**: Real-time measurement of actual loop execution frequencies.

```cpp
struct FrequencyStats {
    float outer_hz = 0.0f;  // Measured outer loop frequency
    float inner_hz = 0.0f;  // Measured inner loop frequency
};
```

**Implementation**: Counts steps over 1-second windows and calculates Hz.

**Files Modified**:
- `src/control/cascade_controller.h` - Added FrequencyStats struct and members
- `src/control/cascade_controller.cpp` - Frequency calculation in step()

---

#### 5. Limit Status Detection

**Added**: Real-time detection of controller saturation.

```cpp
struct LimitStatus {
    bool velocity_saturated = false;
    bool angle_saturated = false;
    bool motor_saturated = false;
};
```

**Implementation**: Checks if outputs exceed 99% of limits.

**Files Modified**:
- `src/control/cascade_controller.h` - Added LimitStatus struct
- `src/control/cascade_controller.cpp` - Saturation detection in step()

---

#### 6. Runtime Statistics

**Added**: Session-persistent statistics (reset on reboot).

```cpp
struct RuntimeStats {
    uint32_t total_runtime_sec = 0;
    uint32_t fault_count_total = 0;
    float max_pitch_ever = 0.0f;
    float min_pitch_ever = 0.0f;
};
```

**Files Modified**:
- `src/control/cascade_controller.h` - Added RuntimeStats struct
- `src/control/cascade_controller.cpp` - Statistics tracking

---

#### 7. Enhanced Web Interface

**Complete rewrite** of `data/index.html` with:

| Feature | Description |
|---------|-------------|
| Chinese Labels | All parameters show Chinese name + English code |
| 4-Column Layout | Optimized for tablets (8-inch and up) |
| 3D Attitude View | Preserved Three.js visualization |
| Real-time Charts | Canvas-based pitch and motor output curves (10s history) |
| Frequency Display | Shows actual measured inner/outer loop frequencies |
| Saturation Indicators | LED-style indicators for limit status |
| Runtime Statistics | Run time, fault count, max angle |
| Responsive Design | Adapts to 4-col → 2-col → 1-col based on width |

**Files Modified**:
- `data/index.html` - Complete rewrite

---

#### 8. Extended Telemetry Protocol

**New JSON format**:
```json
{
  "type": "telem",
  "t": 12345,
  "pitch": 0.05,
  "pitch_deg": 2.8,
  "vel": 0.5,
  "vel_target": 0.0,
  "vel_error": -0.5,
  "vel_i": 0.1,
  "pitch_cmd": 0.08,
  "pitch_error": -0.02,
  "pitch_i": 0.3,
  "motor": 3.5,
  "outer_hz": 50.2,
  "inner_hz": 200.0,
  "saturated": 3,
  "state": 1,
  "fault": 0,
  "runtime": 3600,
  "fault_cnt": 5,
  "max_pitch": 0.5
}
```

**Files Modified**:
- `src/wifi_debug.cpp` - Extended sendTelemetry()

---

### Parameter Persistence Compatibility

**Backward Compatible**: Old parameter files (11 fields without velocity_ki) are still readable.

**New Format**: 12 fields including velocity_ki.

**Implementation**:
```cpp
// Try new format first (12 parameters)
int matched = sscanf(..., "...velocity_ki...", ...);
if (matched == 12) return true;

// Fallback to old format (11 parameters)
matched = sscanf(..., ...);  // without velocity_ki
if (matched == 11) {
    p.velocity_ki = 0.0f;  // Default for old files
    return true;
}
```

---

### API Additions

```cpp
// CascadeController
void setOuterLoopDecimation(uint32_t ratio);  // Default: 4
void getFrequencyStats(FrequencyStats& out) const;
void getLimitStatus(LimitStatus& out) const;
void getRuntimeStats(RuntimeStats& out) const;
void resetRuntimeStats();

// Params struct now includes
float velocity_ki;  // NEW
```

---

### Testing Recommendations

1. **Frequency Verification**: Check that outer_hz ≈ 50Hz, inner_hz ≈ 200Hz
2. **Velocity Ki Tuning**: Start with small values (0.01-0.05) to avoid oscillation
3. **Saturation Monitoring**: Watch velocity saturation indicator when accelerating
4. **Max Tilt**: Verify 8° limit prevents dangerous lean angles

---

### Files Modified Summary

| File | Changes |
|------|---------|
| `src/control/cascade_controller.h` | +90 lines: Decimation, stats structs, new methods |
| `src/control/cascade_controller.cpp` | +80 lines: Decimation logic, stats tracking |
| `src/control/velocity_controller.h` | 1 line: Default max_tilt 0.3→0.14 |
| `src/wifi_debug.cpp` | +40 lines: velocity_ki support, extended telemetry |
| `data/index.html` | Complete rewrite: 518 lines |

---

### Next Steps

- [ ] Deploy to ESP32 and verify frequencies
- [ ] Tune velocity Ki with new interface
- [ ] Document tuning procedure
- [ ] Add position loop (future)
