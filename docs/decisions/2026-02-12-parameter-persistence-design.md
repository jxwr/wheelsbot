## Decision: Parameter Persistence with LittleFS

Date: 2026-02-12
Author: Claude Code

### Context

After migrating to `cascade_controller`, parameters were reset to defaults on every reboot. Users had to retune Kp/Ki/Kd after each power cycle. A persistence mechanism was needed to:

1. Save tuned parameters to non-volatile storage
2. Load them automatically on startup
3. Allow manual save trigger (don't auto-save on every change)
4. Handle missing/corrupted files gracefully (fallback to defaults)

### Decision

Use LittleFS filesystem with JSON file `/params/cascade.json` for parameter storage.

**Design choices:**
1. **Serialization format**: JSON (human-readable, debuggable)
2. **Storage location**: `/params/cascade.json` (dedicated directory)
3. **Trigger**: Manual via `save_params` WebSocket command
4. **Loading**: Automatic in `setup()` if file exists
5. **Granularity**: All cascade parameters in single file (atomic)

**API design:**
```cpp
// In CascadeController
struct Params { /* all parameters with defaults */ };
void setParams(const Params& p);   // Apply to controller
void getParams(Params& p) const;   // Read from controller

// In wifi_debug.cpp
bool saveCascadeParams(const Params& p);  // Write to LittleFS
bool loadCascadeParams(Params& p);        // Read from LittleFS
```

### Alternatives considered

1. **Preferences library (NVS)** - Rejected: Key-value only, no struct support. JSON is more flexible for nested parameters.

2. **EEPROM direct** - Rejected: No wear leveling, fixed size. LittleFS handles wear leveling.

3. **Auto-save on every parameter change** - Rejected: Flash wear concern. User explicitly saves when satisfied.

4. **Binary format** - Rejected: Human-readable JSON is easier to debug and manually edit.

5. **Separate files per parameter category** - Rejected: Atomic updates preferred. Partial saves can leave system in inconsistent state.

### Impact

**Files modified:**
- `src/control/cascade_controller.h` - Added `Params` struct, `setParams()`, `getParams()`
- `src/control/cascade_controller.cpp` - Implementation
- `src/wifi_debug.cpp` - Added `saveCascadeParams()`, `loadCascadeParams()`, `handleSaveParams()`
- `src/wifi_debug.h` - Exposed `loadCascadeParams()` for main.cpp
- `src/main.cpp` - Added `loadCascadeParams()` call in setup()

**JSON format:**
```json
{
  "angle_kp": 6.0000, "angle_ki": 0.0000, "angle_kd": 0.6000,
  "angle_d_alpha": 0.700, "angle_max_out": 6.00, "angle_integrator_limit": 2.00,
  "velocity_kp": 0.0000, "velocity_max_tilt": 0.300,
  "max_tilt": 0.611, "ramp_time": 0.500, "pitch_offset": 0.0000
}
```

**Precision**: 4 decimal places for gains, 3 for angles, 2 for limits.

### Follow-ups

- [x] Add "Save to Flash" button in frontend
- [ ] Add parameter import/export (JSON upload/download)
- [ ] Consider versioning for parameter file format changes
- [ ] Add validation on load (clamp to safe ranges)
- [ ] Document manual file editing procedure
