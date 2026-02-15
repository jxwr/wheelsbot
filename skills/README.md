# Balance Bot Skills

This directory contains reusable AI agent skills for the balance robot project.

## Available Skills

### 🔄 balance-init

**Purpose**: Initialize balance controller parameters after firmware upload.

**Files**:
- `../tools/init_params.py` - Python implementation

**Usage**:

Automatically detect serial port and initialize:
```bash
python3 tools/init_params.py
```

Or specify serial port manually:
```bash
python3 tools/init_params.py /dev/ttyUSB0
```

**When to use**:
- After uploading new firmware
- After factory reset
- When switching to different hardware
- As first step in setup workflow

**Flow**:
```
Compile → Upload → Init Params → Tune
   ↑___________________________|
```

---

### 🎯 balance-tune

**Purpose**: Automatically tune balance controller parameters using closed-loop feedback from hardware.

**Files**:
- `balance-tune.md` - Detailed skill documentation and protocols
- `balance-tune.skill.yaml` - Skill metadata and configuration
- `../tools/auto_tune.py` - Python implementation

**Usage**:

Simply ask Claude Code to tune the balance:
```
User: "tune the balance parameters"
User: "optimize the controller for better stability"
User: "auto tune"
```

**Prerequisites**:
- Firmware uploaded (`pio run --target upload`)
- Parameters initialized (`python3 tools/init_params.py`)
- Robot positioned upright and ready

Claude will:
1. Read current parameters via serial debug interface
2. Collect baseline stability data (pitch variance, oscillation)
3. Diagnose system state (OSCILLATING, UNSTABLE, ACCEPTABLE, etc.)
4. Iteratively adjust parameters based on feedback
5. Save optimized parameters to flash

**Manual Usage**:
```bash
python3 tools/auto_tune.py
```

**Expected Output**:
```
=== Balance Controller Auto-Tune ===

Reading current parameters...
  angle_kp: 1.5000
  angle_gyro_kd: 0.6000
  velocity_kp: 0.1500

Running baseline stability test...
  Mean: 0.0121, Std: 0.0823
  Crossings: 34, Status: OSCILLATING

=== Iteration 1 ===
  Pitch: mean=0.0120, std=0.0712, crossings=28
  Status: OSCILLATING
  → Reducing angle_kp by 15%, increasing angle_gyro_kd by 10%
  angle_kp: 1.5000 → 1.2750
  angle_gyro_kd: 0.6000 → 0.6600

...

✓ CONVERGED!

=== Tuning Complete ===
Initial state: std=0.0823, crossings=34
Final state: std=0.0127, crossings=9

Improvement: 84.6%

Final parameters:
  angle_kp: 1.2750
  angle_gyro_kd: 0.7260
  velocity_kp: 0.1500
```

**Requirements**:
- Balance robot connected via USB serial
- Robot positioned safely for stability testing
- Python 3 with pyserial installed

**Safety**:
- All parameters constrained to safe ranges
- Tuning aborts if stability degrades
- User can interrupt with Ctrl+C
- No parameters saved unless tuning succeeds

---

## Creating New Skills

To create a new skill:

1. **Create documentation**: `your-skill.md`
   - Purpose and capabilities
   - Execution protocol (step-by-step)
   - Safety constraints
   - Example usage

2. **Create configuration**: `your-skill.skill.yaml`
   - Metadata (name, version, description)
   - Trigger phrases
   - Command to execute
   - Parameters and defaults

3. **Create implementation**: `../tools/your-skill.py` or script
   - Main functionality
   - Clear output format (preferably CSV for AI parsing)
   - Error handling

4. **Update this README**
   - Add skill to the list above
   - Provide usage examples

---

## Skill Development Guidelines

**Good skills are**:
- ✅ Self-contained (single responsibility)
- ✅ Deterministic (reproducible results)
- ✅ Well-documented (protocols + examples)
- ✅ Safe (constraints + validation)
- ✅ Observable (clear progress output)

**Output format**:
- Use CSV or structured text for AI parsing
- Avoid JSON where possible (token efficiency)
- Include progress indicators
- Report errors clearly

**Testing**:
- Test manually before committing
- Verify all edge cases
- Document failure modes

---

## Future Skills Ideas

- `balance-diagnose` - Health check and fault diagnosis
- `balance-test` - Automated test suite (sensor health, motor response, etc.)
- `balance-calibrate` - IMU and encoder calibration
- `balance-learn` - Collect training data for ML models
- `balance-simulate` - Test parameters in simulation before deploying

---

**Last updated**: 2026-02-15

## Workflow Summary

**Complete setup workflow**:
```bash
# 1. Compile
pio run

# 2. Upload firmware
pio run --target upload

# 3. Initialize parameters
python3 tools/init_params.py

# 4. Tune parameters (optional, if stability issues)
python3 tools/auto_tune.py
```
