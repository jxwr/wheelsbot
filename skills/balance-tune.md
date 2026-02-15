# Balance Tune Skill

**Purpose**: Automatically tune balance controller parameters using closed-loop feedback from real hardware.

**When to use**: When the user asks to tune, optimize, or stabilize the balance robot parameters.

**Capabilities**:
- Read current system state via serial debug interface
- Collect sensor data samples (pitch, pitch_rate, velocities)
- Analyze stability (variance, oscillation detection, settling time)
- Adjust parameters intelligently based on analysis
- Iterate until system converges to stable state
- Save optimized parameters to flash

---

## Execution Protocol

### Phase 1: Initial Assessment

1. **Connect and verify**:
   ```bash
   python3 -c "
   import serial, time
   ser = serial.Serial('/dev/cu.usbmodem*', 115200, timeout=1)
   ser.write(b'STATUS\n')
   time.sleep(0.2)
   print(ser.read(ser.in_waiting).decode())
   ser.close()
   "
   ```

2. **Get current parameters**:
   ```bash
   python3 -c "
   import serial, time
   ser = serial.Serial('/dev/cu.usbmodem*', 115200, timeout=1)
   ser.reset_input_buffer()
   ser.write(b'PARAMS\n')
   time.sleep(0.3)
   lines = [ser.readline().decode().strip() for _ in range(2)]
   for line in lines:
       print(line)
   ser.close()
   "
   ```

3. **Baseline stability test** (collect 100 samples @ 50Hz for 2 seconds):
   ```python
   import serial, time, math

   ser = serial.Serial('/dev/cu.usbmodem*', 115200, timeout=1)
   ser.reset_input_buffer()
   ser.write(b'STREAM 50\n')
   time.sleep(0.1)

   samples = []
   while len(samples) < 100:
       if ser.in_waiting > 0:
           line = ser.readline().decode().strip()
           if line.startswith('DATA,'):
               fields = line.split(',')
               pitch = float(fields[2])
               samples.append(pitch)

   ser.write(b'STREAM 0\n')
   ser.close()

   # Calculate statistics
   mean = sum(samples) / len(samples)
   variance = sum((x - mean)**2 for x in samples) / len(samples)
   std = math.sqrt(variance)

   # Count zero crossings (oscillation detection)
   crossings = sum(1 for i in range(1, len(samples))
                   if (samples[i-1] > mean and samples[i] < mean) or
                      (samples[i-1] < mean and samples[i] > mean))

   print(f"Pitch Mean: {mean:.4f}, Std: {std:.4f}, Crossings: {crossings}")
   ```

### Phase 2: Diagnosis

Based on baseline test results, classify the system:

| Condition | Classification | Root Cause |
|-----------|----------------|------------|
| `std > 0.05` | UNSTABLE | Insufficient damping or too high gain |
| `crossings > 20` | OSCILLATING | Proportional gain too high or D-term too low |
| `std < 0.01` | OVER-DAMPED | Too much damping, sluggish response |
| `0.01 <= std <= 0.05` | ACCEPTABLE | System functional but could be optimized |

### Phase 3: Parameter Adjustment Strategy

**If OSCILLATING** (crossings > 20):
1. Reduce `angle_kp` by 15%: `SET angle_kp <new_value>`
2. Increase `angle_gyro_kd` by 10%: `SET angle_gyro_kd <new_value>`

**If UNSTABLE** (std > 0.05):
1. Increase `angle_gyro_kd` by 20%: `SET angle_gyro_kd <new_value>`
2. Reduce `velocity_kp` by 10%: `SET velocity_kp <new_value>`

**If OVER-DAMPED** (std < 0.01, crossings < 5):
1. Increase `velocity_kp` by 15%: `SET velocity_kp <new_value>`
2. Reduce `angle_gyro_kd` by 10%: `SET angle_gyro_kd <new_value>`

**If ACCEPTABLE**:
1. Fine-tune: increase `velocity_kp` by 5% for faster response

### Phase 4: Iteration Loop

```python
MAX_ITERATIONS = 15
CONVERGENCE_STD = 0.015  # Target: pitch std < 0.015 rad

for iteration in range(1, MAX_ITERATIONS + 1):
    print(f"\n=== Iteration {iteration} ===")

    # 1. Sample system
    # (use baseline test code from Phase 1)

    # 2. Check convergence
    if std < CONVERGENCE_STD and crossings < 15:
        print("✓ CONVERGED!")
        break

    # 3. Adjust parameters
    # (use adjustment strategy from Phase 3)

    # 4. Apply changes via SET commands
    # Example: ser.write(b'SET angle_kp 1.35\n')

    # 5. Wait for system to settle
    time.sleep(1)
```

### Phase 5: Finalization

1. **Verify final performance**:
   - Collect 200 samples over 4 seconds
   - Ensure std < 0.02 and no persistent oscillation

2. **Save parameters**:
   ```bash
   python3 -c "
   import serial, time
   ser = serial.Serial('/dev/cu.usbmodem*', 115200, timeout=1)
   ser.write(b'SAVE\n')
   time.sleep(0.1)
   print(ser.readline().decode())
   ser.close()
   "
   ```

3. **Report final parameters**:
   - Use `PARAMS` command to show optimized values
   - Document improvement: before/after std comparison

---

## Safety Constraints

**NEVER exceed these limits** (prevents hardware damage or dangerous behavior):

- `angle_kp`: [0.5, 3.0]
- `angle_gyro_kd`: [0.2, 2.0]
- `velocity_kp`: [0.02, 0.3]
- `velocity_ki`: [0.0, 0.1]
- `yaw_kd`: [0.5, 2.5]

**If parameters hit limits**: Stop tuning and report to user.

---

## Example Usage

**User asks**: "tune the balance parameters"

**Agent response**:
1. Run Phase 1 baseline test
2. Print diagnosis (e.g., "System is OSCILLATING: pitch_std=0.082, crossings=34")
3. Apply adjustment (e.g., "Reducing angle_kp from 1.5 to 1.28")
4. Iterate 3-5 times
5. Report: "Converged after 4 iterations. Pitch std: 0.056 → 0.012"
6. Save parameters

---

## Notes

- Always collect samples with robot in **standing still** position
- If std keeps increasing, abort and suggest manual intervention
- High center-of-gravity robots (like this one) require conservative parameters
- Hard plastic wheels = low friction → use lower velocity_kp (< 0.1)
- Typical convergence: 3-8 iterations for initial tune, 1-3 for fine-tuning

---

## Quick Commands Reference

| Task | Command |
|------|---------|
| Get current params | `PARAMS` |
| Get single param | `GET angle_kp` |
| Set param | `SET angle_kp 1.5` |
| Start data stream | `STREAM 50` (50Hz) |
| Stop stream | `STREAM 0` |
| Save to flash | `SAVE` |
| System status | `STATUS` |

---

## Output Format

Always provide structured updates:

```
Iteration N:
  Pitch: mean=X.XXX, std=X.XXX, crossings=XX
  Status: [OSCILLATING|UNSTABLE|ACCEPTABLE|CONVERGED]
  Action: [parameter changes made]
```

Final report:
```
=== Tuning Complete ===
Initial state: std=0.082, crossings=34
Final state: std=0.012, crossings=8
Iterations: 5
Parameters saved: ✓

Key changes:
- angle_kp: 1.50 → 1.28 (-15%)
- angle_gyro_kd: 0.60 → 0.72 (+20%)
```
