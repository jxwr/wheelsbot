#!/usr/bin/env python3
"""
Safe Balance Tuning Script
- Avoids accidental DISABLE commands
- Distinguishes between hand-held and self-standing
- Provides clear diagnostics
"""

import serial
import time
import math
import sys

PORT = '/dev/cu.usbmodem5B5F1233271'
BAUD = 115200


def connect():
    """Connect to serial port"""
    ser = serial.Serial(PORT, BAUD, timeout=2)
    time.sleep(0.3)
    ser.reset_input_buffer()
    return ser


def get_status(ser):
    """Get controller status"""
    ser.reset_input_buffer()
    ser.write(b'STATUS\n')
    time.sleep(0.2)
    data = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
    for line in data.split('\n'):
        if line and not line.startswith('MOT'):
            parts = line.split(',')
            if len(parts) >= 5 and parts[0] not in ['STATUS', 'state']:
                try:
                    return {
                        'state': int(parts[0]),
                        'faults': int(parts[1]),
                        'enabled': int(parts[2]),
                        'voltage': float(parts[3]),
                        'loop_hz': float(parts[4])
                    }
                except:
                    pass
    return None


def get_params(ser):
    """Get current parameters"""
    ser.reset_input_buffer()
    ser.write(b'PARAMS\n')
    time.sleep(0.3)
    data = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
    lines = [l for l in data.split('\n') if l.startswith('PARAMS,') or (not l.startswith('PARAMS') and ',' in l)]
    if len(lines) >= 2:
        headers = lines[0].split(',')[1:]
        values = lines[1].split(',')
        return dict(zip(headers, [float(v) if '.' in v else int(v) for v in values]))
    return None


def set_param(ser, name, value):
    """Set a single parameter"""
    ser.write(f'SET {name} {value}\n'.encode())
    time.sleep(0.1)
    response = ser.readline().decode('utf-8', errors='replace').strip()
    return 'OK' in response or 'ok' in response.lower()


def collect_data(ser, duration_sec, print_interval=0.5):
    """
    Collect sensor data without sending commands during collection
    Returns: list of (time, pitch, pitch_rate, motor_l, motor_r)
    """
    samples = []
    last_print = 0

    start = time.time()
    while time.time() - start < duration_sec:
        # Poll sensors (non-intrusive)
        ser.write(b'SENSORS\n')
        time.sleep(0.05)

        # Read all available data
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line.startswith('SENSORS,') and 'pitch' not in line:
                parts = line.split(',')
                if len(parts) >= 8:
                    try:
                        t = time.time() - start
                        pitch = float(parts[1])
                        pitch_rate = float(parts[2])
                        motor_l = float(parts[6])
                        motor_r = float(parts[7])
                        samples.append((t, pitch, pitch_rate, motor_l, motor_r))

                        # Print at interval
                        if t - last_print >= print_interval:
                            print(f"  {t:5.1f}s | Pitch: {pitch*57.3:6.1f}° | Motor: ({motor_l:5.1f}, {motor_r:5.1f})V")
                            last_print = t
                    except:
                        pass
        time.sleep(0.05)

    return samples


def analyze_stability(samples):
    """Analyze if robot is truly self-standing or hand-held"""
    if len(samples) < 20:
        return {'valid': False, 'reason': 'Insufficient data'}

    pitches = [s[1] for s in samples]
    motors = [(s[3] + s[4])/2 for s in samples]

    mean_pitch = sum(pitches) / len(pitches)
    std_pitch = math.sqrt(sum((x - mean_pitch)**2 for x in pitches) / len(pitches))
    max_motor = max(abs(m) for m in motors)
    mean_motor = sum(abs(m) for m in motors) / len(motors)

    # Detect oscillation (zero crossings)
    crossings = sum(1 for i in range(1, len(pitches))
                    if (pitches[i-1] > mean_pitch) != (pitches[i] > mean_pitch))

    # Heuristics for hand-held vs self-standing:
    # - Hand-held: low motor output, stable pitch (std < 0.02)
    # - Self-standing: higher motor output, active correction

    likely_handheld = (std_pitch < 0.02 and mean_motor < 2.0)

    return {
        'valid': True,
        'mean_pitch': mean_pitch,
        'std_pitch': std_pitch,
        'std_deg': std_pitch * 57.3,
        'max_motor': max_motor,
        'mean_motor': mean_motor,
        'crossings': crossings,
        'likely_handheld': likely_handheld,
        'duration': samples[-1][0] if samples else 0
    }


def diagnose_and_suggest(stats, current_params):
    """Diagnose issues and suggest parameter changes"""
    if not stats['valid']:
        return "Insufficient data for diagnosis"

    suggestions = []

    print("\n" + "="*60)
    print("ANALYSIS")
    print("="*60)
    print(f"  Standard Deviation: {stats['std_pitch']:.4f} rad ({stats['std_deg']:.2f}°)")
    print(f"  Max Motor Output:   {stats['max_motor']:.2f}V")
    print(f"  Mean Motor Output:  {stats['mean_motor']:.2f}V")
    print(f"  Zero Crossings:     {stats['crossings']}")
    print(f"  Likely Hand-Held:   {'YES ⚠️' if stats['likely_handheld'] else 'NO ✓'}")

    if stats['likely_handheld']:
        print("\n  ⚠️  WARNING: Data suggests robot is hand-held")
        print("      (Low motor output + very stable pitch)")
        print("      Please release and let it balance on its own!")
        return None

    # Real diagnosis
    print("\n  DIAGNOSIS:")

    if stats['std_pitch'] > 0.3:
        print("    🔴 CRITICAL: Robot falls immediately")
        print("       → Insufficient restoring force")
        suggestions.append(('angle_kp', current_params.get('angle_kp', 1.5) * 1.5,
                           f"angle_kp: {current_params.get('angle_kp', 1.5):.2f} → "
                           f"{current_params.get('angle_kp', 1.5) * 1.5:.2f} (more force)"))
        suggestions.append(('angle_max_out', 12.0,
                           f"angle_max_out: {current_params.get('angle_max_out', 6.0):.2f} → 12.00 (full power)"))

    elif stats['std_pitch'] > 0.15:
        print("    🟠 UNSTABLE: Large oscillations, can't maintain balance")
        print("       → Need more restoring force and damping")
        suggestions.append(('angle_kp', current_params.get('angle_kp', 1.5) * 1.3,
                           f"angle_kp: {current_params.get('angle_kp', 1.5):.2f} → "
                           f"{current_params.get('angle_kp', 1.5) * 1.3:.2f}"))
        suggestions.append(('angle_gyro_kd', current_params.get('angle_gyro_kd', 0.6) * 1.2,
                           f"angle_gyro_kd: {current_params.get('angle_gyro_kd', 0.6):.2f} → "
                           f"{current_params.get('angle_gyro_kd', 0.6) * 1.2:.2f}"))

    elif stats['std_pitch'] > 0.08:
        print("    🟡 MARGINAL: Standing but with noticeable wobble")
        print("       → Fine-tune gains")
        if stats['crossings'] > 10:
            suggestions.append(('angle_kp', current_params.get('angle_kp', 1.5) * 0.9,
                               f"angle_kp: {current_params.get('angle_kp', 1.5):.2f} → "
                               f"{current_params.get('angle_kp', 1.5) * 0.9:.2f} (reduce oscillation)"))
        else:
            suggestions.append(('angle_kp', current_params.get('angle_kp', 1.5) * 1.1,
                               f"angle_kp: {current_params.get('angle_kp', 1.5):.2f} → "
                               f"{current_params.get('angle_kp', 1.5) * 1.1:.2f}"))

    elif stats['std_pitch'] < 0.03:
        print("    🟢 GOOD: Stable standing!")
        print("       → Parameters are working")
        return []

    else:
        print("    ⚪ ACCEPTABLE: Minor adjustments may help")

    # Check motor saturation
    if stats['max_motor'] < 3.0 and current_params.get('angle_kp', 1.5) < 8.0:
        print(f"\n    ⚠️  Motor not using full power (max={stats['max_motor']:.1f}V)")
        print("       → Increase angle_kp significantly")

    return suggestions


def main():
    print("="*60)
    print("SAFE BALANCE TUNING")
    print("="*60)

    try:
        ser = connect()
    except Exception as e:
        print(f"Failed to connect: {e}")
        sys.exit(1)

    # Check initial status
    print("\n[1] Initial Status:")
    status = get_status(ser)
    if status:
        print(f"    State: {'RUNNING' if status['state'] else 'STOPPED'}")
        print(f"    Enabled: {'YES' if status['enabled'] else 'NO'}")
        print(f"    Faults: {status['faults']}")
        print(f"    Loop: {status['loop_hz']:.1f}Hz")

    # Get current params
    print("\n[2] Current Parameters:")
    params = get_params(ser)
    if params:
        for k in ['angle_kp', 'angle_gyro_kd', 'angle_max_out', 'velocity_kp']:
            if k in params:
                print(f"    {k:20s} = {params[k]:.4f}")

    # Check if already enabled
    if status and status['enabled']:
        print("\n    Controller already ENABLED")
    else:
        print("\n[3] Enabling controller...")
        ser.write(b'ENABLE\n')
        time.sleep(0.3)
        status = get_status(ser)
        if status:
            print(f"    Now: State={status['state']}, Enabled={status['enabled']}")

    # Collect data
    print("\n[4] Collecting data (10 seconds)...")
    print("    Please place robot upright and RELEASE when ready!")
    print("    Data collection starts in 2 seconds...")
    time.sleep(2)

    print("\n    Collecting... (showing every 0.5s)")
    samples = collect_data(ser, duration_sec=10, print_interval=0.5)

    print(f"\n    Collected {len(samples)} samples")

    # Analyze
    stats = analyze_stability(samples)
    suggestions = diagnose_and_suggest(stats, params)

    # Apply suggestions
    if suggestions:
        print("\n" + "="*60)
        print("SUGGESTED CHANGES")
        print("="*60)
        for name, value, description in suggestions:
            print(f"  {description}")

        print("\nApply changes? (y/n): ", end='')
        response = input().strip().lower()

        if response == 'y':
            print("\n[5] Applying changes...")
            for name, value, _ in suggestions:
                if set_param(ser, name, value):
                    print(f"    ✓ {name} = {value:.4f}")
                else:
                    print(f"    ✗ {name} failed")

            # Save
            ser.write(b'SAVE\n')
            time.sleep(0.2)
            print("    ✓ Saved to flash")

            # Verify
            new_params = get_params(ser)
            if new_params:
                print("\n[6] New Parameters:")
                for name, _, _ in suggestions:
                    if name in new_params:
                        print(f"    {name:20s} = {new_params[name]:.4f}")
    elif suggestions == []:
        print("\n" + "="*60)
        print("PARAMETERS ARE GOOD!")
        print("="*60)
        print("No changes needed. Save these parameters:")
        ser.write(b'SAVE\n')
        print("    ✓ Saved")

    # Final status - DO NOT DISABLE
    print("\n[7] Final Status:")
    status = get_status(ser)
    if status:
        print(f"    State: {'RUNNING' if status['state'] else 'STOPPED'}")
        print(f"    Enabled: {'YES' if status['enabled'] else 'NO'}")

    print("\n" + "="*60)
    print("NOTE: Controller remains ENABLED")
    print("Use 'DISABLE' command manually if needed")
    print("="*60)

    ser.close()


if __name__ == '__main__':
    main()
