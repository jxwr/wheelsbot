#!/usr/bin/env python3
"""
Balance Controller Parameter Initialization Script
Part of balance-tune skill - automatically initialize parameters after firmware upload

Usage:
    python3 tools/init_params.py [serial_port]

Default port: /dev/cu.usbmodem5B5F1233271 (macOS)
              /dev/ttyUSB0 (Linux)
"""

import serial
import time
import sys

# Default parameters for balance controller
# These are conservative values suitable for initial testing
DEFAULT_PARAMS = [
    ('velocity_kp', 0.15),
    ('velocity_ki', 0.02),
    ('velocity_kd', 0.0),
    ('velocity_max_tilt', 0.14),
    ('angle_kp', 1.5),
    ('angle_ki', 0.0),
    ('angle_gyro_kd', 0.6),
    ('angle_d_alpha', 0.7),
    ('angle_max_out', 6.0),
    ('yaw_kd', 1.2),
    ('max_tilt', 0.6),
    ('ramp_time', 0.5),
    ('pitch_offset', 0.0),
    ('pitch_cmd_rate_limit', 0.5),
    ('sensor_timeout', 0.2),
]


def find_serial_port():
    """Auto-detect serial port"""
    import glob

    # Common patterns
    patterns = [
        '/dev/cu.usbmodem*',      # macOS USB CDC
        '/dev/tty.usbmodem*',     # macOS USB CDC (alternate)
        '/dev/ttyUSB*',           # Linux USB
        '/dev/ttyACM*',           # Linux CDC ACM
        '/dev/cu.SLAB_USB*',      # macOS CP210x
        '/dev/cu.wchusbserial*',  # macOS CH340
    ]

    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]

    return None


def init_params(port=None, params=None):
    """Initialize balance controller parameters"""

    if params is None:
        params = DEFAULT_PARAMS

    if port is None:
        port = find_serial_port()
        if port is None:
            print("Error: Could not auto-detect serial port")
            print("Please specify port manually: python3 init_params.py /dev/ttyUSB0")
            sys.exit(1)

    print(f"=== Balance Controller Parameter Initialization ===")
    print(f"Port: {port}")
    print()

    try:
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(0.5)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    # Clear buffer
    ser.reset_input_buffer()

    # Reset to defaults first
    print("Resetting to defaults...")
    ser.write(b'RESET\n')
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Set parameters
    print("Setting parameters:")
    for name, value in params:
        cmd = f'SET {name} {value}\n'.encode()
        ser.write(cmd)
        time.sleep(0.05)
        response = ser.readline().decode().strip()
        status = "OK" if response == "OK" else f"ERR: {response[:30]}"
        print(f"  {name:20s} = {value:8.4f}  [{status}]")

    # Save to flash
    print()
    print("Saving to flash...")
    ser.write(b'SAVE\n')
    time.sleep(0.2)
    response = ser.readline().decode().strip()
    if response == "OK":
        print("  Saved successfully!")
    else:
        print(f"  Save response: {response}")

    # Verify by reading back
    print()
    print("Verifying parameters:")
    ser.write(b'PARAMS\n')
    time.sleep(0.2)

    # Read header
    header = ser.readline().decode().strip()
    if header.startswith("PARAMS,"):
        # Read values
        values_line = ser.readline().decode().strip()
        headers = header.split(',')[1:]  # Skip "PARAMS"
        values = values_line.split(',')

        print(f"  {'Parameter':20s} {'Set':>10s} {'Actual':>10s}")
        print(f"  {'-'*42}")

        param_dict = dict(params)
        for h, v in zip(headers, values):
            if h in param_dict:
                set_val = param_dict[h]
                act_val = float(v)
                match = "OK" if abs(set_val - act_val) < 0.001 else "MISMATCH"
                print(f"  {h:20s} {set_val:10.4f} {act_val:10.4f}  [{match}]")

    # Get system status
    print()
    print("System status:")
    ser.write(b'STATUS\n')
    time.sleep(0.2)
    status_line = ser.readline().decode().strip()
    if status_line.startswith("STATUS,"):
        parts = status_line.split(',')
        if len(parts) >= 5:
            print(f"  State:    {'RUNNING' if parts[1] == '1' else 'STOPPED'}")
            print(f"  Faults:   {parts[2]}")
            print(f"  Enabled:  {'YES' if parts[3] == '1' else 'NO'}")
            print(f"  Voltage:  {parts[4]}V")
            print(f"  Loop:     {parts[5]}Hz" if len(parts) > 5 else "")

    ser.close()

    print()
    print("=== Initialization Complete ===")
    print()
    print("Next steps:")
    print("  1. Place robot upright and enable balance control")
    print("  2. Monitor via: python3 tools/auto_tune.py")
    print("  3. Or use web interface at http://192.168.4.1")


if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else None
    init_params(port)
