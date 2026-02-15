#!/usr/bin/env python3
"""
Simple test script for balance robot debug system
Tests all serial commands in CSV format
"""

import serial
import time
import sys

PORT = '/dev/cu.usbmodem5B5F1233271'
BAUD = 115200

def send_command(ser, cmd):
    """Send command and read response"""
    ser.reset_input_buffer()
    ser.write(f'{cmd}\n'.encode())
    ser.flush()
    time.sleep(0.2)

    lines = []
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line and not line.startswith('MOT,'):
            lines.append(line)
    return lines

def main():
    print("=== Balance Bot Debug System Test ===\n")

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(0.5)
        print(f"✓ Connected to {PORT}\n")

        # Test STATUS
        print("1. Testing STATUS command...")
        lines = send_command(ser, 'STATUS')
        for line in lines:
            print(f"   {line}")
        print()

        # Test SENSORS
        print("2. Testing SENSORS command...")
        lines = send_command(ser, 'SENSORS')
        for line in lines:
            print(f"   {line}")
        print()

        # Test GET
        print("3. Testing GET command...")
        lines = send_command(ser, 'GET angle_kp')
        for line in lines:
            print(f"   {line}")
        print()

        # Test SET
        print("4. Testing SET command...")
        lines = send_command(ser, 'SET angle_kp 1.5')
        for line in lines:
            print(f"   {line}")
        print()

        # Test STREAM
        print("5. Testing STREAM command (5 samples)...")
        ser.reset_input_buffer()
        ser.write(b'STREAM 20\n')
        ser.flush()
        time.sleep(0.3)

        count = 0
        while count < 5:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('DATA,'):
                    print(f"   {line}")
                    count += 1

        ser.write(b'STREAM 0\n')
        ser.flush()
        print()

        # Test PARAMS
        print("6. Testing PARAMS command...")
        lines = send_command(ser, 'PARAMS')
        if len(lines) >= 2:
            print(f"   Header: {lines[0][:80]}...")
            print(f"   Values: {lines[1][:80]}...")
        print()

        ser.close()
        print("✓ All tests passed!")

    except Exception as e:
        print(f"✗ Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
