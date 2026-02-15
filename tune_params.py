#!/usr/bin/env python3
"""
自动调参脚本 - 通过串口逐步降低参数直到稳定
使用方法: python3 tune_params.py
"""

import serial
import time
import sys

SERIAL_PORT = "/dev/cu.usbmodem5B5F1233271"
BAUD_RATE = 115200

def send_command(ser, cmd, wait=0.5):
    """发送命令并读取响应"""
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    response = ""
    while ser.in_waiting:
        response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
    return response

def main():
    print(f"Connecting to {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # 等待连接稳定
    except Exception as e:
        print(f"Error: {e}")
        print("请检查:")
        print(f"1. ESP32 是否连接到 {SERIAL_PORT}")
        print("2. 端口号是否正确 (ls /dev/cu.*)")
        return

    print("Connected! 开始调试...\n")

    # 步骤1: 查看当前参数
    print("=== 步骤1: 查看当前参数 ===")
    response = send_command(ser, "D")
    print(response)
    input("\n按回车继续下一步 (降低 angle_kp 到 1.2)...")

    # 步骤2: 降低 angle_kp 到 1.2
    print("\n=== 步骤2: angle_kp = 1.2 ===")
    response = send_command(ser, "P angle_kp 1.2")
    print(response)
    print("\n>>> 请观察小车是否还抖 <<<")
    still_shaking = input("还抖吗? (y/n): ").lower().strip()

    if still_shaking == 'y':
        print("\n=== 步骤3: angle_kp = 1.0 ===")
        response = send_command(ser, "P angle_kp 1.0")
        print(response)
        print("\n>>> 请观察小车是否还抖 <<<")
        still_shaking = input("还抖吗? (y/n): ").lower().strip()

        if still_shaking == 'y':
            print("\n=== 步骤4: angle_kp = 0.8 ===")
            response = send_command(ser, "P angle_kp 0.8")
            print(response)
            print("\n>>> 请观察小车是否还抖 <<<")
            still_shaking = input("还抖吗? (y/n): ").lower().strip()

            if still_shaking == 'y':
                print("\n=== 步骤5: 增加阻尼 angle_kd = 0.8 ===")
                response = send_command(ser, "P angle_kd 0.8")
                print(response)

    # 步骤6: 查看最终参数
    print("\n=== 最终参数 ===")
    response = send_command(ser, "D")
    print(response)

    # 步骤7: 保存
    save = input("\n保存这些参数到Flash? (y/n): ").lower().strip()
    if save == 'y':
        response = send_command(ser, "S")
        print(response)
        print("✓ 参数已保存!")

    ser.close()
    print("\n调试完成!")

if __name__ == "__main__":
    main()
