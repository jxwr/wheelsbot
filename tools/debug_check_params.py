#!/usr/bin/env python3
"""
验证 CoG 自适应修复是否生效
检查 zeropoint_kp 是否为 0.002（修复后的值）
"""
import serial
import time
import re

PORT = '/dev/cu.usbmodem5B5F1233271'
BAUD = 115200

def main():
    print(f"连接串口 {PORT}...")
    s = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.3)

    # 清空缓冲区
    s.reset_input_buffer()

    # 发送 dump 命令
    print("\n>>> 读取当前参数...")
    s.write(b'dump\n')
    time.sleep(0.5)

    data = s.read(1500).decode('utf-8', errors='replace')
    print(data)

    # 解析关键参数
    print("\n=== 参数检查结果 ===")

    # 检查 zeropoint_kp
    match = re.search(r'zeropoint_kp=([\d.]+)', data)
    if match:
        zp = float(match.group(1))
        if abs(zp - 0.002) < 0.0001:
            print(f"✅ zeropoint_kp = {zp:.6f}  (正确)")
        else:
            print(f"❌ zeropoint_kp = {zp:.6f}  (错误! 应为 0.002)")
            print("   修复未生效或旧参数被保存到 flash")
            print("   建议执行: zeropoint_kp=0.002 && save")
    else:
        print("⚠️  无法解析 zeropoint_kp")

    # 检查其他关键参数
    params = {
        'angle_kp': (2.5, 0.1),
        'gyro_kp': (0.40, 0.01),
        'distance_kp': (0.55, 0.01),
        'speed_kp': (0.45, 0.01),
    }

    print("\n--- 其他参数 ---")
    for name, (expected, tol) in params.items():
        match = re.search(rf'{name}=([\d.]+)', data)
        if match:
            val = float(match.group(1))
            status = "✅" if abs(val - expected) < tol else "⚠️"
            print(f"{status} {name} = {val:.4f} (期望: {expected})")

    s.close()
    print("\n检查完成")

if __name__ == '__main__':
    main()
