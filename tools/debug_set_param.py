#!/usr/bin/env python3
"""
实时设置参数并保存
用法: python3 debug_set_param.py <param_name> <value>
示例: python3 debug_set_param.py zeropoint_kp 0.002
"""
import serial
import time
import sys

PORT = '/dev/cu.usbmodem5B5F1233271'
BAUD = 115200

def main():
    if len(sys.argv) != 3:
        print("用法: python3 debug_set_param.py <param_name> <value>")
        print("示例:")
        print("  python3 debug_set_param.py zeropoint_kp 0.002")
        print("  python3 debug_set_param.py angle_kp 2.5")
        print("  python3 debug_set_param.py gyro_kp 0.35")
        sys.exit(1)

    param = sys.argv[1]
    value = sys.argv[2]

    # 验证参数名
    valid_params = [
        'angle_kp', 'gyro_kp', 'distance_kp', 'speed_kp',
        'yaw_angle_kp', 'yaw_gyro_kp',
        'lqr_u_kp', 'lqr_u_ki', 'zeropoint_kp',
        'lpf_target_vel_tf', 'lpf_zeropoint_tf',
        'max_tilt_deg', 'pitch_offset', 'pid_limit',
        'lift_accel_thresh', 'lift_vel_thresh'
    ]

    if param not in valid_params:
        print(f"❌ 未知参数: {param}")
        print(f"有效参数: {', '.join(valid_params)}")
        sys.exit(1)

    print(f"连接串口 {PORT}...")
    s = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.3)
    s.reset_input_buffer()

    # 发送设置命令
    cmd = f"{param}={value}\n"
    print(f"\n>>> 发送: {cmd.strip()}")
    s.write(cmd.encode())
    time.sleep(0.3)

    # 读取响应
    resp = s.read(300).decode('utf-8', errors='replace')
    if resp:
        print(f"响应: {resp.strip()}")

    # 保存到 Flash
    print("\n>>> 保存到 Flash...")
    s.write(b'save\n')
    time.sleep(0.5)

    resp = s.read(300).decode('utf-8', errors='replace')
    if 'successfully' in resp or 'saved' in resp.lower():
        print("✅ 参数已保存")
    else:
        print(f"响应: {resp.strip()}")

    # 验证
    print("\n>>> 验证设置...")
    s.write(f'{param}?\n'.encode())
    time.sleep(0.3)

    resp = s.read(300).decode('utf-8', errors='replace')
    if param in resp:
        print(f"✅ 验证成功: {resp.strip()}")
    else:
        print(f"响应: {resp.strip()}")

    s.close()
    print("\n完成")

if __name__ == '__main__':
    main()
