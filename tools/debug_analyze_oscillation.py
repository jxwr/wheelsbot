#!/usr/bin/env python3
"""
诊断晃动原因
分析各控制环贡献，判断是哪个环节导致振荡
"""
import serial
import time
import re
import statistics

PORT = '/dev/cu.usbmodem5B5F1233271'
BAUD = 115200
SAMPLES = 20  # 采样次数

def main():
    print(f"连接串口 {PORT}...")
    s = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.3)
    s.reset_input_buffer()

    print(f"\n>>> 采集 {SAMPLES} 组数据，分析振荡来源...")
    print("保持小车竖立，观察自然晃动\n")

    # 存储数据序列
    data_seq = {
        'angle': [],
        'gyro': [],
        'dist': [],
        'speed': [],
        'lqr_raw': [],
        'pitch': [],
    }

    for i in range(SAMPLES):
        s.write(b'telem\n')
        time.sleep(0.5)

        resp = s.read(800).decode('utf-8', errors='replace')

        # 解析各字段
        patterns = {
            'angle': r'ANGLE:([-\d.]+)',
            'gyro': r'GYRO:([-\d.]+)',
            'dist': r'DIST:([-\d.]+)',
            'speed': r'SPD:([-\d.]+)',
            'lqr_raw': r'LQR_RAW:([-\d.]+)',
            'pitch': r'PITCH:([-\d.]+)deg',
        }

        for key, pattern in patterns.items():
            match = re.search(pattern, resp)
            if match:
                data_seq[key].append(float(match.group(1)))

        # 显示当前帧
        if data_seq['angle']:
            print(f"[{i+1:2d}] "
                  f"A={data_seq['angle'][-1]:+.2f} "
                  f"G={data_seq['gyro'][-1]:+.2f} "
                  f"D={data_seq['dist'][-1]:+.2f} "
                  f"S={data_seq['speed'][-1]:+.2f}")

    s.close()

    # 分析振荡
    print("\n" + "="*60)
    print("📊 振荡分析")
    print("="*60)

    def analyze_signal(name, values, threshold_ratio=0.3):
        """分析信号是否振荡"""
        if len(values) < 5:
            return None

        # 计算变化
        diffs = [values[i+1] - values[i] for i in range(len(values)-1)]
        sign_changes = sum(1 for i in range(len(diffs)-1)
                          if diffs[i] * diffs[i+1] < 0)

        # 变化幅度
        max_val = max(values)
        min_val = min(values)
        range_val = max_val - min_val
        std_val = statistics.stdev(values) if len(values) > 1 else 0

        # 振荡指标：符号变化次数 / 总变化次数
        oscillation_ratio = sign_changes / len(diffs) if diffs else 0

        # 判断
        is_oscillating = oscillation_ratio > threshold_ratio and range_val > 0.5

        return {
            'name': name,
            'range': range_val,
            'std': std_val,
            'osc_ratio': oscillation_ratio,
            'is_oscillating': is_oscillating,
            'max': max_val,
            'min': min_val,
        }

    # 分析各信号
    results = []
    for key, values in data_seq.items():
        if values:
            result = analyze_signal(key.upper(), values)
            if result:
                results.append(result)

    # 按振荡程度排序
    results.sort(key=lambda x: x['osc_ratio'], reverse=True)

    print("\n各控制环振荡程度（按振荡比排序）:")
    print("-"*60)
    print(f"{'环节':<10} {'范围':<10} {'标准差':<10} {'振荡比':<10} {'状态'}")
    print("-"*60)

    for r in results:
        status = "🔴 振荡" if r['is_oscillating'] else "🟢 稳定"
        print(f"{r['name']:<10} {r['range']:<10.2f} {r['std']:<10.2f} "
              f"{r['osc_ratio']:<10.2f} {status}")

    # 诊断建议
    print("\n" + "="*60)
    print("🔍 诊断建议")
    print("="*60)

    # 找出主要振荡源
    oscillating = [r for r in results if r['is_oscillating']]

    if not oscillating:
        print("✅ 未检测到明显振荡")
        print("   如果小车仍有肉眼可见的晃动，可能是机械问题或采样频率不够")
    else:
        primary = oscillating[0]
        print(f"主要振荡源: {primary['name']} (振荡比: {primary['osc_ratio']:.2f})")

        if primary['name'] == 'ANGLE':
            print("\n📋 建议:")
            print("   1. 降低 angle_kp (当前可能过高)")
            print("   2. 增加 gyro_kp 增强阻尼")
            print(f"   3. 当前 angle 范围: {primary['range']:.2f}V")

        elif primary['name'] == 'GYRO':
            print("\n📋 建议:")
            print("   gyro 环振荡通常意味着噪声过大")
            print("   1. 检查 IMU 数据噪声")
            print("   2. 增加低通滤波时间常数")

        elif primary['name'] == 'DIST':
            print("\n📋 建议:")
            print("   位置环振荡!")
            print("   1. 降低 distance_kp")
            print("   2. 检查 zeropoint_kp 是否仍过高")
            print("   3. 确认 CoG 自适应使用的是 distance_ctrl 而非 position_error")

        elif primary['name'] == 'SPEED':
            print("\n📋 建议:")
            print("   速度环振荡!")
            print("   1. 降低 speed_kp")
            print("   2. 检查速度信号是否有噪声")

        elif primary['name'] == 'LQR_RAW':
            print("\n📋 建议:")
            print("   总输出振荡，可能是多个环节叠加")
            print("   逐一降低各环增益进行测试")

    # 显示 pitch 稳定性
    if data_seq['pitch']:
        pitch_range = max(data_seq['pitch']) - min(data_seq['pitch'])
        print(f"\n📐 实际倾角变化范围: {pitch_range:.2f}°")
        if pitch_range > 5:
            print("   ⚠️  倾角波动过大，建议检查机械平衡")
        elif pitch_range < 1:
            print("   ✅ 倾角很稳定")

if __name__ == '__main__':
    main()
