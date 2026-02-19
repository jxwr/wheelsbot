#!/usr/bin/env python3
"""
监控 CoG 自适应行为
观察 pitch_offset 是否缓慢变化（正常）或快速跳动（异常）
"""
import serial
import time
import re
import statistics

PORT = '/dev/cu.usbmodem5B5F1233271'
BAUD = 115200
MONITOR_TIME = 10  # 监控秒数

def main():
    print(f"连接串口 {PORT}...")
    s = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.3)
    s.reset_input_buffer()

    print(f"\n>>> 持续监控 {MONITOR_TIME} 秒，观察 pitch_offset 变化...")
    print("注意：保持小车静止，不要触碰\n")

    pitch_offsets = []
    timestamps = []

    start_time = time.time()
    sample_count = 0

    while time.time() - start_time < MONITOR_TIME:
        s.write(b'telem\n')
        time.sleep(0.5)

        data = s.read(800).decode('utf-8', errors='replace')

        # 解析 pitch_offset
        match = re.search(r'PITCH_OFFSET:([-\d.]+)deg', data)
        if match:
            po = float(match.group(1))
            pitch_offsets.append(po)
            timestamps.append(time.time() - start_time)
            sample_count += 1

            # 计算变化率
            if len(pitch_offsets) > 1:
                delta_po = po - pitch_offsets[0]
                delta_t = timestamps[-1] - timestamps[0]
                rate = delta_po / delta_t if delta_t > 0 else 0
                print(f"[{sample_count:2d}] t={timestamps[-1]:.1f}s  "
                      f"pitch_offset={po:+.4f}°  "
                      f"变化率={rate:+.3f}°/s")
            else:
                print(f"[{sample_count:2d}] t=0.0s   pitch_offset={po:+.4f}°")

    s.close()

    # 分析结果
    print("\n" + "="*50)
    print("📊 分析结果")
    print("="*50)

    if len(pitch_offsets) >= 3:
        total_change = pitch_offsets[-1] - pitch_offsets[0]
        avg_rate = total_change / MONITOR_TIME

        print(f"初始 pitch_offset: {pitch_offsets[0]:.4f}°")
        print(f"最终 pitch_offset: {pitch_offsets[-1]:.4f}°")
        print(f"总变化: {total_change:+.4f}°")
        print(f"平均变化率: {avg_rate:+.4f}°/s")

        # 判断是否正常
        if abs(avg_rate) < 0.5:
            print("\n✅ CoG 自适应正常（变化缓慢，<0.5°/s）")
            print("   修复已生效")
        elif abs(avg_rate) < 2.0:
            print("\n⚠️  CoG 自适应偏快（0.5-2°/s）")
            print("   可能仍有干扰，观察小车是否稳定")
        else:
            print("\n❌ CoG 自适应过快（>2°/s）")
            print("   修复未生效或存在其他问题！")
            print("   建议: 1) 确认代码已烧录")
            print("         2) 执行 zeropoint_kp=0.002 && save")
            print("         3) 重启小车")

        # 检查波动
        if len(pitch_offsets) > 5:
            # 计算标准差（去掉趋势）
            diffs = [pitch_offsets[i+1] - pitch_offsets[i]
                     for i in range(len(pitch_offsets)-1)]
            jump_count = sum(1 for d in diffs if abs(d) > 0.1)
            if jump_count > len(diffs) * 0.3:
                print(f"\n⚠️  检测到 {jump_count} 次跳动（>0.1°）")
                print("   可能有干扰或 adaptation 条件过于敏感")
    else:
        print("❌ 数据不足，无法分析")

if __name__ == '__main__':
    main()
