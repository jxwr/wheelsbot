# 调试工具集

用于诊断平衡车问题的 Python 脚本。

## 前置要求

```bash
pip install pyserial
```

## 使用前检查

确保串口路径正确（根据你的系统修改脚本中的 PORT）：

```bash
# macOS
PORT = '/dev/cu.usbmodem5B5F1233271'

# Linux
PORT = '/dev/ttyUSB0'  # 或 /dev/ttyACM0
```

## 工具列表

### 1. debug_check_params.py - 参数验证

检查当前参数是否正确，特别是修复后的 `zeropoint_kp`。

```bash
python3 debug_check_params.py
```

**输出示例：**
```
✅ zeropoint_kp = 0.002000  (正确)
✅ angle_kp = 2.50 (期望: 2.5)
✅ gyro_kp = 0.40 (期望: 0.4)
```

如果 `zeropoint_kp` 不是 0.002，需要修复：
```bash
python3 debug_set_param.py zeropoint_kp 0.002
```

---

### 2. debug_monitor_cog.py - CoG 自适应监控

观察 `pitch_offset` 的变化速度，验证修复是否生效。

```bash
python3 debug_monitor_cog.py
```

**正常表现：**
- `pitch_offset` 缓慢变化（<0.5°/秒）
- 显示 "✅ CoG 自适应正常"

**异常表现：**
- `pitch_offset` 快速跳动（>2°/秒）
- 显示 "❌ CoG 自适应过快"
- 说明修复未生效

---

### 3. debug_analyze_oscillation.py - 振荡诊断

分析哪个控制环导致晃动。

```bash
python3 debug_analyze_oscillation.py
```

**输出：**
```
各控制环振荡程度（按振荡比排序）:
环节       范围        标准差      振荡比      状态
------------------------------------------------------------
DIST       3.45       0.82       0.65       🔴 振荡
ANGLE      1.23       0.35       0.25       🟢 稳定
...

🔍 诊断建议
主要振荡源: DIST
📋 建议:
   1. 降低 distance_kp
   2. 检查 zeropoint_kp 是否仍过高
```

---

### 4. debug_set_param.py - 参数设置

实时修改参数并保存。

```bash
# 设置单个参数
python3 debug_set_param.py zeropoint_kp 0.002
python3 debug_set_param.py angle_kp 2.5
python3 debug_set_param.py gyro_kp 0.35

# 查看可用参数
python3 debug_set_param.py
```

---

## 标准调试流程

### Step 1: 验证修复
```bash
python3 debug_check_params.py
```
确保 `zeropoint_kp = 0.002`，如果不是则修正。

### Step 2: 观察自适应行为
```bash
python3 debug_monitor_cog.py
```
确认 `pitch_offset` 变化缓慢（<0.5°/秒）。

### Step 3: 诊断晃动原因（如有）
```bash
python3 debug_analyze_oscillation.py
```
根据建议调整相应参数。

### Step 4: 参数微调
```bash
# 例如诊断建议降低 distance_kp
python3 debug_set_param.py distance_kp 0.45
```

---

## 常见问题

### Q: 串口连接失败
检查串口路径：
```bash
# macOS
ls /dev/cu.usbmodem*

# Linux
ls /dev/ttyUSB* /dev/ttyACM*
```

### Q: 参数修改后重启恢复
确保执行了 `save` 命令保存到 Flash。

### Q: 遥测数据偶尔丢失
正常现象，脚本会自动处理。如果持续失败，检查：
1. 波特率是否正确（115200）
2. 小车是否正常启动
3. 串口是否被其他程序占用
