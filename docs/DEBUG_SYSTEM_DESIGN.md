# 平衡车调试系统技术方案

## 1. 概述

设计一个完整的调试系统，包含嵌入式端和PC端两部分，用于实时监控传感器数据、调整参数，并闭环验证直到小车平衡。

---

## 2. 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32-S3 (嵌入式端)                    │
│  ┌──────────────────────────────────────────────────┐   │
│  │  Serial Debug Module (serial_debug.cpp/h)        │   │
│  │  ┌────────────────────────────────────────────┐  │   │
│  │  │  1. Sensor Data Streaming (可配置频率)      │  │   │
│  │  │  2. Real-time Command Parser                │  │   │
│  │  │  3. Parameter Get/Set                       │  │   │
│  │  │  4. System State Monitor                    │  │   │
│  │  └────────────────────────────────────────────┘  │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
                          ↕ (USB Serial, 115200 baud)
┌─────────────────────────────────────────────────────────┐
│              Golang CLI Tool (bot-debug)                 │
│  ┌──────────────────────────────────────────────────┐   │
│  │  1. Serial Communication Layer                   │   │
│  │  2. TUI (Terminal UI) with Bubble Tea            │   │
│  │  3. Real-time Data Visualizer                    │   │
│  │  4. Parameter Tuning Interface                   │   │
│  │  5. CSV Data Logger                              │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

---

## 3. 嵌入式端：Serial Debug Module

### 3.1 文件结构

```
src/serial_debug.h
src/serial_debug.cpp
```

### 3.2 核心功能

#### A. 数据流式输出（可配置）

**输出格式：CSV（便于解析）**

```
DATA,timestamp,pitch,pitch_rate,yaw_rate,wheel_vel_l,wheel_vel_r,wheel_pos_l,wheel_pos_r,motor_l,motor_r,voltage,state,faults
```

**字段说明：**
- `timestamp` - 毫秒级时间戳
- `pitch` - 倾角 (rad)
- `pitch_rate` - 角速度 (rad/s)
- `yaw_rate` - 偏航角速度 (rad/s)
- `wheel_vel_l/r` - 左右轮速度 (rad/s)
- `wheel_pos_l/r` - 左右轮位置 (rad)
- `motor_l/r` - 左右电机输出 (V)
- `voltage` - 电池电压 (V)
- `state` - 控制器状态 (0=DISABLED, 1=RUNNING, 2=FAULT)
- `faults` - 故障标志位

**频率控制：**
```cpp
// 支持命令动态调整频率
STREAM <freq>  // freq = 0 停止, 1-200 Hz
```

#### B. 命令接口（扩展现有命令）

**1. 查询类命令：**
```
STATUS      - 显示系统状态（一次性）
SENSORS     - 显示所有传感器读数
PARAMS      - 显示所有参数（增强版）
STATS       - 显示统计信息（频率、饱和、运行时间）
```

**2. 控制类命令：**
```
ENABLE      - 启用平衡控制
DISABLE     - 禁用平衡控制
RESET       - 重置控制器
STREAM <hz> - 设置数据流频率 (0=关闭)
```

**3. 参数设置（增强）：**
```
GET <param>         - 获取单个参数值
SET <param> <value> - 设置参数（立即生效）
SAVE                - 保存到flash
LOAD                - 从flash加载
DEFAULT             - 恢复默认参数
```

**4. 调试命令：**
```
CALIBRATE           - IMU校准
MOTOR_TEST <v>      - 电机测试 (-6 to 6V)
ZERO_ENCODER        - 编码器清零
```

#### C. 输出格式标准化

所有输出使用统一前缀便于解析：

```
DATA,    - 数据流
RESP,    - 命令响应
INFO,    - 信息
WARN,    - 警告
ERROR,   - 错误
DEBUG,   - 调试信息
```

### 3.3 实现细节

**串口任务优先级：**
- 独立的 `serialDebugTask` (优先级 2，Core 0)
- 不阻塞控制循环

**缓冲区设计：**
```cpp
// 双缓冲，防止数据丢失
static char tx_buffer[2][512];
static uint8_t tx_buffer_idx = 0;
```

**电压监测：**
```cpp
// 通过ADC读取电池电压
float readBatteryVoltage() {
  // ESP32-S3 ADC1_CH0 (GPIO1)
  // 分压电阻: 47k + 10k
  return analogRead(BATTERY_ADC_PIN) * (3.3f / 4095.0f) * 5.7f;
}
```

---

## 4. Golang CLI Tool: bot-debug

### 4.1 技术栈

- **Language:** Go 1.21+
- **Serial:** `go.bug.st/serial`
- **TUI:** `github.com/charmbracelet/bubbletea`
- **Charts:** `github.com/guptarohit/asciigraph`
- **CLI:** `github.com/spf13/cobra`

### 4.2 项目结构

```
tools/bot-debug/
├── main.go
├── go.mod
├── go.sum
├── cmd/
│   ├── monitor.go      # 监控模式
│   ├── tune.go         # 调参模式
│   ├── record.go       # 数据记录
│   └── test.go         # 测试模式
├── pkg/
│   ├── serial/
│   │   ├── conn.go     # 串口连接
│   │   └── parser.go   # 数据解析
│   ├── tui/
│   │   ├── monitor.go  # 监控界面
│   │   ├── tuner.go    # 调参界面
│   │   └── charts.go   # 图表组件
│   └── model/
│       └── types.go    # 数据结构
└── README.md
```

### 4.3 核心功能

#### A. 监控模式 (monitor)

**TUI布局（使用Bubble Tea）：**

```
┌─────────────────────────────────────────────────────────────────┐
│ 🤖 Balance Bot Monitor                      Battery: 12.4V      │
├─────────────────────────────────────────────────────────────────┤
│ IMU Sensors                │ Wheels                             │
│  Pitch:    0.05 rad  (2.9°)│  Left:   12.3 rad/s  (123.4 RPM)  │
│  Pitch Rate: -0.12 rad/s   │  Right:  12.1 rad/s  (121.8 RPM)  │
│  Yaw Rate:   0.00 rad/s    │  Pos L:   5.2 rad                 │
│                            │  Pos R:   5.1 rad                 │
├─────────────────────────────────────────────────────────────────┤
│ Control Output             │ Status                             │
│  Motor L:  2.34 V          │  State: RUNNING ✓                 │
│  Motor R:  2.31 V          │  Faults: NONE                     │
│  Pitch Cmd: 0.03 rad       │  Loop: 199.8 Hz                   │
├─────────────────────────────────────────────────────────────────┤
│ Pitch History (5s)                                              │
│   0.1│     ╭╮                                                   │
│   0.0│─────┤╰──╮╭─╮───────────────────────────────────────────│
│  -0.1│         ╰╯ ╰                                             │
│      └─────────────────────────────────────────────────────────│
├─────────────────────────────────────────────────────────────────┤
│ [SPACE] Pause  [E] Enable  [D] Disable  [R] Reset  [Q] Quit    │
└─────────────────────────────────────────────────────────────────┘
```

**功能：**
- 实时数据显示
- 历史曲线（pitch, wheel_vel）
- 快捷键控制

#### B. 调参模式 (tune)

**交互式调参界面：**

```
┌─────────────────────────────────────────────────────────────────┐
│ 🔧 Parameter Tuning                                             │
├─────────────────────────────────────────────────────────────────┤
│ Velocity Loop (Outer, 50Hz)                                     │
│  > velocity_kp:        0.0800  [↑↓ adjust] [Enter to apply]    │
│    velocity_ki:        0.0200                                   │
│    velocity_kd:        0.0000                                   │
│    velocity_max_tilt:  0.1400                                   │
├─────────────────────────────────────────────────────────────────┤
│ Angle Loop (Inner, 200Hz)                                       │
│    angle_kp:           1.5000                                   │
│    angle_gyro_kd:      0.6000                                   │
│    angle_d_alpha:      0.7000                                   │
│    angle_max_out:      6.0000                                   │
├─────────────────────────────────────────────────────────────────┤
│ Real-time Feedback:                                             │
│  Pitch: 0.05 rad  │  Oscillation: LOW  │  Response: MODERATE   │
├─────────────────────────────────────────────────────────────────┤
│ [↑↓] Select  [←→] Adjust  [Enter] Apply  [S] Save  [Q] Quit    │
└─────────────────────────────────────────────────────────────────┘
```

**调参辅助：**
- 自动震荡检测（基于pitch波动）
- 响应速度评估
- 参数合理性提示

#### C. 数据记录模式 (record)

**功能：**
```bash
# 记录10秒数据到CSV
bot-debug record -d 10 -o balance_test_001.csv

# 自动生成报告
bot-debug record -d 30 --report
```

**CSV格式：**
```csv
timestamp,pitch,pitch_rate,yaw_rate,wheel_vel_l,wheel_vel_r,motor_l,motor_r,voltage,state
0.000,0.05,-0.12,0.00,12.3,12.1,2.34,2.31,12.4,1
0.005,0.04,-0.10,0.01,12.4,12.2,2.35,2.32,12.4,1
...
```

**报告生成：**
- 统计摘要（均值、方差、最大值）
- 稳定性评分
- 震荡频率分析

#### D. 测试模式 (test)

**自动化测试序列：**

```bash
# 静态平衡测试
bot-debug test balance --duration 30

# 阶跃响应测试
bot-debug test step --amplitude 0.1

# 参数扫描
bot-debug test sweep --param angle_kp --range 1.0-2.0 --step 0.1
```

### 4.4 命令行接口

```bash
# 安装
go install ./tools/bot-debug

# 基本用法
bot-debug monitor --port /dev/cu.usbmodem*  # 监控
bot-debug tune --port /dev/cu.usbmodem*     # 调参
bot-debug record -d 10 -o test.csv          # 记录
bot-debug test balance                      # 测试

# 高级选项
bot-debug monitor --stream-rate 100         # 100Hz数据流
bot-debug tune --preset conservative        # 加载预设参数
```

---

## 5. 闭环调试流程

### Phase 1: 传感器验证 (5分钟)

```bash
bot-debug monitor --sensors-only
```

**检查项：**
- [ ] IMU读数稳定（pitch ±0.01 rad静止时）
- [ ] 编码器读数正确（手转轮子观察）
- [ ] 电压监测正常（>11V）
- [ ] 无传感器超时

### Phase 2: 开环电机测试 (5分钟)

```bash
bot-debug test motor --voltage 1.0
```

**检查项：**
- [ ] 左右电机方向一致
- [ ] 电机响应平滑
- [ ] 无异常噪音

### Phase 3: 静态平衡（核心调试）

**步骤1：超保守参数启动**
```bash
bot-debug tune --preset ultra-conservative
# velocity_kp=0.04, angle_kp=1.0, angle_gyro_kd=1.0
```

**步骤2：手扶测试**
- 启用平衡
- 观察是否震荡
- 如果震荡 → 降低angle_kp或增大angle_gyro_kd

**步骤3：逐步优化**
```bash
# 使用调参界面实时调整
bot-debug tune
```

**优化顺序：**
1. 调angle_gyro_kd直到不震荡
2. 调angle_kp提高响应速度
3. 调velocity_kp改善速度跟踪
4. 微调angle_d_alpha优化噪声

**步骤4：性能验证**
```bash
# 记录30秒数据并分析
bot-debug record -d 30 --analyze
```

**合格标准：**
- Pitch std < 0.05 rad (静止)
- 无持续震荡
- 能自主站立5秒以上

### Phase 4: 动态响应测试

**阶跃测试：**
```bash
bot-debug test step --type velocity --amplitude 0.2
```

**检查项：**
- [ ] 超调 < 20%
- [ ] 稳定时间 < 2s
- [ ] 无持续震荡

---

## 6. 安全机制

### 嵌入式端

```cpp
// 调试模式下自动安全限制
#define DEBUG_MAX_VOLTAGE 4.0f   // 降低电压限制
#define DEBUG_MAX_TILT 0.3f      // 降低倾角限制
```

### CLI端

```go
// 参数合理性检查
func ValidateParam(name string, value float64) error {
    limits := map[string][2]float64{
        "angle_kp":      {0.5, 5.0},
        "angle_gyro_kd": {0.1, 3.0},
        "velocity_kp":   {0.01, 0.5},
    }
    // ...
}
```

---

## 7. 实施计划

### Week 1: 基础设施

- [ ] Day 1-2: serial_debug.cpp/h（嵌入式端）
- [ ] Day 3-4: bot-debug基础框架（Golang）
- [ ] Day 5: 集成测试

### Week 2: 高级功能

- [ ] Day 1-2: TUI监控界面
- [ ] Day 3-4: 调参界面
- [ ] Day 5: 自动化测试

### Week 3: 实车调试

- [ ] Day 1: 传感器验证
- [ ] Day 2-3: 静态平衡调试
- [ ] Day 4: 动态响应优化
- [ ] Day 5: 性能测试与文档

---

## 8. 预期效果

### 调试效率提升

| 项目 | 手动调试 | 自动化调试 | 提升 |
|------|----------|-----------|------|
| 传感器检查 | 15分钟 | 2分钟 | 7.5x |
| 参数调整 | 30分钟 | 5分钟 | 6x |
| 性能验证 | 20分钟 | 3分钟 | 6.7x |
| **总计** | **65分钟** | **10分钟** | **6.5x** |

### 数据驱动优化

- 自动记录所有调试数据
- 量化评估参数改进
- 可重现的测试流程

---

## 9. 技术细节

### 串口协议设计

**命令格式：**
```
<CMD>[SPACE<ARG1>[SPACE<ARG2>...]]\n
```

**响应格式：**
```
RESP,<status>,<message>\n
```

**数据流格式：**
```
DATA,<timestamp>,<field1>,<field2>,...\n
```

### 性能优化

**嵌入式端：**
- 使用DMA串口发送（不阻塞）
- 数据流频率自适应（避免丢包）
- 命令队列（FIFO，最多8个）

**Golang端：**
- 并发读写（goroutine）
- 数据缓冲（channel，容量1000）
- 定时刷新TUI（60fps）

---

## 10. 可选扩展

### 远程调试（后期）

```
ESP32 WiFi → WebSocket → bot-debug
```

### 机器学习集成

```bash
# 导出数据用于强化学习
bot-debug record --format rl-dataset
```

### 参数自动优化

```bash
# 遗传算法自动调参
bot-debug auto-tune --iterations 100
```

---

**方案完成！请确认是否开始实施。**
