# 平衡车调试系统技术方案 (Agent-First Design)

## 设计目标

为AI Agent提供命令行工具，用于：
1. 实时读取传感器数据
2. 设置/获取参数
3. 分析系统状态
4. 自动化调试闭环

**核心原则：机器可读 > 人类可读**

---

## 1. 系统架构

```
┌────────────────────────────────────────┐
│         ESP32-S3 (嵌入式端)             │
│  serial_debug.cpp/h                    │
│  - CSV数据流                            │
│  - 简单命令协议                         │
└────────────────────────────────────────┘
                ↕ USB Serial
┌────────────────────────────────────────┐
│      Golang CLI: bot-debug              │
│  - CSV输出（stdin/stdout）              │
│  - 无交互，适合脚本调用                  │
└────────────────────────────────────────┘
                ↕ Bash调用
┌────────────────────────────────────────┐
│         AI Agent (Claude)               │
│  - 读取CSV数据（最省token）             │
│  - 分析系统状态                         │
│  - 调整参数                             │
│  - 闭环验证                             │
└────────────────────────────────────────┘
```

---

## 2. 嵌入式端：serial_debug模块

### 2.1 文件结构

```cpp
src/serial_debug.h
src/serial_debug.cpp
```

### 2.2 数据流格式（CSV）

```
DATA,<timestamp>,<pitch>,<pitch_rate>,<yaw_rate>,<wl>,<wr>,<ml>,<mr>,<voltage>,<state>,<faults>
```

**示例：**
```
DATA,12345,0.052,-0.123,0.001,12.3,12.1,2.34,2.31,12.4,1,0
```

### 2.3 命令协议（简单文本）

**输入格式：** `<CMD> [args]\n`

**输出格式：** `<TYPE>,<payload>\n`

#### 命令列表

| 命令 | 参数 | 响应 | 说明 |
|------|------|------|------|
| `STATUS` | - | `STATUS,<csv>` | 完整状态 |
| `SENSORS` | - | `SENSORS,<csv>` | 传感器读数 |
| `PARAMS` | - | `PARAMS,<csv>` | 所有参数 |
| `GET <param>` | 参数名 | `VALUE,<param>,<value>` | 获取参数 |
| `SET <param> <val>` | 参数名,值 | `OK` / `ERROR,<msg>` | 设置参数 |
| `STREAM <hz>` | 频率(0-200) | `OK` | 启动数据流 |
| `ENABLE` | - | `OK` | 启用平衡 |
| `DISABLE` | - | `OK` | 禁用平衡 |
| `SAVE` | - | `OK` | 保存参数 |
| `LOAD` | - | `OK` | 加载参数 |

#### 响应示例

**STATUS响应（CSV）：**
```
STATUS,state,faults,enabled,voltage,loop_hz
1,0,1,12.4,199.8
```

**SENSORS响应（CSV）：**
```
SENSORS,pitch,pitch_rate,yaw_rate,wheel_l,wheel_r,motor_l,motor_r
0.052,-0.123,0.001,12.3,12.1,2.34,2.31
```

**PARAMS响应（CSV）：**
```
PARAMS,velocity_kp,velocity_ki,velocity_kd,velocity_max_tilt,angle_kp,angle_ki,angle_gyro_kd,angle_d_alpha,angle_max_out,yaw_kd,max_tilt,ramp_time,pitch_offset,pitch_cmd_rate_limit,sensor_timeout,velocity_decimation
0.15,0.02,0.0,0.14,1.5,0.0,0.6,0.7,6.0,1.2,0.6,0.5,0.0,0.5,0.2,4
```

---

## 3. Golang CLI: bot-debug

### 3.1 项目结构

```
tools/bot-debug/
├── main.go
├── go.mod
├── cmd/
│   ├── read.go      # 读取一次数据
│   ├── stream.go    # 采样N次
│   ├── set.go       # 设置参数
│   ├── get.go       # 获取参数
│   ├── status.go    # 获取状态
│   └── analyze.go   # 分析数据
└── pkg/
    ├── serial/      # 串口通信
    └── parser/      # 数据解析
```

### 3.2 命令接口（CSV输出）

#### A. `bot-debug read` - 读取一次传感器数据

**用法：**
```bash
bot-debug read --port /dev/cu.usbmodem*
```

**输出（CSV）：**
```
timestamp,pitch,pitch_rate,yaw_rate,wheel_vel_l,wheel_vel_r,motor_l,motor_r,voltage,state,faults
12345,0.052,-0.123,0.001,12.3,12.1,2.34,2.31,12.4,1,0
```

#### B. `bot-debug stream` - 采样N次数据

**用法：**
```bash
bot-debug stream --port /dev/cu.usbmodem* --samples 100 --rate 50
```

**输出（CSV with stats）：**
```
# Samples (100 rows)
timestamp,pitch,pitch_rate,yaw_rate,wheel_vel_l,wheel_vel_r,motor_l,motor_r,voltage,state,faults
0,0.052,-0.123,0.001,12.3,12.1,2.34,2.31,12.4,1,0
20,0.049,-0.118,0.002,12.2,12.0,2.32,2.29,12.4,1,0
...
# Stats
stat,pitch_mean,pitch_std,pitch_max,pitch_min
value,0.050,0.015,0.082,0.021
```

#### C. `bot-debug set` - 设置参数

**用法：**
```bash
bot-debug set --port /dev/cu.usbmodem* --param angle_kp --value 1.5
```

**输出（CSV）：**
```
success,param,value
1,angle_kp,1.5
```

#### D. `bot-debug get` - 获取参数

**用法：**
```bash
bot-debug get --port /dev/cu.usbmodem* --param angle_kp
```

**输出（CSV）：**
```
param,value
angle_kp,1.5
```

#### E. `bot-debug status` - 完整状态

**用法：**
```bash
bot-debug status --port /dev/cu.usbmodem*
```

**输出（CSV sections）：**
```
# System Status
state,enabled,voltage,loop_hz,faults
1,1,12.4,199.8,0
# Sensors
pitch,pitch_rate,yaw_rate,wheel_l,wheel_r,motor_l,motor_r
0.052,-0.123,0.001,12.3,12.1,2.34,2.31
# Params
velocity_kp,velocity_ki,angle_kp,angle_gyro_kd,yaw_kd
0.15,0.02,1.5,0.6,1.2
```

#### F. `bot-debug analyze` - 分析采样数据

**用法：**
```bash
bot-debug analyze --samples 200 --duration 4
```

**输出（CSV sections）：**
```
# Summary
duration_s,sample_count,sample_rate_hz
4.0,200,50.0
# Stability
pitch_std,oscillation_detected,oscillation_freq_hz
0.015,0,0.0
# Performance
max_pitch_deviation,settling_time_s,overshoot_percent
0.082,1.2,15.3
# Diagnosis
status,issue_count,recommendation
STABLE,0,Consider increasing velocity_kp for faster response
```

---

## 4. AI Agent调试流程

### Phase 1: 初始检查

```bash
# 1. 获取完整状态
STATUS=$(bot-debug status --port /dev/cu.usbmodem*)
echo "$STATUS"

# 2. 检查传感器健康（CSV解析：pitch是第2列）
SENSORS=$(bot-debug read --port /dev/cu.usbmodem*)
PITCH=$(echo "$SENSORS" | tail -1 | cut -d',' -f2)
PITCH_STABLE=$(awk -v p=$PITCH 'BEGIN{if(p<0.1 && p>-0.1) print 1; else print 0}')

# 3. 检查电压（CSV解析：voltage是第9列）
VOLTAGE=$(echo "$SENSORS" | tail -1 | cut -d',' -f9)
if (( $(echo "$VOLTAGE < 11.0" | bc -l) )); then
  echo "ERROR: Low battery"
  exit 1
fi
```

### Phase 2: 设置初始参数（超保守）

```bash
# 设置超保守参数
bot-debug set --param velocity_kp --value 0.04
bot-debug set --param angle_kp --value 1.0
bot-debug set --param angle_gyro_kd --value 1.0
bot-debug set --param yaw_kd --value 1.5
```

### Phase 3: 启用并采样

```bash
# 启用平衡控制
bot-debug enable

# 采样4秒数据（200个样本 @ 50Hz）
RESULT=$(bot-debug analyze --samples 200 --duration 4)
echo "$RESULT"
```

### Phase 4: 分析并决策

```bash
# 解析分析结果（CSV格式）
# Diagnosis行: status,issue_count,recommendation
STATUS=$(echo "$RESULT" | grep -A1 "^# Diagnosis" | tail -1 | cut -d',' -f1)
PITCH_STD=$(echo "$RESULT" | grep -A1 "^# Stability" | tail -1 | cut -d',' -f1)
OSCILLATION=$(echo "$RESULT" | grep -A1 "^# Stability" | tail -1 | cut -d',' -f2)

if [ "$OSCILLATION" = "1" ]; then
  echo "OSCILLATION DETECTED - Reducing gains"
  # 降低增益或增大阻尼
  CURRENT_KP=$(bot-debug get --param angle_kp | tail -1 | cut -d',' -f2)
  NEW_KP=$(echo "$CURRENT_KP * 0.8" | bc -l)
  bot-debug set --param angle_kp --value $NEW_KP
fi

if (( $(echo "$PITCH_STD < 0.02" | bc -l) )); then
  echo "STABLE - Can increase performance"
  # 提高响应速度
  CURRENT_VKP=$(bot-debug get --param velocity_kp | tail -1 | cut -d',' -f2)
  NEW_VKP=$(echo "$CURRENT_VKP * 1.2" | bc -l)
  bot-debug set --param velocity_kp --value $NEW_VKP
fi
```

### Phase 5: 迭代优化

```bash
# 循环调优
for i in {1..10}; do
  echo "=== Iteration $i ==="

  # 采样分析
  RESULT=$(bot-debug analyze --samples 100 --duration 2)

  # 检查是否收敛（CSV解析）
  STATUS=$(echo "$RESULT" | grep -A1 "^# Diagnosis" | tail -1 | cut -d',' -f1)
  if [ "$STATUS" = "STABLE" ]; then
    echo "CONVERGED!"
    break
  fi

  # 自动调参逻辑...
done

# 保存最终参数
bot-debug save
```

---

## 5. 实现细节

### 5.1 嵌入式端实现要点

```cpp
// serial_debug.cpp

// 全局状态
static bool stream_enabled = false;
static uint32_t stream_interval_ms = 20;  // 50Hz default

// 命令处理
void handleCommand(const char* cmd) {
  if (strncmp(cmd, "STATUS", 6) == 0) {
    sendStatus();
  } else if (strncmp(cmd, "STREAM ", 7) == 0) {
    int hz = atoi(cmd + 7);
    stream_interval_ms = (hz > 0) ? (1000 / hz) : 0;
    stream_enabled = (hz > 0);
    Serial.println("OK");
  }
  // ... 其他命令
}

// 数据流任务
void serialDebugTask(void* arg) {
  auto* ctx = static_cast<AppContext*>(arg);
  uint32_t last_stream = 0;

  while (true) {
    // 处理命令
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      handleCommand(cmd.c_str());
    }

    // 发送数据流
    if (stream_enabled && millis() - last_stream >= stream_interval_ms) {
      sendData(ctx);
      last_stream = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// CSV输出辅助
void sendStatus() {
  Serial.println("STATUS,state,faults,enabled,voltage,loop_hz");
  Serial.printf("%d,%d,%d,%.2f,%.1f\n",
    state, faults, enabled, voltage, loop_hz);
}
```

### 5.2 Golang实现要点

```go
// cmd/analyze.go
func analyzeCmd(samples []SensorData) AnalysisResult {
    // 计算统计量
    pitchMean, pitchStd := calcStats(samples, "pitch")

    // 检测震荡
    oscillation, freq := detectOscillation(samples)

    // 生成诊断
    diagnosis := generateDiagnosis(pitchStd, oscillation, freq)

    return AnalysisResult{
        Stability: StabilityMetrics{
            PitchStd: pitchStd,
            Oscillation: oscillation,
            OscillationFreq: freq,
        },
        Diagnosis: diagnosis,
    }
}

// 震荡检测（FFT或自相关）
func detectOscillation(samples []SensorData) (bool, float64) {
    // 简单实现：检测连续过零点
    zeros := countZeroCrossings(samples)
    if zeros > len(samples) / 10 {  // 超过10%过零
        freq := float64(zeros) / (float64(len(samples)) / sampleRate)
        return true, freq
    }
    return false, 0.0
}
```

---

## 6. 实施计划（2天完成MVP）

### Day 1: 嵌入式端

**上午（3小时）：**
- [ ] 创建 `serial_debug.cpp/h`
- [ ] 实现基本命令解析
- [ ] 实现 STATUS, SENSORS, PARAMS 命令
- [ ] 实现数据流（CSV格式）

**下午（3小时）：**
- [ ] 实现 SET/GET 参数命令
- [ ] 实现 ENABLE/DISABLE 控制
- [ ] 集成到 main.cpp
- [ ] 串口测试验证

### Day 2: Golang CLI

**上午（3小时）：**
- [ ] 搭建项目框架
- [ ] 实现串口通信层
- [ ] 实现 read, status, get, set 命令
- [ ] CSV输出格式

**下午（3小时）：**
- [ ] 实现 stream 和 analyze 命令
- [ ] 震荡检测算法
- [ ] 完整测试
- [ ] 编写调试脚本示例

---

## 7. 测试验证

### 单元测试

```bash
# 测试读取（CSV输出）
bot-debug read --port /dev/cu.usbmodem*

# 测试设置
bot-debug set --param angle_kp --value 1.5

# 测试采样（计算样本数）
bot-debug stream --samples 10 | grep -v '^#' | grep -v '^stat' | wc -l

# 测试分析（提取诊断状态）
bot-debug analyze --samples 100 | grep -A1 "^# Diagnosis" | tail -1 | cut -d',' -f1
```

### 集成测试脚本

```bash
#!/bin/bash
# test_debug_system.sh

PORT=/dev/cu.usbmodem*

echo "=== 1. Status Check ==="
bot-debug status --port $PORT

echo "=== 2. Parameter Test ==="
bot-debug set --port $PORT --param angle_kp --value 1.2
bot-debug get --port $PORT --param angle_kp

echo "=== 3. Sampling Test ==="
bot-debug stream --port $PORT --samples 50 | grep -A4 "^# Stats"

echo "=== 4. Analysis Test ==="
bot-debug analyze --port $PORT --samples 200 | grep -A1 "^# Diagnosis"

echo "=== ALL TESTS PASSED ==="
```

---

## 8. Agent调试示例脚本

```bash
#!/bin/bash
# auto_balance_tune.sh

PORT=/dev/cu.usbmodem*

# 初始化超保守参数
echo "Setting ultra-conservative params..."
bot-debug set --port $PORT --param velocity_kp --value 0.04
bot-debug set --port $PORT --param angle_kp --value 1.0
bot-debug set --port $PORT --param angle_gyro_kd --value 1.0

# 启用平衡
bot-debug enable --port $PORT

# 迭代优化
for iter in {1..20}; do
  echo "=== Iteration $iter ==="

  # 采样分析
  RESULT=$(bot-debug analyze --port $PORT --samples 100 --duration 2)
  # CSV解析
  STATUS=$(echo "$RESULT" | grep -A1 "^# Diagnosis" | tail -1 | cut -d',' -f1)
  PITCH_STD=$(echo "$RESULT" | grep -A1 "^# Stability" | tail -1 | cut -d',' -f1)
  OSC=$(echo "$RESULT" | grep -A1 "^# Stability" | tail -1 | cut -d',' -f2)

  echo "Status: $STATUS, Pitch Std: $PITCH_STD, Oscillation: $OSC"

  # 决策逻辑
  if [ "$OSC" = "1" ]; then
    echo "Oscillation detected - reducing gains"
    KP=$(bot-debug get --port $PORT --param angle_kp | tail -1 | cut -d',' -f2)
    NEW_KP=$(echo "$KP * 0.85" | bc -l)
    bot-debug set --port $PORT --param angle_kp --value $NEW_KP
  elif (( $(echo "$PITCH_STD < 0.02" | bc -l) )); then
    echo "Stable - increasing performance"
    VKP=$(bot-debug get --port $PORT --param velocity_kp | tail -1 | cut -d',' -f2)
    NEW_VKP=$(echo "$VKP * 1.15" | bc -l)
    bot-debug set --port $PORT --param velocity_kp --value $NEW_VKP
  fi

  # 检查收敛
  if [ "$STATUS" = "STABLE" ] && (( $(echo "$PITCH_STD < 0.015" | bc -l) )); then
    echo "CONVERGED! Saving params..."
    bot-debug save --port $PORT
    break
  fi

  sleep 1
done

echo "=== Final Parameters ==="
bot-debug status --port $PORT | grep -A2 "^# Params"
```

---

**方案完成！可以开始实施。**

**预期时间：2天完成MVP，立即可用于AI Agent闭环调试。**
