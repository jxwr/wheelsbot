# WheelsBot 开发指南

两轮自平衡机器人项目，运行于 ESP32-S3 平台。

---

## 1. 项目概述

### 1.1 核心架构

```
src/
├── main.cpp           # 硬件实例 + FreeRTOS 任务 + 串口命令
├── robot.h            # 数据结构定义（配置/状态/调试）
├── balance.cpp        # 平衡控制算法
├── imu_mpu6050.cpp    # IMU 读取
├── wifi_ctrl.cpp      # WiFi/WebSocket/OTA
└── pins.h             # GPIO 定义
```

**设计原则**：数据放 struct，算法用函数，状态全局可见。

### 1.2 技术栈

| 组件 | 技术 |
|------|------|
| 平台 | ESP32-S3 (Arduino Framework) |
| 电机控制 | SimpleFOC (FOC 无刷电机) |
| 实时系统 | FreeRTOS |
| IMU | MPU6050 (互补滤波) |
| 编码器 | AS5600 (I2C 磁编码器) |
| 通信 | WiFi AP + WebSocket |
| 构建系统 | PlatformIO |

---

## 2. 控制系统

### 2.1 任务频率

| 任务 | 频率 | 核心 | 优先级 |
|------|------|------|--------|
| FOC 电机 | 1kHz | 1 | 5 |
| 平衡控制 | 200Hz | 0 | 4 |
| IMU 读取 | 200Hz | 0 | 5 |
| WiFi 遥测 | 20Hz | 0 | 1 |

### 2.2 控制环路结构

LQR 分解的并联 PID 控制：

```
                    ┌─ angle_kp × angle_error ────┐
                    │                              │
                    ├─ gyro_kp × pitch_rate ──────┤
                    │                              │
lqr_u ──────────────┼─ distance_kp × pos_error ───┼──→ motor_output
                    │                              │
                    ├─ speed_kp × wheel_vel ──────┤
                    │                              │
                    └─ lqr_u_kp × lqr_u + ki × ∫ ─┘

differential:  motor_L = -0.5 × (lqr_u + yaw)
               motor_R = -0.5 × (lqr_u - yaw)
```

### 2.3 关键参数

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `angle_kp` | 1.0 | 角度环增益 |
| `gyro_kp` | 0.08 | 角速度阻尼 |
| `distance_kp` | 0.3 | 位置环增益 |
| `speed_kp` | 1.0 | 速度环增益 |
| `zeropoint_kp` | 0.002 | CoG 自适应速率 |
| `pitch_offset` | 0.0 | 重心偏移补偿 (°) |
| `max_tilt_deg` | 60.0 | 最大倾角保护 (°) |

### 2.4 CoG 自适应

机器人静止时自动调整 `pitch_offset` 以补偿重心偏移：

**激活条件**：
- `|lqr_u| < 5.0` (输出较小)
- 无摇杆输入
- `|speed| < 2.0 rad/s`
- 轮子未离地

**调整方向**：`pitch_offset -= zeropoint_kp × distance_ctrl`

---

## 3. 轮子方向约定

**重要**：轮子速度/位置符号必须正确，否则控制环路混乱。

```c
// 正确配置 (与 main 分支一致)
float wL_raw = sensor_left.getVelocity();   // 左轮不取反
float wR_raw = -sensor_right.getVelocity(); // 右轮取反

g_wheel.x_l = sensor_left.getAngle();    // 左轮不取反
g_wheel.x_r = -sensor_right.getAngle();  // 右轮取反
```

---

## 4. 串口调试

### 4.1 命令格式

```
param=value    # 设置参数
param?         # 查询参数
dump           # 显示所有参数
telem          # 显示遥测数据
save           # 保存参数到 Flash
reset          # 重置控制器
help           # 帮助
```

### 4.2 调试工具

```bash
cd tools
python3 debug_check_params.py      # 检查参数
python3 debug_monitor_cog.py       # 监控 CoG 自适应
python3 debug_analyze_oscillation.py  # 分析振荡
```

---

## 5. 命名规范

### 5.1 保留完整的核心词

`angle`, `gyro`, `distance`, `speed`, `yaw`, `pitch`, `roll`

### 5.2 缩写规则

| 完整词 | 缩写 |
|--------|------|
| contribution | ctrl |
| enable | en |
| zeropoint | zero |
| velocity | vel |
| compensated | comp |
| adjustment | adj |
| joystick | joy |
| output | out |

### 5.3 示例

```c
// 正确
float angle_ctrl, gyro_ctrl, distance_ctrl;
bool balance_en, motor_en;
float motor_l, motor_r, w_l, w_r;

// 错误
float angle_contribution, wL, xR;
```

---

## 6. 硬件参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 轮子直径 | 60mm | radius = 0.03m |
| 极对数 | 7 | BLDC 电机 |
| 电源电压 | 12V | 标称 |
| I2C1 | SDA=3, SCL=9 | IMU + 左编码器 |
| I2C2 | SDA=2, SCL=1 | 右编码器 |

---

## 7. 开发流程

### 7.1 探索-计划-编码-验证

1. **Explore**: 使用 `grep`/`Glob` 定位相关代码
2. **Plan**: 输出修改计划，等待确认
3. **Code**: 实现修改，保持现有风格
4. **Verify**: 构建验证 `pio run -e esp32s3`

### 7.2 提交规范

- 使用 [Conventional Commits](https://www.conventionalcommits.org/)
- 类型：`feat:`, `fix:`, `refactor:`, `chore:`
- 示例：`fix(control): correct wheel velocity sign convention`

### 7.3 分支策略

- `main` - 稳定版本
- `refactor/unix-c-style` - 当前开发版本
- 功能分支从 main 创建

---

## 8. 决策记录

重要决策记录在 `docs/decisions/`，格式：

```markdown
# Decision: <标题>

Date: YYYY-MM-DD

## Context
背景描述

## Decision
选择的方案

## Impact
影响范围
```

---

## 9. 故障排查

### 9.1 机器人无法平衡

1. 检查轮子方向符号是否正确
2. 检查 IMU 是否初始化成功 (`telem` 命令)
3. 检查参数是否合理 (`dump` 命令)

### 9.2 机器人漂移

1. 检查 `speed_kp` 是否过小 (<0.3)
2. 检查 CoG 自适应是否激活 (`debug_monitor_cog.py`)
3. 检查 `pitch_offset` 是否收敛

### 9.3 机器人振荡

1. 增加 `gyro_kp` (0.08 → 0.12)
2. 减少 `angle_kp`
3. 使用 `debug_analyze_oscillation.py` 定位振荡源
