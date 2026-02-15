# WheelsBot 技术文档

## 1. 系统概述

WheelsBot 是一个基于 ESP32-S3 的自平衡双轮机器人控制系统。系统采用级联控制架构，结合 IMU 传感器融合和 FOC 电机控制，实现稳定的自平衡和遥控功能。

### 硬件配置

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控 | ESP32-S3 | 240MHz, 320KB RAM |
| IMU | MPU6050 | 6轴加速度计+陀螺仪 |
| 编码器 | AS5600 | 磁编码器, 12位分辨率 |
| 电机 | BLDC | 直流无刷电机 |
| 驱动 | SimpleFOC | 3PWM FOC驱动 |

---

## 2. 软件架构

### 2.1 分层结构

```
┌─────────────────────────────────────────┐
│         Web 调试/遥控界面               │
│         (index.html, control.html)      │
├─────────────────────────────────────────┤
│         WiFi + WebSocket                │
│         (wifi_debug.cpp)                │
├─────────────────────────────────────────┤
│         控制层                          │
│         (BalanceController)             │
├─────────────────────────────────────────┤
│         传感器抽象层                     │
│         (HardwareManager, IMU, Encoder) │
├─────────────────────────────────────────┤
│         SimpleFOC 驱动层                │
│         (BLDCMotor, BLDCDriver3PWM)     │
└─────────────────────────────────────────┘
```

### 2.2 任务调度

| 任务 | 频率 | 核心 | 说明 |
|------|------|------|------|
| imuTask | 200Hz | Core 0 | IMU 数据读取 |
| balanceTask | 200Hz | Core 0 | 平衡控制计算 |
| focTask | 1kHz | Core 1 | FOC 电机控制 |
| wifiDebugTask | 20Hz | Core 0 | WebSocket 遥测 |
| ledTask | 2Hz | Core 0 | LED 指示灯 |

---

## 3. 控制算法

### 3.1 BalanceController

系统使用 LQR 分解思想，将复杂控制器拆分为多个可独立调参的 PID 控制器：

```cpp
// 控制框图
target_velocity ──┐
                  ├──> speed_pid ──┐
wheel_velocity ───┘                 │
                                   ├──> sum ──> angle_pid ──┐
pitch (rad) ────────────────────────┘                      │
                                                                 ├──> sum ──> motor
pitch_rate ───────────────────────────────────────────────┐    │
                                                          │    │
distance_error ──> distance_pid ──────────────────────────┘    │
wheel_position ─────────────────────────────────────────────┘

yaw_rate ──> yaw_gyro_pid ──┐
                            ├──> sum ──> differential
yaw_error ──> yaw_angle_pid ─┘
```

### 3.2 各环说明

| 控制器 | 作用 | 典型参数 |
|--------|------|----------|
| pid_angle | 角度环 - 主要回复力 | Kp=6.0 |
| pid_gyro | 角速度环 - 阻尼防震荡 | Kp=0.06 |
| pid_distance | 位置环 - 防止漂移 | Kp=0.5 |
| pid_speed | 速度环 - 跟踪目标速度 | Kp=0.7 |
| pid_yaw_angle | 偏航角环 - heading保持 | Kp=1.0 |
| pid_yaw_gyro | 偏航率环 - 转向阻尼 | Kp=0.04 |
| pid_lqr_u | 输出积分补偿 - 静摩擦 | Kp=1.0, Ki=15 |

### 3.3 输入输出

**BalanceInput:**
```cpp
struct BalanceInput {
  float pitch;            // 俯仰角 (rad)
  float pitch_rate;       // 俯仰角速度 (rad/s)
  float wheel_position;  // 平均轮位置 (rad)
  float wheel_velocity;  // 平均轮速度 (rad/s)
  float yaw_rate;        // 偏航角速度 (rad/s)
  float target_velocity; // 目标线速度 (m/s)
  float target_yaw_rate; // 目标偏航率 (rad/s)
  float dt;              // 时间步长 (s)
  bool enabled;          // 使能标志
  bool sensors_valid;    // 传感器有效标志
};
```

**BalanceOutput:**
```cpp
struct BalanceOutput {
  float left_motor;   // 左电机电压 (V)
  float right_motor;  // 右电机电压 (V)
  bool valid;         // 输出有效标志
  uint32_t fault_flags; // 故障标志
};
```

---

## 4. 传感器数据

### 4.1 IMU (MPU6050)

原始数据：
- 加速度: ax, ay, az (m/s²)
- 陀螺仪: gx, gy, gz (rad/s)

融合后：
- pitch, roll, yaw (degrees)

互补滤波器参数：
- alpha = 0.98 (陀螺仪权重)

### 4.2 编码器

- 左轮速度: wL (rad/s)
- 右轮速度: wR (rad/s)
- 左轮位置: xL (rad)
- 右轮位置: xR (rad)

注意：编码器方向已取反，使正向速度对应前进方向。

---

## 5. 安全机制

### 5.1 故障类型

| 故障标志 | 说明 |
|----------|------|
| BALANCE_FAULT_TILT_TOO_LARGE | 倾角超过阈值 |
| BALANCE_FAULT_SENSOR_LOST | 传感器失效 |
| BALANCE_FAULT_DISABLED | 控制器被禁用 |

### 5.2 倾斜保护

安全- 最大倾角: 25° (可配置)
- 恢复条件: 倾角降至 10° 以下
- 不可控状态持续 200 个周期 (~1s) 后完全停止

### 5.3 轮子抬起检测

```cpp
// 加速度检测
speed_accel = |speed - speed_prev| / dt
wheel_lifted = (speed_accel > 10 rad/s²) || (|speed| > 50 rad/s)
```

轮子离地时自动禁用 distance 和 speed 环，仅保留 angle + gyro 环。

---

## 6. 通信协议

### 6.1 WebSocket 消息

**遥测消息 (20Hz):**
```json
{
  "type": "telem",
  "t": 12345678,
  "pitch": 3.14,
  "pitch_deg": 180.0,
  "pr": 0.5,
  "angle": 1.2,
  "gyro": 0.3,
  "dist": 0.5,
  "spd": 0.7,
  "lqr_raw": 2.0,
  "lqr_comp": 2.1,
  "yaw": 0.5,
  "heading": 1.57,
  "pofs": 0.1,
  "dzp": -256.0,
  "lm": 1.5,
  "rm": 1.5,
  "state": 1,
  "fault": 0,
  "lifted": 0
}
```

**参数消息:**
```json
{
  "type": "params",
  "balance": {
    "angle_kp": 6.0,
    "gyro_kp": 0.06,
    ...
  },
  "foc": {
    "voltage_limit": 8.0,
    "velocity_limit": 100.0,
    ...
  },
  "ctrl": {
    "motor_enable": true,
    "balance_enable": true,
    ...
  }
}
```

### 6.2 命令消息

设置参数:
```json
{"type": "set_angle", "key": "angle_kp", "value": 6.5}
{"type": "set_velocity", "key": "speed_kp", "value": 0.8}
{"type": "set_yaw", "key": "yaw_angle_kp", "value": 1.2}
{"type": "set_safety", "key": "max_tilt_deg", "value": 25.0}
{"type": "set_foc", "key": "voltage_limit", "value": 10.0}
{"type": "set_ctrl", "key": "motor_enable", "value": true}
{"type": "set_target", "key": "linear_vel", "value": 0.5}
```

---

## 7. 文件结构

```
src/
├── main.cpp                 # 主程序, 任务创建
├── app_context.h            # 全局上下文
├── pins.h                   # 引脚定义
├── shared_state.h           # 共享状态结构
├── wifi_debug.h/cpp         # Web服务器和调试
├── control/
│   └── balance_controller.h # 平衡控制器
├── hardware/
│   ├── imu_interface.h      # IMU 接口
│   ├── imu_mpu6050_hal.cpp  # MPU6050 实现
│   ├── encoder_interface.h   # 编码器接口
│   ├── simplefoc_sensor_adapter.h
│   └── hardware_manager.h/cpp
└── doc/
    └── (本文档)
```

---

## 8. 调参指南

### 8.1 调参顺序

1. **angle_kp**: 先调，使机器人能在失衡时快速回复
2. **gyro_kp**: 添加阻尼，防止震荡
3. **speed_kp**: 调速度跟踪响应
4. **distance_kp**: 调位置保持
5. **yaw_gyro_kp**: 调转向响应

### 8.2 典型参数

```cpp
BalanceController::Params{
  .angle_kp = 6.0f,
  .gyro_kp = 0.06f,
  .distance_kp = 0.5f,
  .speed_kp = 0.7f,
  .yaw_angle_kp = 1.0f,
  .yaw_gyro_kp = 0.04f,
  .lqr_u_kp = 1.0f,
  .lqr_u_ki = 15.0f,
  .max_tilt_deg = 25.0f,
  .pid_limit = 8.0f,
};
```

---

## 9. 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2026-02 | 初始版本, 级联控制 |
| 2.0 | 2026-02 | 简化为 BalanceController |

---

## 10. 参考资料

- SimpleFOC: https://docs.simplefoc.com/
- ESP32 Arduino: https://docs.espressif.com/
- MPU6050 Datasheet
