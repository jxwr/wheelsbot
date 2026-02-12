# 双摇杆遥控与位置环控制方案

## 决策记录

### 1. 位置估计方案
**选择**: 编码器积分（相对位置）
- **原因**: 不需要额外传感器，实现简单，短期精度足够
- **公式**: `position += (wL + wR) / 2 * wheel_radius * dt`
- **限制**: 会累积误差，但遥控场景下用户持续操作可容忍

### 2. 偏航估计方案
**选择**: IMU gyro_z 积分
- **原因**: 已有 MPU6050，无需额外硬件
- **注意**: 长期会漂移，但遥控场景短期使用足够
- **替代**: 如需长期精度，后续可添加地磁计融合

### 3. 控制架构
```
Phase 1: Position Loop (10Hz)
    └── 输入: 位置误差 (目标 - 当前)
    └── 输出: 目标速度 (rad/s)
    └── 限制: ±0.5 m/s

Phase 2: Velocity Loop (50Hz) [已有]
    └── 输入: 速度误差
    └── 输出: 目标倾角
    └── 限制: ±8°

Phase 3: Angle Loop (200Hz) [已有]
    └── 输入: 角度误差
    └── 输出: 电机扭矩

Phase 4: Yaw Mixing
    └── 输入: 目标偏航角速度
    └── 输出: 左右轮差速
    └── 限制: ±30% 纵向命令
```

### 4. 双摇杆映射
| 摇杆 | 方向 | 控制量 | 范围 |
|------|------|--------|------|
| 左摇杆 | 前后 | 目标位置变化率 | ±0.5 m/s |
| 右摇杆 | 左右 | 目标偏航角速度 | ±3.0 rad/s |

### 5. 安全限制
```cpp
max_linear_velocity = 0.5 m/s      // 最大线速度
max_yaw_rate = 3.0 rad/s           // 最大偏航速度 (~172°/s)
max_position_integral = 1.0 m      // 位置积分限制
yaw_deadband = 0.1                 // 偏航死区
linear_deadband = 0.05             // 线速度死区
```

### 6. 模式切换逻辑
- 进入 `/control` 页面 → 自动启用遥控模式
- 离开页面或点击停止 → 恢复定点模式（目标位置 = 当前位置）
- 主页面显示当前模式状态

### 7. 实现阶段
| Phase | 内容 | 文件 |
|-------|------|------|
| 1 | 位置环控制器 | position_controller.h/cpp |
| 2 | 集成到级联控制器 | cascade_controller.h/cpp, main.cpp |
| 3 | 偏航控制 | main.cpp (yaw estimation + mixing) |
| 4 | Web界面 | control.html, wifi_debug.cpp |

### 8. 参考标准
- CLAUDE.md Section 2.2: Cascade Control Philosophy
- CLAUDE.md Section 3.1: Generic ControlLoop Interface
- CLAUDE.md Section 4: Technical Standards (C++17, static allocation)
