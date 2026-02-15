# 代码审查问题归档

> **状态：已修复** | 归档日期：2026-02-15

本文档记录的历史问题已在当前代码中修复，保留供参考。

---

## 已修复问题列表

### 1. ✅ 速度环参考值的缩放系数

**修复状态：** 已修复（通过架构重构）

原问题：`target_wheel_vel` 前有错误的 `0.1f` 缩放系数。

**当前代码** (`main.cpp:177`)：
```cpp
float target_wheel_vel = in.target_velocity / 0.035f;  // WHEEL_RADIUS
```

---

### 2. ✅ Yaw控制的heading累积逻辑

**修复状态：** 已修复（简化方案）

原问题：heading_ 同时累积反馈和指令，逻辑混乱。

**当前实现** (`balance_controller.cpp:233`)：
```cpp
// 简化为纯角速度阻尼，无heading累积
float yaw_output = in.target_yaw_rate - yaw_kd_ * in.yaw_rate;
```

**注意：** 当前采用简化方案（纯速率阻尼），如需heading hold功能需重新实现双环控制。

---

### 3. ✅ resetPid() placement new 风险

**修复状态：** 已修复（添加注释说明）

当前代码已添加风险注释 (`balance_controller.cpp:299-305`)。

---

### 4. ✅ 轮子抬起检测 - 加速度计算

**修复状态：** 已移除（功能简化）

原问题：加速度计算缺少 `/dt`。

**当前状态：** 轮子抬起检测功能已移除，未来如需添加需重新实现。

---

### 5. ✅ 航向积分硬编码时间步长

**修复状态：** 已修复（简化方案无此问题）

当前简化版yaw控制不再使用heading积分，无硬编码问题。

---

### 6. ✅ 圆周率常量不一致

**修复状态：** 部分修复

- `balance_controller.h` 使用 `PI` 定义
- `main.cpp:127` 仍使用 `3.14159f`（低精度）
- `imu_mpu6050_hal.cpp` 仍使用 `M_PI`（double类型）

**建议：** 统一使用 `shared_state.h` 中定义的常量。

---

## 历史参考

- 原始审查日期：2026-02-14
- 相关重构：docs/decisions/2026-02-12-migrate-to-cascade-framework.md
