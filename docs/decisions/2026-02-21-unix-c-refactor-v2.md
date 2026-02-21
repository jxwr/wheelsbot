# Decision: Unix C Style Refactor V2

Date: 2026-02-21
Author: Claude Code
Branch: `refactor/unix-c-style`

## Context

基于最新代码（commit 4455468）重新设计重构方案。当前代码已调优，控制逻辑稳定。

**重构原则**：
1. **不改变任何控制逻辑** — 算法完全保持不变
2. **不改变功能** — WiFi/OTA/串口调试/参数持久化全部保留
3. **不改变接口** — WebSocket 协议、串口命令格式保持兼容
4. **只改代码组织方式** — C++ 类 → C struct + 函数

---

## 当前代码结构

```
src/
├── main.cpp (697 行)              # 任务 + 串口命令 + 参数持久化
├── app_context.h (71 行)          # 硬件 + 状态大杂烩
├── shared_state.h (56 行)         # 4 个共享状态结构
├── pins.h                         # GPIO 定义
├── wifi_debug.h/cpp (522 行)      # WiFi/WebSocket
├── control/
│   ├── balance_controller.h (550 行)      # BalanceController 类
│   └── balance_controller_defaults.h (76 行) # 参数默认值
└── hardware/
    ├── imu_interface.h            # 虚基类 (无实际用途)
    ├── encoder_interface.h        # 虚基类 (无实际用途)
    ├── hardware_manager.h/cpp     # 几乎无用
    ├── simplefoc_sensor_adapter.h # SimpleFOC 适配器
    └── imu_mpu6050_hal.h/cpp      # MPU6050 实现

总计: ~2000 行，14 个文件
```

---

## 重构后代码结构

```
src/
├── main.cpp              # 硬件实例 + FreeRTOS 任务 + 串口命令
├── robot.h               # 所有数据结构定义（配置/状态/调试）
├── balance.c             # 平衡控制算法（从 BalanceController::step 提取）
├── imu_mpu6050.c         # IMU 读取
├── wifi_ctrl.cpp         # WiFi/WebSocket/OTA（保留原样，改名）
└── pins.h                # GPIO 定义（保留）

总计: ~800 行，6 个文件
```

---

## 详细设计

### 1. robot.h — 统一的数据结构定义

```c
#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>
#include <stdbool.h>

/*
 * 平衡控制参数（对应 BalanceController::Params）
 */
struct BalanceParams {
    float angle_kp;
    float gyro_kp;
    float distance_kp;
    float speed_kp;
    float yaw_angle_kp;
    float yaw_gyro_kp;
    float lqr_u_kp;
    float lqr_u_ki;
    float zeropoint_kp;
    float lpf_target_vel_tf;
    float lpf_zeropoint_tf;
    float ff_gain;
    float max_tilt_deg;
    float pitch_offset;
    float pid_limit;
    float lift_accel_thresh;
    float lift_vel_thresh;
};

/*
 * 默认参数（从 balance_controller_defaults.h 提取）
 */
static const struct BalanceParams DEFAULT_PARAMS = {
    .angle_kp = 1.0f,
    .gyro_kp = 0.08f,
    .distance_kp = 0.3f,
    .speed_kp = 1.0f,
    .yaw_angle_kp = 1.0f,
    .yaw_gyro_kp = 0.04f,
    .lqr_u_kp = 1.0f,
    .lqr_u_ki = 0.0f,
    .zeropoint_kp = 0.002f,
    .lpf_target_vel_tf = 0.6f,
    .lpf_zeropoint_tf = 0.1f,
    .ff_gain = 0.0f,
    .max_tilt_deg = 60.0f,
    .pitch_offset = 0.0f,
    .pid_limit = 10.0f,
    .lift_accel_thresh = 50.0f,
    .lift_vel_thresh = 150.0f,
};

/*
 * IMU 数据（对应 ImuShared）
 */
struct ImuState {
    volatile bool valid;
    volatile float ax, ay, az;   // m/s^2
    volatile float gx, gy, gz;   // rad/s
    volatile float pitch_deg;
    volatile float roll_deg;
    volatile float yaw_deg;
};

/*
 * 轮子数据（对应 WheelShared）
 */
struct WheelState {
    volatile bool valid;
    volatile float wL, wR;       // rad/s
    volatile float xL, xR;       // rad
};

/*
 * 控制输出（对应 BalanceShared）
 */
struct BalanceOutput {
    volatile float motor_L;
    volatile float motor_R;
    volatile bool ok;
};

/*
 * 控制命令（对应 CommandShared）
 */
struct CommandState {
    volatile bool balance_enable;
    volatile bool motor_enable;
    volatile int req_motion_mode;
    volatile int req_torque_mode;
    volatile bool req_apply_mode;
    volatile float manual_target;
    volatile float req_vlimit;
    volatile float req_tlimit;
    volatile bool req_apply_limits;
    volatile bool req_apply_pid;
    volatile float req_pid_p, req_pid_i, req_pid_d;
};

/*
 * 调试数据（对应 BalanceDebug）
 * 供 WebSocket 遥测使用
 */
struct BalanceDebug {
    float ax, ay, az, gx, gy, gz;
    float pitch, pitch_rate, roll, yaw_rate;
    float wheel_velocity, wL, wR, xL, xR;
    float angle_contribution, gyro_contribution;
    float distance_contribution, speed_contribution;
    float lqr_u_raw, lqr_u_compensated;
    float yaw_output, heading;
    float pitch_offset, distance_zeropoint;
    float left_motor, right_motor;
    bool running;
    uint32_t fault_flags;
    bool wheel_lifted;
    bool cog_adapt_active;
    float cog_distance_ctrl, cog_adjustment, cog_lqr_u, cog_speed;
    bool cog_has_joystick;
};

/*
 * 全局实例（在 main.cpp 中定义）
 */
extern struct BalanceParams g_params;
extern struct ImuState       g_imu;
extern struct WheelState     g_wheel;
extern struct BalanceOutput  g_bal_out;
extern struct CommandState   g_cmd;
extern struct BalanceDebug   g_debug;
extern volatile float        g_target_vel;
extern volatile float        g_target_yaw;

/*
 * 接口函数
 */
void balance_init(void);
void balance_step(float dt);
void balance_reset(void);
void balance_get_debug(struct BalanceDebug* out);
void balance_get_params(struct BalanceParams* out);
void balance_set_params(const struct BalanceParams* p);

void imu_init(void);
void imu_read(void);

#endif /* ROBOT_H */
```

### 2. balance.c — 控制算法（逻辑完全保持不变）

```c
#include "robot.h"
#include <math.h>

/* 内部 PID 状态（使用 SimpleFOC PIDController） */
#include <SimpleFOC.h>
static PIDController pid_angle     = {1, 0, 0, 100000, 1};
static PIDController pid_gyro      = {1, 0, 0, 100000, 1};
static PIDController pid_distance  = {1, 0, 0, 100000, 1};
static PIDController pid_speed     = {1, 0, 0, 100000, 1};
static PIDController pid_yaw_angle = {1, 0, 0, 100000, 1};
static PIDController pid_yaw_gyro  = {1, 0, 0, 100000, 1};
static PIDController pid_lqr_u     = {1, 1, 0, 100000, 1};
static PIDController pid_zeropoint = {1, 0, 0, 100000, 1};
static LowPassFilter  lpf_target_vel = {0.2f};
static LowPassFilter  lpf_zeropoint  = {0.1f};

/* 内部状态 */
static float distance_zeropoint = -256.0f;
static float speed_prev = 0;
static float prev_target_vel = 0;
static float heading_current = 0;
static float heading_target = 0;
static int   uncontrollable = 0;
static bool  had_joystick_input = false;
static bool  had_yaw_input = false;

static void reset_pid(PIDController* pid) {
    float p = pid->P, i = pid->I, d = pid->D;
    float ramp = pid->output_ramp, lim = pid->limit;
    pid->~PIDController();
    new (pid) PIDController(p, i, d, ramp, lim);
}

static void reset_all_pids(void) {
    reset_pid(&pid_angle);
    reset_pid(&pid_gyro);
    reset_pid(&pid_distance);
    reset_pid(&pid_speed);
    reset_pid(&pid_yaw_angle);
    reset_pid(&pid_yaw_gyro);
    reset_pid(&pid_lqr_u);
    reset_pid(&pid_zeropoint);
}

void balance_init(void) {
    /* 从默认参数同步到 PID */
    balance_set_params(&DEFAULT_PARAMS);
}

void balance_step(float dt) {
    /* ===== 完全复制 BalanceController::step() 的逻辑 ===== */

    /* 1. 安全检查 */
    if (!g_cmd.balance_enable || !g_imu.valid || !g_wheel.valid) {
        g_bal_out.motor_L = g_bal_out.motor_R = 0;
        g_bal_out.ok = false;
        g_debug.running = false;
        g_debug.fault_flags = !g_cmd.balance_enable ? 4 : 2;
        reset_all_pids();
        return;
    }

    float pitch_deg = g_imu.pitch_deg;

    /* 2. 倾角保护 */
    if (fabsf(pitch_deg) > g_params.max_tilt_deg) {
        uncontrollable = 1;
    }
    if (uncontrollable != 0) {
        if (fabsf(pitch_deg) < g_params.max_tilt_deg * 0.4f) {
            uncontrollable++;
        }
        if (uncontrollable > 200) {
            uncontrollable = 0;
        }
    }
    if (uncontrollable != 0) {
        g_bal_out.motor_L = g_bal_out.motor_R = 0;
        g_bal_out.ok = false;
        g_debug.running = false;
        g_debug.fault_flags = 1;
        reset_all_pids();
        return;
    }

    /* 3. 前馈 + 摇杆滤波 */
    float target_vel_filtered = lpf_target_vel(g_target_vel);
    float desired_accel = 0.0f;
    if (dt > 0.0001f) {
        desired_accel = (target_vel_filtered - prev_target_vel) / dt;
    }
    prev_target_vel = target_vel_filtered;

    float ff_angle_deg = (desired_accel / 9.8f) * (180.0f / 3.14159f) * g_params.ff_gain;
    ff_angle_deg = constrain(ff_angle_deg, -10.0f, 10.0f);

    float target_wheel_vel = target_vel_filtered / 0.03f;  // WHEEL_RADIUS

    /* 4. 角度环 + 角速度环 */
    float angle_setpoint = g_params.pitch_offset + ff_angle_deg;
    float angle_error = pitch_deg - angle_setpoint;
    float angle_ctrl = pid_angle(angle_error);

    float pitch_rate_dps = g_imu.gy * (180.0f / 3.14159f);
    float gyro_ctrl = pid_gyro(pitch_rate_dps);

    /* 5. 位置环 + 速度环 */
    float distance = (g_wheel.xL + g_wheel.xR) * 0.5f;
    float speed = (g_wheel.wL + g_wheel.wR) * 0.5f;

    if (distance_zeropoint < -200.0f) {
        distance_zeropoint = distance;
    }

    bool has_joystick_input = (fabsf(g_target_vel) > 0.01f);
    if (has_joystick_input) {
        distance_zeropoint = distance;
        reset_pid(&pid_lqr_u);
    }

    if (fabsf(speed) > 30.0f) {
        distance_zeropoint = distance;
    }

    had_joystick_input = has_joystick_input;

    float distance_ctrl = pid_distance(distance - distance_zeropoint);
    float speed_ctrl = pid_speed(speed - target_wheel_vel);

    /* 6. 轮子抬起检测（已禁用） */
    float speed_accel = 0.0f;
    if (dt > 0.0001f) {
        speed_accel = fabsf(speed - speed_prev) / dt;
    }
    speed_prev = speed;
    bool wheel_lifted = false;

    float lqr_u;
    if (wheel_lifted) {
        distance_zeropoint = distance;
        lqr_u = angle_ctrl + gyro_ctrl;
        reset_pid(&pid_lqr_u);
    } else {
        lqr_u = angle_ctrl + gyro_ctrl + distance_ctrl + speed_ctrl;
    }

    g_debug.lqr_u_raw = lqr_u;

    /* 7. CoG 自适应 */
    bool cog_active = fabsf(lqr_u) < 5.0f && !has_joystick_input
                   && fabsf(speed) < 2.0f && !wheel_lifted;
    if (cog_active) {
        lqr_u = pid_lqr_u(lqr_u);
        float zeropoint_adj = pid_zeropoint(lpf_zeropoint(distance_ctrl));
        g_params.pitch_offset -= zeropoint_adj;
        if (g_params.pitch_offset > 10.0f) g_params.pitch_offset = 10.0f;
        if (g_params.pitch_offset < -10.0f) g_params.pitch_offset = -10.0f;
        g_debug.cog_adapt_active = true;
        g_debug.cog_adjustment = zeropoint_adj;
    } else {
        g_debug.cog_adapt_active = false;
        g_debug.cog_adjustment = 0.0f;
        reset_pid(&pid_lqr_u);
    }
    g_debug.cog_distance_ctrl = distance_ctrl;
    g_debug.cog_lqr_u = lqr_u;
    g_debug.cog_speed = speed;
    g_debug.cog_has_joystick = has_joystick_input;

    /* 8. 偏航控制 */
    float yaw_output = 0.0f;
    bool has_yaw_input = (fabsf(g_target_yaw) > 0.05f);

    if (has_yaw_input) {
        heading_target += g_target_yaw * dt;
        while (heading_target > 3.14159f)  heading_target -= 2.0f * 3.14159f;
        while (heading_target < -3.14159f) heading_target += 2.0f * 3.14159f;

        if (!had_yaw_input) {
            heading_current = heading_target;
        }

        heading_current += g_imu.gz * dt;
        while (heading_current > 3.14159f)  heading_current -= 2.0f * 3.14159f;
        while (heading_current < -3.14159f) heading_current += 2.0f * 3.14159f;

        float yaw_angle_error = heading_target - heading_current;
        if (yaw_angle_error > 3.14159f)  yaw_angle_error -= 2.0f * 3.14159f;
        if (yaw_angle_error < -3.14159f) yaw_angle_error += 2.0f * 3.14159f;

        float yaw_angle_ctrl = pid_yaw_angle(yaw_angle_error);
        float yaw_gyro_ctrl = -g_params.yaw_gyro_kp * g_imu.gz;
        yaw_output = yaw_angle_ctrl + yaw_gyro_ctrl;
    } else {
        yaw_output = -g_params.yaw_gyro_kp * g_imu.gz;
        heading_current = 0;
        heading_target = 0;
        reset_pid(&pid_yaw_angle);
    }
    had_yaw_input = has_yaw_input;

    /* 9. 差速输出 */
    g_bal_out.motor_L = -0.5f * (lqr_u + yaw_output);
    g_bal_out.motor_R = -0.5f * (lqr_u - yaw_output);
    g_bal_out.ok = true;

    /* 10. 更新调试数据 */
    g_debug.ax = g_imu.ax; g_debug.ay = g_imu.ay; g_debug.az = g_imu.az;
    g_debug.gx = g_imu.gx; g_debug.gy = g_imu.gy; g_debug.gz = g_imu.gz;
    g_debug.pitch = g_imu.pitch_deg * (3.14159f / 180.0f);
    g_debug.pitch_rate = g_imu.gy;
    g_debug.roll = g_imu.roll_deg * (3.14159f / 180.0f);
    g_debug.yaw_rate = g_imu.gz;
    g_debug.wheel_velocity = speed;
    g_debug.wL = g_wheel.wL; g_debug.wR = g_wheel.wR;
    g_debug.xL = g_wheel.xL; g_debug.xR = g_wheel.xR;
    g_debug.angle_contribution = angle_ctrl;
    g_debug.gyro_contribution = gyro_ctrl;
    g_debug.distance_contribution = distance_ctrl;
    g_debug.speed_contribution = speed_ctrl;
    g_debug.lqr_u_compensated = lqr_u;
    g_debug.yaw_output = yaw_output;
    g_debug.pitch_offset = g_params.pitch_offset;
    g_debug.distance_zeropoint = distance_zeropoint;
    g_debug.heading = heading_current;
    g_debug.left_motor = g_bal_out.motor_L;
    g_debug.right_motor = g_bal_out.motor_R;
    g_debug.running = true;
    g_debug.fault_flags = 0;
    g_debug.wheel_lifted = wheel_lifted;
}

void balance_reset(void) {
    reset_all_pids();
    heading_current = 0;
    heading_target = 0;
    distance_zeropoint = -256.0f;
    speed_prev = 0;
    prev_target_vel = 0;
    uncontrollable = 0;
    had_joystick_input = false;
    had_yaw_input = false;
}

void balance_get_debug(struct BalanceDebug* out) {
    *out = g_debug;
}

void balance_get_params(struct BalanceParams* out) {
    *out = g_params;
}

void balance_set_params(const struct BalanceParams* p) {
    g_params = *p;

    pid_angle.P     = p->angle_kp;
    pid_gyro.P      = p->gyro_kp;
    pid_distance.P  = p->distance_kp;
    pid_speed.P     = p->speed_kp;
    pid_yaw_angle.P = p->yaw_angle_kp;
    pid_yaw_gyro.P  = p->yaw_gyro_kp;
    pid_lqr_u.P     = p->lqr_u_kp;
    pid_lqr_u.I     = p->lqr_u_ki;
    pid_zeropoint.P = p->zeropoint_kp;
    lpf_target_vel.Tf = p->lpf_target_vel_tf;
    lpf_zeropoint.Tf  = p->lpf_zeropoint_tf;

    float lim = p->pid_limit;
    pid_angle.limit     = lim;
    pid_gyro.limit      = lim;
    pid_distance.limit  = lim;
    pid_speed.limit     = lim;
    pid_yaw_angle.limit = lim;
    pid_yaw_gyro.limit  = lim;
    pid_lqr_u.limit     = lim;
}
```

### 3. wifi_ctrl.cpp — 适配新接口

```cpp
/* WiFi/WebSocket 代码基本保持不变，只需修改数据访问方式 */

// 旧代码:
//   s_ctx->balance.getDebug(dbg);
//   s_ctx->balance.getParams(p);
//   s_ctx->balance.setParams(p);

// 新代码:
//   balance_get_debug(&dbg);
//   balance_get_params(&p);
//   balance_set_params(&p);

// 旧代码:
//   s_ctx->imu_state.ax
//   s_ctx->wheel_state.wL
//   s_ctx->cmd_state.balance_enable
//   s_ctx->target_linear_vel

// 新代码:
//   g_imu.ax
//   g_wheel.wL
//   g_cmd.balance_enable
//   g_target_vel
```

### 4. 删除的文件

```
src/app_context.h                   → 合并到 robot.h
src/shared_state.h                  → 合并到 robot.h
src/control/balance_controller.h    → balance.c
src/control/balance_controller_defaults.h → 合并到 robot.h
src/hardware/imu_interface.h        → 删除
src/hardware/encoder_interface.h    → 删除
src/hardware/hardware_manager.*     → 删除
src/hardware/simplefoc_sensor_adapter.h → 删除（直接在 main.cpp 调用 SimpleFOC）
src/hardware/imu_mpu6050_hal.*      → imu_mpu6050.c
```

---

## 重构步骤（保证每步可编译）

### Phase 1: 准备（不破坏编译）
1. 创建 `robot.h`，定义所有 struct
2. 创建 `balance.c`，复制 `BalanceController::step()` 逻辑
3. 创建 `imu_mpu6050.c`，从 HAL 提取核心代码

### Phase 2: 切换（修改 main.cpp）
1. 在 `main.cpp` 中定义全局实例
2. 修改 `balanceTask` 调用 `balance_step()`
3. 修改 `imuTask` 调用 `imu_read()`

### Phase 3: 清理
1. 修改 `wifi_ctrl.cpp` 使用新接口
2. 删除旧文件
3. 验证编译通过

---

## 代码量对比

| 指标 | 当前 | 重构后 | 变化 |
|------|------|--------|------|
| 文件数 | 14 | 6 | -57% |
| 总行数 | ~2000 | ~800 | -60% |
| 抽象层 | 4 层 | 1 层 | -75% |
| 虚函数 | 2 个基类 | 0 | -100% |

---

## 风险控制

| 风险 | 缓解措施 |
|------|----------|
| 算法行为变化 | 逐行对比，确保逻辑完全相同 |
| WebSocket 协议变化 | 保持 JSON 格式完全一致 |
| 参数丢失 | 使用相同的默认值 |
| 编译失败 | 分阶段重构，每阶段确保可编译 |

---

## Follow-ups

1. [ ] Phase 1: 创建 `robot.h`
2. [ ] Phase 1: 创建 `balance.c`
3. [ ] Phase 1: 创建 `imu_mpu6050.c`
4. [ ] Phase 2: 修改 `main.cpp`
5. [ ] Phase 3: 修改 `wifi_ctrl.cpp`
6. [ ] Phase 3: 删除旧文件
7. [ ] 验证编译
8. [ ] 实机测试
