# Decision: Unix C Style Refactor

Date: 2026-02-20
Author: Claude Code

## Context

当前代码用了不少 C++ 抽象，写法上不够直观。参考 Micro-Wheeled_leg-Robot 项目，发现 C 风格的代码更简洁易懂。

**目标**：用 Unix C 风格重写，但保留所有功能（WiFi、OTA、OLED、FreeRTOS）。

---

## Decision

### 核心原则

```
数据放 struct，算法用函数，状态全局可见
```

### 文件结构

```
src/
├── main.cpp           # 硬件实例 + FreeRTOS 任务 + 串口命令
├── robot.h            # 配置参数 + 运行状态（核心）
├── balance.c          # 平衡控制算法
├── imu_mpu6050.c      # IMU 读取
├── wifi_ctrl.c        # WiFi + WebSocket + OTA
└── pins.h             # GPIO 定义
```

**6 个文件**

---

### robot.h — 唯一的数据结构定义

```c
#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>
#include <stdbool.h>

/*
 * 控制参数（调这些改变机器人行为）
 */
struct config {
    /* 机械 */
    float wheel_radius;      /* 轮子半径 m */
    float supply_voltage;    /* 电源电压 V */

    /* 平衡 PID */
    float angle_kp;
    float gyro_kp;
    float distance_kp;
    float speed_kp;
    float yaw_kp;

    /* 自适应 */
    float lqr_u_kp;
    float lqr_u_ki;
    float zeropoint_kp;
    float pitch_offset;      /* 重心偏移 deg */
    float max_tilt;          /* 最大倾角 deg */
    float out_limit;         /* 输出限幅 V */
};

/*
 * 运行状态（只读，由各模块更新）
 */
struct state {
    /* IMU */
    float ax, ay, az;
    float gx, gy, gz;
    float pitch, roll, yaw;
    bool  imu_ok;

    /* 轮子 */
    float wL, wR;            /* 角速度 rad/s */
    float xL, xR;            /* 角度 rad */
    bool  wheel_ok;

    /* 输出 */
    float motor_L, motor_R;  /* 电压 V */

    /* 命令 */
    float target_vel;        /* m/s */
    float target_yaw;        /* rad/s */
    bool  enabled;

    /* 内部 */
    float dist_zero;
    int   fault;
};

/*
 * 调试数据（balance_step 更新）
 */
struct debug {
    /* 各环输出 */
    float angle_out;
    float gyro_out;
    float dist_out;
    float speed_out;
    float yaw_out;
    float lqr_u;

    /* 自适应 */
    float lqr_i;
    bool  cog_active;

    /* 航向 */
    float heading;
};

/* 全局实例 */
extern struct config cfg;
extern struct state  st;
extern struct debug  dbg;

/* 接口 */
void balance_step(float dt);
void robot_reset(void);

/* 调试输出 */
void print_telemetry(void);      /* 串口打遥测 */
void print_params(void);         /* 串口打参数 */
void handle_serial_cmd(void);    /* 解析串口命令 */

#endif
```

---

### balance.c — 纯函数，无状态隐藏

```c
#include "robot.h"
#include <math.h>

/* 内部变量（文件作用域） */
static float lqr_i = 0;
static float heading = 0;

/* 工具函数 */
static inline float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/*
 * 平衡控制主函数
 * 输入: st (全局状态), cfg (全局配置)
 * 输出: st.motor_L, st.motor_R
 */
void balance_step(float dt) {
    /* 1. 安全检查 */
    if (!st.enabled || !st.imu_ok || !st.wheel_ok) {
        st.motor_L = st.motor_R = 0;
        st.fault = !st.enabled ? 1 : 2;
        return;
    }

    /* 2. 姿态 */
    float pitch_deg = st.pitch * (180.0f / M_PI);

    /* 倾角保护 */
    static int recovery = 0;
    if (fabsf(pitch_deg) > cfg.max_tilt) recovery = 1;
    if (recovery > 0) {
        if (fabsf(pitch_deg) < cfg.max_tilt * 0.4f) recovery++;
        if (recovery > 200) recovery = 0;
        if (recovery > 0) {
            st.motor_L = st.motor_R = 0;
            st.fault = 4;
            return;
        }
    }

    /* 3. 角度环 + 角速度环 */
    float angle_err = pitch_deg - cfg.pitch_offset;
    float angle_out = clamp(cfg.angle_kp * angle_err, -cfg.out_limit, cfg.out_limit);
    float gyro_out  = clamp(cfg.gyro_kp * st.gy, -cfg.out_limit, cfg.out_limit);

    /* 4. 位置环 + 速度环 */
    float dist  = (st.xL + st.xR) * 0.5f;
    float speed = (st.wL + st.wR) * 0.5f;

    bool moving = (fabsf(st.target_vel) > 0.01f);
    if (moving) st.dist_zero = dist;

    float dist_out  = clamp(cfg.distance_kp * (dist - st.dist_zero), -cfg.out_limit, cfg.out_limit);
    float speed_out = clamp(cfg.speed_kp * speed, -cfg.out_limit, cfg.out_limit);

    /* 5. 合成 */
    float u = angle_out + gyro_out + dist_out + speed_out;

    /* 6. 静摩擦补偿 + 重心自适应 */
    dbg.cog_active = false;
    if (fabsf(u) < 5.0f && !moving && fabsf(speed) < 3.0f) {
        lqr_i += cfg.lqr_u_ki * u * dt;
        lqr_i = clamp(lqr_i, -cfg.out_limit, cfg.out_limit);
        u = cfg.lqr_u_kp * u + lqr_i;

        cfg.pitch_offset -= cfg.zeropoint_kp * dist_out;
        cfg.pitch_offset = clamp(cfg.pitch_offset, -10.0f, 10.0f);
        dbg.cog_active = true;
    }

    /* 7. 偏航 */
    float yaw_out = -cfg.yaw_kp * st.gz;
    heading += st.gz * dt;

    /* 8. 差速输出 */
    st.motor_L = -0.5f * (u + yaw_out);
    st.motor_R = -0.5f * (u - yaw_out);
    st.fault = 0;

    /* 9. 填充调试数据 */
    dbg.angle_out = angle_out;
    dbg.gyro_out  = gyro_out;
    dbg.dist_out  = dist_out;
    dbg.speed_out = speed_out;
    dbg.yaw_out   = yaw_out;
    dbg.lqr_u     = u;
    dbg.lqr_i     = lqr_i;
    dbg.heading   = heading;
}

void robot_reset(void) {
    lqr_i = 0;
    heading = 0;
    st.dist_zero = -256;
    st.fault = 0;
}
```

---

### imu_mpu6050.c — 简单直接

```c
#include <Wire.h>
#include <Arduino.h>
#include "robot.h"

#define MPU_ADDR 0x68

static float alpha = 0.98f;
static float _pitch, _roll, _yaw;
static uint32_t _last_us;

void imu_init(void) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);  /* wake up */
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);  /* gyro ±250dps */
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);  /* accel ±2g */
    Wire.write(0x00);
    Wire.endTransmission();

    _last_us = micros();
}

void imu_read(void) {
    uint8_t buf[14];

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14);
    for (int i = 0; i < 14; i++) buf[i] = Wire.read();

    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];

    st.ax = ax / 16384.0f * 9.8f;
    st.ay = ay / 16384.0f * 9.8f;
    st.az = az / 16384.0f * 9.8f;
    st.gx = gx / 131.0f * (M_PI / 180.0f);
    st.gy = gy / 131.0f * (M_PI / 180.0f);
    st.gz = gz / 131.0f * (M_PI / 180.0f);

    uint32_t now = micros();
    float dt = (now - _last_us) * 1e-6f;
    _last_us = now;

    float pitch_acc = atan2f(-st.ax, sqrtf(st.ay*st.ay + st.az*st.az));
    float roll_acc  = atan2f(st.ay, st.az);

    _pitch = alpha * (_pitch + st.gy * dt) + (1 - alpha) * pitch_acc;
    _roll  = alpha * (_roll  + st.gx * dt) + (1 - alpha) * roll_acc;
    _yaw  += st.gz * dt;

    st.pitch = _pitch;
    st.roll  = _roll;
    st.yaw   = _yaw;
    st.imu_ok = true;
}
```

---

### main.cpp — 硬件 + 调度 + 串口调试

```cpp
#include <Arduino.h>
#include <SimpleFOC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "robot.h"
#include "pins.h"

/* ===== 硬件实例 ===== */
BLDCMotor motorL(7), motorR(7);
BLDCDriver3PWM driverL(PIN_PWM_U, PIN_PWM_V, PIN_PWM_W, PIN_EN);
BLDCDriver3PWM driverR(PIN_PWM_U2, PIN_PWM_V2, PIN_PWM_W2, PIN_EN2);
MagneticSensorI2C sensorL(AS5600_I2C);
MagneticSensorI2C sensorR(AS5600_I2C);

/* ===== 全局配置 ===== */
struct config cfg = {
    .wheel_radius  = 0.03f,
    .supply_voltage = 8.0f,
    .angle_kp      = 6.5f,
    .gyro_kp       = 0.06f,
    .distance_kp   = 0.5f,
    .speed_kp      = 0.04f,
    .yaw_kp        = 0.04f,
    .lqr_u_kp      = 1.0f,
    .lqr_u_ki      = 15.0f,
    .zeropoint_kp  = 0.002f,
    .pitch_offset  = -2.0f,
    .max_tilt      = 25.0f,
    .out_limit     = 8.0f,
};

struct state st = {0};
struct debug dbg = {0};

/* ===== 串口命令解析 ===== */
static char cmd_buf[64];
static int cmd_len = 0;

void handle_serial_cmd(void) {
    char *eq = strchr(cmd_buf, '=');
    char *q  = strchr(cmd_buf, '?');

    /* help */
    if (strcmp(cmd_buf, "help") == 0) {
        Serial.println("\n=== Commands ===");
        Serial.println("angle_kp=6.5    set param");
        Serial.println("angle_kp?       query param");
        Serial.println("dump            show all params");
        Serial.println("telem           show telemetry");
        Serial.println("reset           reset controller");
        Serial.println("\nParams: angle_kp, gyro_kp, distance_kp, speed_kp,");
        Serial.println("        yaw_kp, lqr_u_kp, lqr_u_ki, zeropoint_kp,");
        Serial.println("        pitch_offset, max_tilt, out_limit");
        return;
    }

    /* dump */
    if (strcmp(cmd_buf, "dump") == 0) {
        print_params();
        return;
    }

    /* telem */
    if (strcmp(cmd_buf, "telem") == 0) {
        print_telemetry();
        return;
    }

    /* reset */
    if (strcmp(cmd_buf, "reset") == 0) {
        robot_reset();
        Serial.println("Reset done");
        return;
    }

    /* param set/query */
    char key[32] = {0};
    bool is_query = (q && (!eq || q < eq));

    if (is_query) {
        int len = q - cmd_buf;
        memcpy(key, cmd_buf, len);
    } else if (eq) {
        int len = eq - cmd_buf;
        memcpy(key, cmd_buf, len);
    } else {
        Serial.println("Unknown cmd. Type 'help'");
        return;
    }

    float val = eq ? atof(eq + 1) : 0;

    #define PARAM(name) \
        if (strcmp(key, #name) == 0) { \
            if (is_query) Serial.printf("%s=%.4f\n", #name, cfg.name); \
            else { cfg.name = val; Serial.printf("Set %s=%.4f\n", #name, val); } \
            return; \
        }

    PARAM(angle_kp)
    PARAM(gyro_kp)
    PARAM(distance_kp)
    PARAM(speed_kp)
    PARAM(yaw_kp)
    PARAM(lqr_u_kp)
    PARAM(lqr_u_ki)
    PARAM(zeropoint_kp)
    PARAM(pitch_offset)
    PARAM(max_tilt)
    PARAM(out_limit)

    #undef PARAM
    Serial.printf("Unknown param: %s\n", key);
}

void print_params(void) {
    Serial.println("\n=== Params ===");
    Serial.printf("angle_kp=%.2f  gyro_kp=%.4f\n", cfg.angle_kp, cfg.gyro_kp);
    Serial.printf("distance_kp=%.2f  speed_kp=%.4f\n", cfg.distance_kp, cfg.speed_kp);
    Serial.printf("yaw_kp=%.4f  lqr_u_kp=%.2f  lqr_u_ki=%.2f\n", cfg.yaw_kp, cfg.lqr_u_kp, cfg.lqr_u_ki);
    Serial.printf("zeropoint_kp=%.4f  pitch_offset=%.2f\n", cfg.zeropoint_kp, cfg.pitch_offset);
    Serial.printf("max_tilt=%.1f  out_limit=%.2f\n", cfg.max_tilt, cfg.out_limit);
}

void print_telemetry(void) {
    Serial.println("\n=== Telemetry ===");
    Serial.printf("PITCH:%.2fdeg  ROLL:%.2fdeg\n", st.pitch*180/3.1416f, st.roll*180/3.1416f);
    Serial.printf("wL:%.2f  wR:%.2f rad/s  xL:%.1f  xR:%.1f rad\n", st.wL, st.wR, st.xL, st.xR);
    Serial.printf("ANGLE:%.2f  GYRO:%.2f  DIST:%.2f  SPD:%.2f  YAW:%.2f\n",
        dbg.angle_out, dbg.gyro_out, dbg.dist_out, dbg.speed_out, dbg.yaw_out);
    Serial.printf("LQR_U:%.2f  LQR_I:%.2f  COG:%d\n", dbg.lqr_u, dbg.lqr_i, dbg.cog_active);
    Serial.printf("MOTOR_L:%.2fV  MOTOR_R:%.2fV\n", st.motor_L, st.motor_R);
    Serial.printf("FAULT:%d  ENABLED:%d\n", st.fault, st.enabled);
}

/* ===== FreeRTOS 任务 ===== */
void task_imu(void*) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        imu_read();
        vTaskDelayUntil(&last, 5);
    }
}

void task_balance(void*) {
    TickType_t last = xTaskGetTickCount();
    uint32_t prev = micros();
    for (;;) {
        uint32_t now = micros();
        float dt = (now - prev) * 1e-6f;
        if (dt > 0.1f) dt = 0.005f;
        prev = now;

        st.wL = -sensorL.getVelocity();
        st.wR = sensorR.getVelocity();
        st.xL = -sensorL.getAngle();
        st.xR = sensorR.getAngle();
        st.wheel_ok = true;

        balance_step(dt);
        vTaskDelayUntil(&last, 5);
    }
}

void task_foc(void*) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        motorL.loopFOC();
        motorR.loopFOC();
        motorL.move(st.motor_L);
        motorR.move(st.motor_R);
        vTaskDelayUntil(&last, 1);
    }
}

/* ===== Arduino 入口 ===== */
void setup() {
    Serial.begin(115200);

    Wire.begin(PIN_SDA1, PIN_SCL1, 400000);
    Wire2.begin(PIN_SDA2, PIN_SCL2, 400000);

    imu_init();

    sensorL.init(&Wire);
    sensorR.init(&Wire2);

    driverL.voltage_power_supply = cfg.supply_voltage;
    driverR.voltage_power_supply = cfg.supply_voltage;
    driverL.init();
    driverR.init();

    motorL.linkSensor(&sensorL);
    motorR.linkSensor(&sensorR);
    motorL.linkDriver(&driverL);
    motorR.linkDriver(&driverR);
    motorL.controller = MotionControlType::torque;
    motorR.controller = MotionControlType::torque;
    motorL.init();
    motorL.initFOC();
    motorR.init();
    motorR.initFOC();
    motorL.enable();
    motorR.enable();

    st.enabled = true;
    xTaskCreatePinnedToCore(task_imu,     "IMU", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(task_balance, "BAL", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(task_foc,     "FOC", 4096, NULL, 5, NULL, 1);

    Serial.println("Robot ready. Type 'help'");
}

void loop() {
    /* 串口命令处理 */
    while (Serial.available() && cmd_len < 63) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                handle_serial_cmd();
                cmd_len = 0;
            }
        } else {
            cmd_buf[cmd_len++] = c;
        }
    }

    vTaskDelay(10);
}
```

---

## 风格对比

| 特性 | 当前代码 | 重构后 |
|------|----------|--------|
| 类/虚函数 | `BalanceController` 类 | `struct` + 函数 |
| 命名空间 | `wheelsbot::control::` | 无 |
| 接口抽象 | `ImuSensor`, `WheelEncoder` | 直接调用 |
| 状态管理 | `AppContext` 大杂烩 | `cfg` + `st` + `dbg` |
| 调试输出 | 分散在各处 | `print_telemetry()` |
| 代码行数 | ~1500 行 | ~500 行 |

---

## 文件行数估算

| 文件 | 行数 | 职责 |
|------|------|------|
| `robot.h` | ~70 | 数据结构 |
| `balance.c` | ~100 | 控制算法 |
| `imu_mpu6050.c` | ~60 | IMU |
| `main.cpp` | ~200 | 硬件 + 调度 + 串口 |
| `wifi_ctrl.c` | ~120 | WiFi/OTA/WebSocket |
| `pins.h` | ~20 | 引脚定义 |
| **总计** | **~570** | |

---

## 迁移大车时改什么

```
改机械尺寸: cfg.wheel_radius, motor 极对数
改控制参数: cfg.angle_kp, cfg.gyro_kp, ...
换 IMU:     重写 imu_mpu6050.c
换控制算法: 重写 balance_step()
```

---

## Follow-ups

1. [ ] 创建 `robot.h`
2. [ ] 实现 `balance.c`
3. [ ] 实现 `imu_mpu6050.c`
4. [ ] 重写 `main.cpp`
5. [ ] 实现 `wifi_ctrl.c` (WebSocket + OTA)
6. [ ] 删除旧文件
7. [ ] 测试编译运行
