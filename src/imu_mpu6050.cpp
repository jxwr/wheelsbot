/*
 * imu_mpu6050.cpp - MPU6050 IMU 读取
 *
 * 简化版本，直接操作 I2C，更新全局状态 g_imu。
 */

#include "robot.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

#define MPU_ADDR 0x68

/* 内部状态 */
static TwoWire* imu_wire = nullptr;
static float alpha_ = 0.98f;
static float pitch_ = 0, roll_ = 0, yaw_ = 0;
static uint32_t last_us_ = 0;
static bool initialized_ = false;

/* 缩放因子 */
static constexpr float ACCEL_SCALE = 9.8f / 16384.0f;  /* ±2g */
static constexpr float GYRO_SCALE  = (3.14159f / 180.0f) / 131.0f;  /* ±250dps -> rad/s */

void imu_init(void) {
    imu_wire = &Wire;
    initialized_ = false;
    pitch_ = roll_ = yaw_ = 0;
    last_us_ = micros();

    /* 唤醒 MPU6050 */
    imu_wire->beginTransmission(MPU_ADDR);
    imu_wire->write(0x6B);
    imu_wire->write(0x00);
    if (imu_wire->endTransmission() != 0) {
        Serial.println("IMU: wake up failed");
        return;
    }
    delay(10);

    /* 配置陀螺仪 ±250dps */
    imu_wire->beginTransmission(MPU_ADDR);
    imu_wire->write(0x1B);
    imu_wire->write(0x00);
    if (imu_wire->endTransmission() != 0) {
        Serial.println("IMU: gyro config failed");
        return;
    }

    /* 配置加速度计 ±2g */
    imu_wire->beginTransmission(MPU_ADDR);
    imu_wire->write(0x1C);
    imu_wire->write(0x00);
    if (imu_wire->endTransmission() != 0) {
        Serial.println("IMU: accel config failed");
        return;
    }

    /* 验证 WHO_AM_I */
    imu_wire->beginTransmission(MPU_ADDR);
    imu_wire->write(0x75);
    imu_wire->endTransmission(false);
    imu_wire->requestFrom(MPU_ADDR, 1);
    uint8_t whoami = imu_wire->read();
    Serial.printf("IMU: WHO_AM_I = 0x%02X\n", whoami);

    initialized_ = true;
    Serial.println("IMU: init success");
}

void imu_read(void) {
    if (!initialized_) {
        g_imu.valid = false;
        return;
    }

    /* 读取 14 字节数据 (加速度 + 温度 + 陀螺仪) */
    uint8_t buf[14];
    imu_wire->beginTransmission(MPU_ADDR);
    imu_wire->write(0x3B);
    if (imu_wire->endTransmission(false) != 0) {
        g_imu.valid = false;
        return;
    }

    imu_wire->requestFrom((int)MPU_ADDR, 14);
    for (int i = 0; i < 14; i++) {
        buf[i] = imu_wire->read();
    }

    /* 解析原始数据 */
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    /* buf[6:7] = 温度, 跳过 */
    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    /* 转换单位 */
    g_imu.ax = ax * ACCEL_SCALE;
    g_imu.ay = ay * ACCEL_SCALE;
    g_imu.az = az * ACCEL_SCALE;
    g_imu.gx = gx * GYRO_SCALE;
    g_imu.gy = gy * GYRO_SCALE;
    g_imu.gz = gz * GYRO_SCALE;

    /* 计算时间步长 */
    uint32_t now_us = micros();
    float dt = (now_us - last_us_) * 1e-6f;
    if (!(dt > 0.0f && dt < 0.1f)) dt = 0.005f;
    last_us_ = now_us;

    /* 互补滤波 */
    float pitch_acc = atan2f(-g_imu.ax, sqrtf(g_imu.ay * g_imu.ay + g_imu.az * g_imu.az));
    float roll_acc  = atan2f(g_imu.ay, g_imu.az);

    pitch_ = alpha_ * (pitch_ + g_imu.gy * dt) + (1.0f - alpha_) * pitch_acc;
    roll_  = alpha_ * (roll_  + g_imu.gx * dt) + (1.0f - alpha_) * roll_acc;
    yaw_  += g_imu.gz * dt;

    /* 转换为角度 */
    g_imu.pitch = pitch_ * (180.0f / 3.14159f);
    g_imu.roll  = roll_  * (180.0f / 3.14159f);
    g_imu.yaw   = yaw_   * (180.0f / 3.14159f);

    g_imu.valid = true;
}
