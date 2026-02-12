#pragma once

#include <Arduino.h>

// ============================================================
// GPIO pins — ESP32-S3-DevKitC-1
// ============================================================

// Left motor (BLDC 3-PWM)
static constexpr gpio_num_t PIN_PWM_U = GPIO_NUM_4;
static constexpr gpio_num_t PIN_PWM_V = GPIO_NUM_5;
static constexpr gpio_num_t PIN_PWM_W = GPIO_NUM_6;
static constexpr gpio_num_t PIN_EN    = GPIO_NUM_7;

// Left I2C bus (MPU6050 + AS5600)
static constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_3;
static constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_9;

// Right motor (BLDC 3-PWM)
static constexpr gpio_num_t PIN_PWM_U2 = GPIO_NUM_35;
static constexpr gpio_num_t PIN_PWM_V2 = GPIO_NUM_36;
static constexpr gpio_num_t PIN_PWM_W2 = GPIO_NUM_37;
static constexpr gpio_num_t PIN_EN2    = GPIO_NUM_38;

// Right I2C bus (AS5600)
static constexpr gpio_num_t PIN_I2C_SDA2 = GPIO_NUM_2;
static constexpr gpio_num_t PIN_I2C_SCL2 = GPIO_NUM_1;

// LED
static constexpr int LED_PIN = GPIO_NUM_21;

// ============================================================
// Motor constants
// ============================================================
static constexpr int   POLE_PAIRS        = 7;
static constexpr float SUPPLY_VOLTAGE    = 12.0f;
static constexpr float VOLTAGE_LIMIT_INIT = 12.0f;

// ============================================================
// Vehicle geometry
// ============================================================
static constexpr float WHEEL_RADIUS_M = 0.035f;  // 70mm diameter
static constexpr float TRACK_WIDTH_M  = 0.18f;   // wheel-to-wheel
