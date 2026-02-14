#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <SimpleFOC.h>

#include "pins.h"
#include "shared_state.h"
#include "hardware/imu_mpu6050_hal.h"
#include "hardware/simplefoc_sensor_adapter.h"
#include "hardware/hardware_manager.h"
#include "control/balance_controller.h"

using namespace wheelsbot::hardware;
using namespace wheelsbot::control;

// ============================================================
// AppContext
// Central struct holding all hardware, control, and shared state.
// Constructed once in setup() after I2C buses are initialized.
// Passed to FreeRTOS tasks via pvParameter — no globals needed.
// ============================================================

struct AppContext {
  // --- Concrete hardware (value semantics, static allocation) ---
  MPU6050_HAL imu;
  MagneticSensorI2C lsensor{AS5600_I2C};
  MagneticSensorI2C rsensor{AS5600_I2C};
  SimpleFOCSensorAdapter left_enc;
  SimpleFOCSensorAdapter right_enc;   // inverted

  BLDCMotor lmotor{POLE_PAIRS};
  BLDCMotor rmotor{POLE_PAIRS};
  BLDCDriver3PWM ldriver{PIN_PWM_U, PIN_PWM_V, PIN_PWM_W, PIN_EN};
  BLDCDriver3PWM rdriver{PIN_PWM_U2, PIN_PWM_V2, PIN_PWM_W2, PIN_EN2};

  // --- Hardware abstraction ---
  HardwareManager hw;

  // --- Control ---
  BalanceController balance;

  // --- I2C bus mutex (protects multi-step Wire transactions) ---
  SemaphoreHandle_t wire_mutex = nullptr;

  // --- Shared state (volatile, task-safe) ---
  ImuShared imu_state{};
  BalanceShared bal_state{};
  WheelShared wheel_state{};
  CommandShared cmd_state{};

  // --- Joystick command state ---
  volatile float target_linear_vel = 0.0f; // Target linear velocity from joystick (m/s)
  volatile float target_yaw_rate = 0.0f;   // Target yaw rate for differential drive (rad/s)

  // Explicit constructor — only call after Wire.begin()
  AppContext(TwoWire& wire_imu, TwoWire& /*wire2 — reserved for future use*/)
      : imu(wire_imu),
        left_enc(lsensor, false),
        right_enc(rsensor, true),
        hw(imu, left_enc, right_enc)
  {
    wire_mutex = xSemaphoreCreateMutex();
  }

  // No copy/move
  AppContext(const AppContext&) = delete;
  AppContext& operator=(const AppContext&) = delete;
};
