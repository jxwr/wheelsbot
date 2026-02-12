#include "hardware_manager.h"
#include <Arduino.h>
#include <new>

namespace wheelsbot {
namespace hardware {

// Global instance
static HardwareManager g_hardware_manager;

HardwareManager& HardwareManager::instance() {
  return g_hardware_manager;
}

bool HardwareManager::init(TwoWire& wire_imu, TwoWire& wire_left_enc, TwoWire& wire_right_enc) {
  // Reconstruct members with correct I2C buses using placement new
  // This is safe because we control the lifecycle

  new (&imu_) MPU6050_HAL(wire_imu);
  new (&left_encoder_) AS5600_HAL(wire_left_enc, false);
  new (&right_encoder_) AS5600_HAL(wire_right_enc, true);  // Inverted

  bool ok = true;

  // Initialize IMU
  if (!imu_.init()) {
    Serial.println("HW: IMU init failed");
    ok = false;
  } else {
    Serial.println("HW: IMU initialized");
  }

  // Initialize encoders
  if (!left_encoder_.init()) {
    Serial.println("HW: Left encoder init failed");
    ok = false;
  } else {
    Serial.println("HW: Left encoder initialized");
  }

  if (!right_encoder_.init()) {
    Serial.println("HW: Right encoder init failed");
    ok = false;
  } else {
    Serial.println("HW: Right encoder initialized");
  }

  return ok;
}

}  // namespace hardware
}  // namespace wheelsbot
