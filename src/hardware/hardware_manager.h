#pragma once

#include "imu_interface.h"
#include "encoder_interface.h"
#include "imu_mpu6050_hal.h"
#include "encoder_as5600_hal.h"
#include <Wire.h>

namespace wheelsbot {
namespace hardware {

// ============================================================
// Hardware Manager
// Centralized hardware lifecycle management
// Uses static storage (no heap allocation) for embedded safety
// ============================================================

class HardwareManager {
 public:
  // Initialize all hardware with I2C buses
  bool init(TwoWire& wire_imu, TwoWire& wire_left_enc, TwoWire& wire_right_enc);

  // Get sensor instances
  ImuSensor* imu() { return &imu_; }
  WheelEncoder* leftEncoder() { return &left_encoder_; }
  WheelEncoder* rightEncoder() { return &right_encoder_; }

  // Status
  bool isImuReady() const { return imu_.isReady(); }
  bool isLeftEncoderReady() const { return left_encoder_.isReady(); }
  bool isRightEncoderReady() const { return right_encoder_.isReady(); }
  bool allReady() const { return isImuReady() && isLeftEncoderReady() && isRightEncoderReady(); }

  // Reset individual components
  void resetImu() { imu_.reset(); }
  void resetEncoders() {
    left_encoder_.reset();
    right_encoder_.reset();
  }

  // Static accessor for global instance
  static HardwareManager& instance();

 private:
  HardwareManager() = default;
  ~HardwareManager() = default;
  HardwareManager(const HardwareManager&) = delete;
  HardwareManager& operator=(const HardwareManager&) = delete;

  // Static storage (no heap allocation)
  MPU6050_HAL imu_{Wire};  // Placeholder, reassigned in init
  AS5600_HAL left_encoder_{Wire};
  AS5600_HAL right_encoder_{Wire, true};  // Right encoder inverted
};

}  // namespace hardware
}  // namespace wheelsbot
