#pragma once

#include "imu_interface.h"
#include "encoder_interface.h"

namespace wheelsbot {
namespace hardware {

// ============================================================
// Hardware Manager
// Centralized hardware lifecycle management
// Dependency-injected: receives sensor references at construction
// No singleton, no static storage, no placement new
// ============================================================

class HardwareManager {
 public:
  // DI constructor: all sensors owned externally (e.g., by AppContext)
  HardwareManager(ImuSensor& imu, WheelEncoder& left_enc, WheelEncoder& right_enc);

  // Initialize IMU only (encoders are inited by SimpleFOC later)
  bool initImu();

  // Get sensor instances (non-owning)
  ImuSensor* imu() { return imu_; }
  WheelEncoder* leftEncoder() { return left_enc_; }
  WheelEncoder* rightEncoder() { return right_enc_; }

  // Status
  bool isImuReady() const { return imu_->isReady(); }
  bool isLeftEncoderReady() const { return left_enc_->isReady(); }
  bool isRightEncoderReady() const { return right_enc_->isReady(); }

  // Prevent copying
  HardwareManager(const HardwareManager&) = delete;
  HardwareManager& operator=(const HardwareManager&) = delete;

 private:
  ImuSensor* imu_;
  WheelEncoder* left_enc_;
  WheelEncoder* right_enc_;
};

}  // namespace hardware
}  // namespace wheelsbot
