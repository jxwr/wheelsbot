#include "hardware_manager.h"
#include <Arduino.h>

namespace wheelsbot {
namespace hardware {

HardwareManager::HardwareManager(ImuSensor& imu, WheelEncoder& left_enc, WheelEncoder& right_enc)
    : imu_(&imu), left_enc_(&left_enc), right_enc_(&right_enc) {}

bool HardwareManager::initImu() {
  if (!imu_->init()) {
    Serial.println("HW: IMU init failed");
    return false;
  }
  Serial.println("HW: IMU initialized");
  return true;
}

}  // namespace hardware
}  // namespace wheelsbot
