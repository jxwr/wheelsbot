#pragma once

#include "imu_interface.h"
#include <Wire.h>

namespace wheelsbot {
namespace hardware {

// ============================================================
// MPU6050 HAL Implementation
// Supports both I2C bus options with complementary filter
// ============================================================

class MPU6050_HAL : public ImuSensor {
 public:
  // Constructor: wire = I2C bus, alpha = complementary filter coefficient
  explicit MPU6050_HAL(TwoWire& wire = Wire, float alpha = 0.98f);

  bool init() override;
  bool read(ImuData& out) override;
  bool isReady() const override { return initialized_; }
  void reset() override;
  const char* name() const override { return "MPU6050"; }
  void setComplementaryAlpha(float alpha) override { alpha_ = alpha; }

 private:
  TwoWire& wire_;
  static constexpr uint8_t MPU_ADDR = 0x68;

  bool initialized_ = false;
  float alpha_ = 0.98f;  // Complementary filter: gyro weight

  // State for complementary filter
  float pitch_ = 0.0f;
  float roll_ = 0.0f;
  float yaw_ = 0.0f;
  uint32_t last_us_ = 0;

  // Raw to physical units conversion
  static constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;  // m/s^2 per LSB
  static constexpr float GYRO_SCALE = (M_PI / 180.0f) / 131.0f;  // rad/s per LSB

  bool writeRegister(uint8_t reg, uint8_t data);
  bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
  bool readRaw(int16_t& ax, int16_t& ay, int16_t& az,
               int16_t& gx, int16_t& gy, int16_t& gz);
};

}  // namespace hardware
}  // namespace wheelsbot
