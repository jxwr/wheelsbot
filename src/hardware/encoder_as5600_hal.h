#pragma once

#include "encoder_interface.h"
#include <Wire.h>

namespace wheelsbot {
namespace hardware {

// ============================================================
// AS5600 Magnetic Encoder HAL Implementation
// Supports both I2C bus options with SimpleFOC integration
// ============================================================

class AS5600_HAL : public WheelEncoder {
 public:
  // Constructor: wire = I2C bus, invert = flip direction
  explicit AS5600_HAL(TwoWire& wire = Wire, bool invert = false);

  bool init() override;
  bool read(EncoderData& out) override;
  float getAngle() override;
  float getVelocity() override;
  bool isReady() const override { return initialized_; }
  void reset() override;
  const char* name() const override { return "AS5600"; }

 private:
  TwoWire& wire_;
  static constexpr uint8_t AS5600_ADDR = 0x36;

  bool initialized_ = false;
  bool invert_ = false;

  // State tracking for velocity calculation
  float last_angle_ = 0.0f;
  uint32_t last_us_ = 0;
  float velocity_ = 0.0f;

  // Conversion: 12-bit (4096 steps) to radians
  static constexpr float RAW_TO_RAD = (2.0f * M_PI) / 4096.0f;

  bool readRawAngle(uint16_t& angle);
};

}  // namespace hardware
}  // namespace wheelsbot
