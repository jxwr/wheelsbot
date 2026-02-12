#include "encoder_as5600_hal.h"
#include <Arduino.h>

namespace wheelsbot {
namespace hardware {

AS5600_HAL::AS5600_HAL(TwoWire& wire, bool invert)
    : wire_(wire), invert_(invert) {}

bool AS5600_HAL::init() {
  initialized_ = false;
  last_angle_ = 0.0f;
  velocity_ = 0.0f;
  last_us_ = micros();

  // Check if AS5600 is present
  wire_.beginTransmission(AS5600_ADDR);
  uint8_t error = wire_.endTransmission();
  if (error != 0) {
    return false;
  }

  // Read initial angle
  uint16_t raw;
  if (!readRawAngle(raw)) {
    return false;
  }

  last_angle_ = raw * RAW_TO_RAD;
  if (invert_) last_angle_ = -last_angle_;
  initialized_ = true;
  return true;
}

bool AS5600_HAL::readRawAngle(uint16_t& angle) {
  // Read 12-bit angle from registers 0x0E (MSB) and 0x0F (LSB)
  wire_.beginTransmission(AS5600_ADDR);
  wire_.write(0x0E);
  if (wire_.endTransmission(false) != 0) return false;

  uint8_t received = wire_.requestFrom((int)AS5600_ADDR, 2);
  if (received != 2) return false;

  uint8_t msb = wire_.read();
  uint8_t lsb = wire_.read();

  angle = ((msb << 8) | lsb) & 0x0FFF;  // 12-bit value
  return true;
}

bool AS5600_HAL::read(EncoderData& out) {
  if (!initialized_) {
    out.valid = false;
    return false;
  }

  uint16_t raw;
  if (!readRawAngle(raw)) {
    out.valid = false;
    return false;
  }

  // Convert to radians
  float angle = raw * RAW_TO_RAD;
  if (invert_) angle = -angle;

  // Calculate velocity
  uint32_t now_us = micros();
  float dt = (now_us - last_us_) * 1e-6f;

  // Handle wrap-around for velocity calculation
  float delta = angle - last_angle_;
  if (delta > M_PI) delta -= 2.0f * M_PI;
  if (delta < -M_PI) delta += 2.0f * M_PI;

  if (dt > 0.0f && dt < 0.1f) {
    velocity_ = delta / dt;
  }

  last_angle_ = angle;
  last_us_ = now_us;

  out.angle = angle;
  out.velocity = velocity_;
  out.raw = raw;
  out.valid = true;

  return true;
}

float AS5600_HAL::getAngle() {
  if (!initialized_) return 0.0f;

  uint16_t raw;
  if (!readRawAngle(raw)) return last_angle_;

  float angle = raw * RAW_TO_RAD;
  if (invert_) angle = -angle;

  // Update internal state for continuity
  last_angle_ = angle;
  last_us_ = micros();

  return angle;
}

float AS5600_HAL::getVelocity() {
  if (!initialized_) return 0.0f;

  // Trigger a read to update velocity
  EncoderData data;
  read(data);
  return velocity_;
}

void AS5600_HAL::reset() {
  last_angle_ = 0.0f;
  velocity_ = 0.0f;
  last_us_ = micros();
}

}  // namespace hardware
}  // namespace wheelsbot
