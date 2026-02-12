#pragma once

#include <stdint.h>

namespace wheelsbot {
namespace hardware {

// ============================================================
// Wheel Encoder Interface
// Abstract base class for all wheel encoder implementations
// ============================================================

struct EncoderData {
  float angle;      // Angular position in radians
  float velocity;   // Angular velocity in rad/s
  int32_t raw;      // Raw sensor value (optional)
  bool valid;       // Data validity flag
};

class WheelEncoder {
 public:
  virtual ~WheelEncoder() = default;

  // Initialize the encoder hardware
  virtual bool init() = 0;

  // Read latest encoder data
  virtual bool read(EncoderData& out) = 0;

  // Get current angle in radians (convenience method)
  virtual float getAngle() = 0;

  // Get current velocity in rad/s (convenience method)
  virtual float getVelocity() = 0;

  // Check if encoder is ready/initialized
  virtual bool isReady() const = 0;

  // Reset encoder position (does not re-initialize hardware)
  virtual void reset() = 0;

  // Get encoder name for diagnostics
  virtual const char* name() const = 0;
};

}  // namespace hardware
}  // namespace wheelsbot
