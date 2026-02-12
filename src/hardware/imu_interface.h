#pragma once

#include <stdint.h>

namespace wheelsbot {
namespace hardware {

// ============================================================
// IMU Sensor Interface
// Abstract base class for all IMU implementations
// ============================================================

struct ImuData {
  float ax, ay, az;      // Acceleration in m/s^2
  float gx, gy, gz;      // Gyroscope in rad/s
  float roll, pitch, yaw; // Orientation in degrees (optional, may be 0)
  uint32_t timestamp_us; // Microsecond timestamp
  bool valid;            // Data validity flag
};

class ImuSensor {
 public:
  virtual ~ImuSensor() = default;

  // Initialize the IMU hardware
  virtual bool init() = 0;

  // Read latest sensor data
  virtual bool read(ImuData& out) = 0;

  // Check if sensor is ready/initialized
  virtual bool isReady() const = 0;

  // Reset sensor state (does not re-initialize hardware)
  virtual void reset() = 0;

  // Get sensor name for diagnostics
  virtual const char* name() const = 0;

  // Optional: set complementary filter alpha (if supported by implementation)
  virtual void setComplementaryAlpha(float alpha) {}
};

}  // namespace hardware
}  // namespace wheelsbot
