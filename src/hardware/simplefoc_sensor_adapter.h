#pragma once

#include "encoder_interface.h"
#include <SimpleFOC.h>

namespace wheelsbot {
namespace hardware {

// ============================================================
// SimpleFOC Sensor Adapter
// Wraps a SimpleFOC MagneticSensorI2C as a WheelEncoder interface.
// Does NOT independently perform I2C — delegates to SimpleFOC sensor.
// ============================================================

class SimpleFOCSensorAdapter : public WheelEncoder {
 public:
  // sensor: SimpleFOC MagneticSensorI2C (owned externally)
  // invert: flip sign of angle/velocity
  SimpleFOCSensorAdapter(MagneticSensorI2C& sensor, bool invert = false)
      : sensor_(sensor), invert_(invert) {}

  bool init() override {
    initialized_ = true;
    return true;  // SimpleFOC owns actual sensor init
  }

  bool read(EncoderData& out) override {
    if (!initialized_) {
      out.valid = false;
      return false;
    }
    out.angle = getAngle();
    out.velocity = getVelocity();
    out.raw = 0;
    out.valid = true;
    return true;
  }

  float getAngle() override {
    float a = sensor_.getAngle();
    return invert_ ? -a : a;
  }

  float getVelocity() override {
    float v = sensor_.getVelocity();
    return invert_ ? -v : v;
  }

  bool isReady() const override { return initialized_; }

  void reset() override {
    // No-op: SimpleFOC manages sensor state
  }

  const char* name() const override { return "SimpleFOCSensorAdapter"; }

 private:
  MagneticSensorI2C& sensor_;
  bool invert_ = false;
  bool initialized_ = false;
};

}  // namespace hardware
}  // namespace wheelsbot
