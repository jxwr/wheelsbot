#include "imu_mpu6050.h"
#include <Arduino.h>

bool mpu_init(TwoWire& wire) {
  wire.beginTransmission(MPU_ADDR);
  wire.write(0x6B);  // PWR_MGMT_1
  wire.write(0x00);  // wake up
  return wire.endTransmission() == 0;
}

bool mpu_read14(TwoWire& wire, ImuRaw& out) {
  wire.beginTransmission(MPU_ADDR);
  wire.write(0x3B); // ACCEL_XOUT_H
  if (wire.endTransmission(false) != 0) return false;

  int n = wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (n != 14) return false;

  auto rd16 = [&]() -> int16_t {
    int16_t hi = wire.read();
    int16_t lo = wire.read();
    return (int16_t)((hi << 8) | lo);
  };

  out.ax = rd16(); out.ay = rd16(); out.az = rd16();
  out.temp = rd16();
  out.gx = rd16(); out.gy = rd16(); out.gz = rd16();
  out.t_us = micros();
  return true;
}
