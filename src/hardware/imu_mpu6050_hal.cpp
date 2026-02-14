#include "imu_mpu6050_hal.h"
#include <Arduino.h>

namespace wheelsbot {
namespace hardware {

MPU6050_HAL::MPU6050_HAL(TwoWire& wire, float alpha)
    : wire_(wire), alpha_(alpha) {}

bool MPU6050_HAL::writeRegister(uint8_t reg, uint8_t data) {
  wire_.beginTransmission(MPU_ADDR);
  wire_.write(reg);
  wire_.write(data);
  return wire_.endTransmission() == 0;
}

bool MPU6050_HAL::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
  wire_.beginTransmission(MPU_ADDR);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;

  uint8_t received = wire_.requestFrom((int)MPU_ADDR, (int)len);
  if (received != len) return false;

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = wire_.read();
  }
  return true;
}

bool MPU6050_HAL::init() {
  initialized_ = false;
  pitch_ = roll_ = yaw_ = 0.0f;
  last_us_ = micros();

  Serial.println("IMU: waking up...");
  // Wake up MPU6050
  if (!writeRegister(0x6B, 0x00)) {
    Serial.println("IMU: wake up failed");
    return false;
  }
  delay(10);

  Serial.println("IMU: configuring gyro...");
  // Set gyro full scale range to ±250 dps (0x00)
  if (!writeRegister(0x1B, 0x00)) {
    Serial.println("IMU: gyro config failed");
    return false;
  }

  Serial.println("IMU: configuring accel...");
  // Set accel full scale range to ±2g (0x00)
  if (!writeRegister(0x1C, 0x00)) {
    Serial.println("IMU: accel config failed");
    return false;
  }

  // Verify WHO_AM_I
  uint8_t whoami = 0;
  Serial.println("IMU: reading WHO_AM_I...");
  if (!readRegisters(0x75, &whoami, 1)) {
    Serial.println("IMU: WHO_AM_I read failed");
    return false;
  }
  // 0x68 = MPU6050, 0x70 = MPU6500, 0x71 = MPU9250 — all register-compatible
  Serial.printf("IMU: WHO_AM_I = 0x%02X\n", whoami);
  if (whoami != 0x68 && whoami != 0x70 && whoami != 0x71) {
    Serial.printf("IMU: unexpected WHO_AM_I 0x%02X (want 0x68/0x70/0x71)\n", whoami);
    return false;
  }

  Serial.println("IMU: init success");
  initialized_ = true;
  return true;
}

bool MPU6050_HAL::readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                          int16_t& gx, int16_t& gy, int16_t& gz) {
  uint8_t buf[14];
  if (!readRegisters(0x3B, buf, 14)) return false;

  auto to_int16 = [](uint8_t h, uint8_t l) -> int16_t {
    return (int16_t)((h << 8) | l);
  };

  ax = to_int16(buf[0], buf[1]);
  ay = to_int16(buf[2], buf[3]);
  az = to_int16(buf[4], buf[5]);
  // buf[6:7] = temperature, skip
  gx = to_int16(buf[8], buf[9]);
  gy = to_int16(buf[10], buf[11]);
  gz = to_int16(buf[12], buf[13]);

  return true;
}

bool MPU6050_HAL::read(ImuData& out) {
  if (!initialized_) {
    out.valid = false;
    return false;
  }

  int16_t rax, ray, raz, rgx, rgy, rgz;
  if (!readRaw(rax, ray, raz, rgx, rgy, rgz)) {
    out.valid = false;
    return false;
  }

  // Convert to physical units
  float ax = rax * ACCEL_SCALE;
  float ay = ray * ACCEL_SCALE;
  float az = raz * ACCEL_SCALE;
  float gx = rgx * GYRO_SCALE;
  float gy = rgy * GYRO_SCALE;
  float gz = rgz * GYRO_SCALE;

  // Calculate dt
  uint32_t now_us = micros();
  float dt = (now_us - last_us_) * 1e-6f;
  if (!(dt > 0.0f && dt < 0.1f)) dt = 0.005f;
  last_us_ = now_us;

  // Complementary filter for pitch and roll
  // Accel angles
  float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));
  float roll_acc = atan2f(ay, az);

  // Gyro integration
  float pitch_gyro = pitch_ + gy * dt;
  float roll_gyro = roll_ + gx * dt;
  float yaw_gyro = yaw_ + gz * dt;

  // Complementary filter
  pitch_ = alpha_ * pitch_gyro + (1.0f - alpha_) * pitch_acc;
  roll_ = alpha_ * roll_gyro + (1.0f - alpha_) * roll_acc;
  yaw_ = yaw_gyro;

  // Fill output
  out.ax = ax;
  out.ay = ay;
  out.az = az;
  out.gx = gx;
  out.gy = gy;
  out.gz = gz;
  // Convert to degrees (M_PI is available in Arduino, cast to float)
  out.pitch = pitch_ * (180.0f / (float)M_PI);
  out.roll = roll_ * (180.0f / (float)M_PI);
  out.yaw = yaw_ * (180.0f / (float)M_PI);
  out.timestamp_us = now_us;
  out.valid = true;

  return true;
}

void MPU6050_HAL::reset() {
  pitch_ = roll_ = yaw_ = 0.0f;
  last_us_ = micros();
}

}  // namespace hardware
}  // namespace wheelsbot
