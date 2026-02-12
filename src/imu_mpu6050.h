#pragma once

#include <Wire.h>
#include <stdint.h>

static const uint8_t MPU_ADDR = 0x68; // AD0=0

struct ImuRaw {
  int16_t ax, ay, az;
  int16_t temp;
  int16_t gx, gy, gz;
  uint32_t t_us;
};

bool mpu_init(TwoWire& wire = Wire);
bool mpu_read14(TwoWire& wire, ImuRaw& out);
