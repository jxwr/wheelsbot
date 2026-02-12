#pragma once

#include <stdint.h>
#include "balance_core.h"

// ============================================================
// Shared state structures (single-writer / single-reader safe)
// ============================================================

// imuTask writes, balanceTask/wifiTask reads
struct ImuShared {
  volatile bool  valid      = false;
  volatile float ax = 0, ay = 0, az = 0;   // m/s^2
  volatile float gx = 0, gy = 0, gz = 0;   // rad/s
  volatile float pitch_deg  = 0;
  volatile float roll_deg   = 0;
  volatile float yaw_deg    = 0;
};

// balanceTask writes, focTask reads
struct BalanceShared {
  volatile float left_target  = 0;
  volatile float right_target = 0;
  volatile bool  ok           = false;
};

// focTask writes, balanceTask reads
struct WheelShared {
  volatile bool  valid = false;
  volatile float wL = 0, wR = 0;   // rad/s
  volatile float xL = 0, xR = 0;   // rad (angle)
};

// wifiTask writes, various tasks read
struct CommandShared {
  volatile bool  balance_enable  = true;
  volatile bool  motor_enable    = true;

  // FOC mode requests
  volatile int   req_motion_mode = 0;   // 0=torque, 1=velocity, 2=angle
  volatile int   req_torque_mode = 0;   // 0=voltage, 2=foc_current
  volatile bool  req_apply_mode  = true;

  volatile float manual_target   = 0.0f;

  volatile float req_vlimit      = 0.0f;
  volatile float req_tlimit      = 8.0f;  // VOLTAGE_LIMIT_INIT
  volatile bool  req_apply_limits = true;

  volatile bool  req_apply_pid   = false;
  volatile float req_pid_p       = 0.2f;
  volatile float req_pid_i       = 20.0f;
  volatile float req_pid_d       = 0.001f;
};

// ============================================================
// Global instances (defined in main.cpp)
// ============================================================
extern ImuShared      g_imu;
extern BalanceShared  g_bal;
extern WheelShared    g_wheel;
extern CommandShared  g_cmd;
extern bc_ctx_t       g_bc;
