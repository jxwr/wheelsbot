#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

// ============================================================
// Data types (zero Arduino dependency)
// ============================================================

typedef struct { float ax, ay, az; float gx, gy, gz; bool valid; } bc_imu_t;
typedef struct { float wL, wR; bool valid; } bc_wheelsense_t;
typedef struct { float dt; } bc_time_t;

typedef struct {
  float v_fwd;
  float w_yaw;
  bool  enable;
  bool  pitch_inject_enable;
  float pitch_inject_deg;
} bc_user_cmd_t;

typedef struct {
  bc_imu_t        imu;
  bc_wheelsense_t wheel;
  bc_time_t       time;
  bc_user_cmd_t   cmd;
} bc_input_t;

typedef struct { int mode; float left; float right; bool ok; } bc_output_t;

// ============================================================
// Parameters (all tuneable at runtime via WebSocket)
// ============================================================
typedef struct {
  float comp_alpha;
  uint8_t pitch_gyro_axis; int8_t pitch_gyro_sign;
  uint8_t pitch_acc_axis;  int8_t pitch_acc_sign;

  float pitch_target;
  float Kp, Ki, Kd;
  float d_lpf_alpha;        // D-term low-pass filter coefficient (0..1, higher = less filtering)
  float integrator_limit;
  float deadband;            // error deadband (default 0 — disabled)

  float v2speed_gain;
  float yaw2diff_gain;

  float max_out;
  float max_tilt;
  float sensor_timeout_s;
  float ramp_time_s;

  // outer-loop (velocity/position -> tilt command)
  float Kv;
  float Kx;
  float tilt_cmd_max;

  // pitch_ref self-learning
  float bias_learn_k;
  float v_gate;
  float w_gate;
} bc_params_t;

// ============================================================
// State / debug
// ============================================================
typedef enum { BC_STATE_DISABLED=0, BC_STATE_RUNNING=1, BC_STATE_FAULT=2 } bc_state_t;

typedef struct {
  float pitch, pitch_rate;
  float err, err_i;
  float u_balance;
  float u_left, u_right;
  bc_state_t state;
  uint32_t fault_flags;
} bc_debug_t;

enum {
  BC_FAULT_NONE           = 0,
  BC_FAULT_SENSOR_LOST    = 1u << 0,
  BC_FAULT_TILT_TOO_LARGE = 1u << 1,
  BC_FAULT_BAD_DT         = 1u << 2,
};

typedef struct {
  bc_params_t p;
  bc_debug_t  d;
  float pitch;
  float pitch_rate_filt;   // D-term filtered pitch rate
  float err_i;
  float sensor_bad_t;
  float pitch_ref;
  float enable_gain;
  float enable_t;
  bool  was_enabled;
} bc_ctx_t;

// ============================================================
// API
// ============================================================
void bc_set_defaults(bc_params_t* p);
void bc_init(bc_ctx_t* ctx);
void bc_reset(bc_ctx_t* ctx);
void bc_set_params(bc_ctx_t* ctx, const bc_params_t* p);
void bc_get_debug(const bc_ctx_t* ctx, bc_debug_t* out);
bool bc_step(bc_ctx_t* ctx, const bc_input_t* in, bc_output_t* out);

// Utility
static inline float bc_clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
