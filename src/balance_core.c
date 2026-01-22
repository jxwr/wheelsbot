#include "balance_core.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// fault flags bitmask
enum {
  BC_FAULT_NONE           = 0,
  BC_FAULT_SENSOR_LOST    = 1u << 0,
  BC_FAULT_TILT_TOO_LARGE = 1u << 1,
  BC_FAULT_BAD_DT         = 1u << 2,
};

struct bc_ctx {
  bc_params_t p;
  bc_debug_t  d;

  float pitch;        // internal state (rad)
  float err_i;        // integral
  float sensor_bad_t; // seconds
};

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float v3_get(const float v[3], uint8_t axis) {
  if (axis > 2) axis = 0;
  return v[axis];
}

// A practical template for pitch_acc:
// - choose one accel axis as "forward/back" projection (main)
// - use sqrt(sum(other_axes^2)) as denominator
static float pitch_from_acc(const bc_params_t* p, float ax, float ay, float az) {
  float a[3] = {ax, ay, az};
  float main = (float)p->pitch_acc_sign * v3_get(a, p->pitch_acc_axis);

  float s = 0.f;
  for (int i = 0; i < 3; ++i) {
    if ((uint8_t)i == p->pitch_acc_axis) continue;
    s += a[i] * a[i];
  }
  float denom = sqrtf(fmaxf(s, 1e-6f));
  return atan2f(main, denom); // rad
}

static float pitch_rate_from_gyro(const bc_params_t* p, float gx, float gy, float gz) {
  float g[3] = {gx, gy, gz};
  float v = v3_get(g, p->pitch_gyro_axis);
  return (float)p->pitch_gyro_sign * v; // rad/s
}

static void set_defaults(bc_params_t* p) {
  memset(p, 0, sizeof(*p));

  p->comp_alpha = 0.98f;

  // Default mapping (you should change to match your IMU installation)
  p->pitch_gyro_axis = 1; // y
  p->pitch_gyro_sign = +1;
  p->pitch_acc_axis  = 0; // x
  p->pitch_acc_sign  = +1;

  p->pitch_target = 0.0f;

  // A workable starting point (still needs tuning for real hardware)
  p->Kp = 35.0f;
  p->Ki = 0.0f;
  p->Kd = 1.2f;
  p->integrator_limit = 2.0f;

  // Command mixing gains
  // If cmd is in physical units (m/s and rad/s),
  // main.c should compute these from wheel radius and track width.
  p->v2speed_gain   = 10.0f;
  p->yaw2diff_gain  = 6.0f;

  // Limits
  p->max_out = 40.0f;
  p->max_tilt = 25.0f * (float)(M_PI / 180.0);
  p->sensor_timeout_s = 0.2f;
}

bc_ctx_t* bc_create(void) {
  bc_ctx_t* ctx = (bc_ctx_t*)calloc(1, sizeof(bc_ctx_t));
  if (!ctx) return NULL;
  set_defaults(&ctx->p);
  ctx->d.state = BC_STATE_DISABLED;
  return ctx;
}

void bc_destroy(bc_ctx_t* ctx) {
  free(ctx);
}

void bc_reset(bc_ctx_t* ctx) {
  if (!ctx) return;
  ctx->pitch = 0.f;
  ctx->err_i = 0.f;
  ctx->sensor_bad_t = 0.f;

  memset(&ctx->d, 0, sizeof(ctx->d));
  ctx->d.state = BC_STATE_DISABLED;
  ctx->d.fault_flags = 0;
}

void bc_set_params(bc_ctx_t* ctx, const bc_params_t* p) {
  if (!ctx || !p) return;
  ctx->p = *p;
}

void bc_get_params(const bc_ctx_t* ctx, bc_params_t* out) {
  if (!ctx || !out) return;
  *out = ctx->p;
}

bc_state_t bc_get_state(const bc_ctx_t* ctx) {
  return ctx ? ctx->d.state : BC_STATE_FAULT;
}

void bc_get_debug(const bc_ctx_t* ctx, bc_debug_t* out) {
  if (!ctx || !out) return;
  *out = ctx->d;
}

bool bc_step(bc_ctx_t* ctx, const bc_input_t* in, bc_output_t* out) {
  if (!ctx || !in || !out) return false;

  // output defaults
  out->mode = BC_OUT_WHEEL_VELOCITY;
  out->left = 0.f;
  out->right= 0.f;
  out->ok   = false;

  float dt = in->time.dt;

  // basic dt sanity
  if (!(dt > 0.f && dt < 0.1f)) { // dt should be in (0, 100ms)
    ctx->d.state = BC_STATE_FAULT;
    ctx->d.fault_flags |= BC_FAULT_BAD_DT;
    return true;
  }

  // disabled: do not drive motors
  if (!in->cmd.enable) {
    ctx->d.state = BC_STATE_DISABLED;
    ctx->d.fault_flags = 0;
    return true;
  }

  // sensor validity accumulation
  if (!in->imu.valid) ctx->sensor_bad_t += dt;
  else                ctx->sensor_bad_t = 0.f;

  if (ctx->sensor_bad_t > ctx->p.sensor_timeout_s) {
    ctx->d.state = BC_STATE_FAULT;
    ctx->d.fault_flags |= BC_FAULT_SENSOR_LOST;
    return true;
  }

  // ========= attitude estimation =========
  float pitch_rate = pitch_rate_from_gyro(&ctx->p, in->imu.gx, in->imu.gy, in->imu.gz);
  float pitch_acc  = pitch_from_acc(&ctx->p, in->imu.ax, in->imu.ay, in->imu.az);

  float pitch_pred = ctx->pitch + pitch_rate * dt;
  ctx->pitch = ctx->p.comp_alpha * pitch_pred + (1.f - ctx->p.comp_alpha) * pitch_acc;

  // safety: too much tilt
  if (fabsf(ctx->pitch) > ctx->p.max_tilt) {
    ctx->d.state = BC_STATE_FAULT;
    ctx->d.fault_flags |= BC_FAULT_TILT_TOO_LARGE;
    return true;
  }

  ctx->d.state = BC_STATE_RUNNING;
  ctx->d.fault_flags = 0;

  // ========= balance outer loop =========
  float err = ctx->p.pitch_target - ctx->pitch;

  ctx->err_i += err * dt;
  ctx->err_i = clampf(ctx->err_i, -ctx->p.integrator_limit, ctx->p.integrator_limit);

  // D term uses -pitch_rate (damping)
  float u_balance = ctx->p.Kp * err + ctx->p.Ki * ctx->err_i + ctx->p.Kd * (-pitch_rate);

  // ========= mixing =========
  float u_total = u_balance + ctx->p.v2speed_gain * in->cmd.v_fwd;
  float diff    = ctx->p.yaw2diff_gain * in->cmd.w_yaw;

  float left  = u_total - diff;
  float right = u_total + diff;

  left  = clampf(left,  -ctx->p.max_out, ctx->p.max_out);
  right = clampf(right, -ctx->p.max_out, ctx->p.max_out);

  out->left = left;
  out->right= right;
  out->ok   = true;

  // ========= debug =========
  ctx->d.pitch = ctx->pitch;
  ctx->d.pitch_rate = pitch_rate;
  ctx->d.err = err;
  ctx->d.err_i = ctx->err_i;
  ctx->d.u_balance = u_balance;
  ctx->d.u_left = left;
  ctx->d.u_right= right;

  return true;
}
