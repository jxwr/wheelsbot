#include "balance_core.h"

// ============================================================
// Internal helpers
// ============================================================

static inline float bc_v3_get(const float v[3], uint8_t axis) {
  if (axis > 2) axis = 0;
  return v[axis];
}

static float bc_pitch_from_acc(const bc_params_t* p, float ax, float ay, float az) {
  float a[3] = {ax, ay, az};
  float main_val = (float)p->pitch_acc_sign * bc_v3_get(a, p->pitch_acc_axis);
  float s = 0.f;
  for (int i = 0; i < 3; ++i) {
    if ((uint8_t)i == p->pitch_acc_axis) continue;
    s += a[i] * a[i];
  }
  float denom = sqrtf(fmaxf(s, 1e-6f));
  return atan2f(main_val, denom);
}

static float bc_pitch_rate_from_gyro(const bc_params_t* p, float gx, float gy, float gz) {
  float g[3] = {gx, gy, gz};
  return (float)p->pitch_gyro_sign * bc_v3_get(g, p->pitch_gyro_axis);
}

// ============================================================
// Defaults / init / reset
// ============================================================

void bc_set_defaults(bc_params_t* p) {
  memset(p, 0, sizeof(*p));
  p->comp_alpha = 0.98f;

  p->pitch_acc_axis  = 0;   // x
  p->pitch_acc_sign  = -1;  // -ax
  p->pitch_gyro_axis = 1;   // y
  p->pitch_gyro_sign = +1;

  p->pitch_target = 0.0f;

  p->Kp = 6.0f;
  p->Ki = 0.0f;
  p->Kd = 0.6f;
  p->d_lpf_alpha = 0.7f;     // D-term LPF: 70% new, 30% old
  p->integrator_limit = 2.0f;
  p->deadband = 0.0f;        // disabled by default (avoids limit-cycle oscillation)

  // v2speed_gain = 1/WHEEL_RADIUS_M, yaw2diff_gain = (TRACK_WIDTH_M*0.5)/WHEEL_RADIUS_M
  p->v2speed_gain  = 1.0f / 0.035f;           // ~28.57
  p->yaw2diff_gain = (0.18f * 0.5f) / 0.035f; // ~2.57

  p->max_out = 6.0f;         // bug fix: was VOLTAGE_LIMIT_INIT (8V), too high for safe tuning
  p->max_tilt = 35.0f * (float)(M_PI / 180.0);
  p->sensor_timeout_s = 0.2f;
  p->ramp_time_s = 0.5f;

  // outer-loop: default OFF for safe inner-loop-only tuning
  p->Kv = 0.0f;
  p->Kx = 0.0f;
  p->tilt_cmd_max = 0.30f;

  // pitch_ref self-learning: default OFF
  p->bias_learn_k = 0.0f;
  p->v_gate = 0.25f;
  p->w_gate = 0.15f;
}

void bc_init(bc_ctx_t* ctx) {
  memset(ctx, 0, sizeof(*ctx));
  bc_set_defaults(&ctx->p);
  ctx->d.state = BC_STATE_DISABLED;
}

void bc_reset(bc_ctx_t* ctx) {
  ctx->pitch = 0.f;
  ctx->pitch_rate_filt = 0.f;
  ctx->err_i = 0.f;
  ctx->sensor_bad_t = 0.f;
  ctx->pitch_ref = 0.f;
  ctx->enable_gain = 0.f;
  ctx->enable_t = 0.f;
  ctx->was_enabled = false;
  memset(&ctx->d, 0, sizeof(ctx->d));
  ctx->d.state = BC_STATE_DISABLED;
  ctx->d.fault_flags = 0;
}

void bc_set_params(bc_ctx_t* ctx, const bc_params_t* p) { ctx->p = *p; }
void bc_get_debug(const bc_ctx_t* ctx, bc_debug_t* out) { *out = ctx->d; }

// ============================================================
// bc_step — main control loop tick
// ============================================================
bool bc_step(bc_ctx_t* ctx, const bc_input_t* in, bc_output_t* out) {
  out->mode  = 0;
  out->left  = 0.f;
  out->right = 0.f;
  out->ok    = false;

  float dt = in->time.dt;
  if (!(dt > 0.f && dt < 0.1f)) {
    ctx->d.state = BC_STATE_FAULT;
    ctx->d.fault_flags |= BC_FAULT_BAD_DT;
    ctx->was_enabled = false;
    return true;
  }

  bool en = in->cmd.enable;
  if (!en) {
    ctx->d.state = BC_STATE_DISABLED;
    ctx->d.fault_flags = 0;
    ctx->was_enabled  = false;
    ctx->enable_gain  = 0.f;
    ctx->enable_t     = 0.f;
    ctx->err_i        = 0.f;
    return true;
  }

  // sensor validity
  if (!in->imu.valid) ctx->sensor_bad_t += dt;
  else                ctx->sensor_bad_t = 0.f;

  if (ctx->sensor_bad_t > ctx->p.sensor_timeout_s) {
    ctx->d.state = BC_STATE_FAULT;
    ctx->d.fault_flags |= BC_FAULT_SENSOR_LOST;
    ctx->was_enabled = false;
    return true;
  }

  // complementary filter
  float pitch_rate = bc_pitch_rate_from_gyro(&ctx->p, in->imu.gx, in->imu.gy, in->imu.gz);
  float pitch_acc  = bc_pitch_from_acc(&ctx->p, in->imu.ax, in->imu.ay, in->imu.az);

  float pitch_pred = ctx->pitch + pitch_rate * dt;
  ctx->pitch = ctx->p.comp_alpha * pitch_pred + (1.f - ctx->p.comp_alpha) * pitch_acc;

  // D-term low-pass filter on pitch_rate
  float alpha = bc_clampf(ctx->p.d_lpf_alpha, 0.f, 1.f);
  ctx->pitch_rate_filt = alpha * pitch_rate + (1.f - alpha) * ctx->pitch_rate_filt;

  // pitch injection mode (for direction/linkage testing)
  if (in->cmd.pitch_inject_enable) {
    ctx->err_i = 0.f;
    ctx->pitch = in->cmd.pitch_inject_deg * (float)(M_PI / 180.0);
    pitch_rate = 0.f;
    ctx->pitch_rate_filt = 0.f;
  }

  // tilt protection
  if (fabsf(ctx->pitch) > ctx->p.max_tilt) {
    ctx->d.state = BC_STATE_FAULT;
    ctx->d.fault_flags |= BC_FAULT_TILT_TOO_LARGE;
    ctx->was_enabled = false;
    return true;
  }

  // enable rising edge: calibrate pitch_ref + clear integrator
  if (en && !ctx->was_enabled) {
    ctx->err_i = 0.f;
    ctx->enable_t = 0.f;
    ctx->enable_gain = 0.f;
    ctx->pitch_ref = ctx->pitch;
  }

  ctx->d.state = BC_STATE_RUNNING;
  ctx->d.fault_flags = 0;

  // outer-loop: wheel speed -> tilt command
  float v = 0.f;
  if (in->wheel.valid) {
    v = 0.5f * (in->wheel.wL + in->wheel.wR);
  }

  float theta_cmd = 0.f;
  if (in->wheel.valid) {
    theta_cmd = -ctx->p.Kv * v;
    theta_cmd = bc_clampf(theta_cmd, -ctx->p.tilt_cmd_max, ctx->p.tilt_cmd_max);
  }

  // pitch_ref self-learning
  if (in->wheel.valid && ctx->p.bias_learn_k != 0.f) {
    if (fabsf(v) < ctx->p.v_gate && fabsf(pitch_rate) < ctx->p.w_gate) {
      ctx->pitch_ref += ctx->p.bias_learn_k * v * dt;
      float lim = 25.f * (float)(M_PI / 180.0);
      ctx->pitch_ref = bc_clampf(ctx->pitch_ref, -lim, +lim);
    }
  }

  // error
  float target = ctx->pitch_ref + ctx->p.pitch_target + theta_cmd;
  float err = target - ctx->pitch;

  // deadband (default 0 = disabled)
  if (ctx->p.deadband > 0.f && fabsf(err) < ctx->p.deadband) {
    err = 0.f;
  }

  // integrator
  if (ctx->p.Ki != 0.f && !in->cmd.pitch_inject_enable) {
    ctx->err_i += err * dt;
    ctx->err_i = bc_clampf(ctx->err_i, -ctx->p.integrator_limit, ctx->p.integrator_limit);
  } else {
    ctx->err_i = 0.f;
  }

  // PID (D term uses filtered pitch_rate for noise rejection)
  float u_balance = ctx->p.Kp * err
                  + ctx->p.Ki * ctx->err_i
                  + ctx->p.Kd * (-ctx->pitch_rate_filt);

  // ramp-in
  ctx->enable_t += dt;
  float g = 1.f;
  if (ctx->p.ramp_time_s > 1e-3f) {
    g = bc_clampf(ctx->enable_t / ctx->p.ramp_time_s, 0.f, 1.f);
  }
  ctx->enable_gain = g;
  u_balance *= g;

  // forward/yaw mixing
  float u_total = u_balance + ctx->p.v2speed_gain * in->cmd.v_fwd;
  float diff    = ctx->p.yaw2diff_gain * in->cmd.w_yaw;

  float left  = bc_clampf(u_total - diff, -ctx->p.max_out, ctx->p.max_out);
  float right = bc_clampf(u_total + diff, -ctx->p.max_out, ctx->p.max_out);

  out->left  = left;
  out->right = right;
  out->ok    = true;

  // debug snapshot
  ctx->d.pitch      = ctx->pitch;
  ctx->d.pitch_rate = pitch_rate;
  ctx->d.err        = err;
  ctx->d.err_i      = ctx->err_i;
  ctx->d.u_balance  = u_balance;
  ctx->d.u_left     = left;
  ctx->d.u_right    = right;

  ctx->was_enabled = en;
  return true;
}
