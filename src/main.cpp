#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <math.h>

/*
  目标：
  1) 保留 IMU,pitch,roll,yaw (deg) 输出 -> 网页 3D 渲染
  2) 处理网页发来的命令：
     - ME1/ME0      (motor enable/disable)
     - BE1/BE0      (balance enable/disable)
     - BC?          (读 balance_core 参数，输出 BCP,...)
     - BS?          (读 balance 状态，输出 BAL,pitch=...,pr=...,uL=...,state=...,ok=...)
     - BC <Key> <V> (写 balance_core 参数)
     - MC0/1/2      (SimpleFOC motion: torque/velocity/angle)
     - MT0/2        (torque mode: voltage/current)
     - M...         (设置目标/限幅，兼容你网页 slider 的 autoSend 形式)
     - M P/I/D <v>  (设置速度 PID 参数)
     - ?            (help)
  3) 目前只有一个电机：假设两轮速度一致。balance 输出用 uL 作为单电机 target。

  线程模型：
  - imuTask: 200Hz 读 MPU6050，做互补滤波，50Hz 打印 IMU 行；同时写入最新 IMU scaled 给 balanceTask
  - balanceTask: 200Hz 调 bc_step()，更新 g_balance_target (rad/s) & 打印 BAL (默认 10Hz，可改)
  - focTask: 1kHz loopFOC + move()，根据 enable/模式/手动or平衡目标选择 motor.target
  - cmdTask: 100Hz 解析串口行命令，只修改“请求变量”；真正应用到 motor 的动作在 focTask 内执行（避免并发踩坑）
*/

// ============================================================
// Pin config for ESP32-S3-DevKitC-1
// ============================================================
static constexpr gpio_num_t PIN_PWM_U = GPIO_NUM_4;
static constexpr gpio_num_t PIN_PWM_V = GPIO_NUM_5;
static constexpr gpio_num_t PIN_PWM_W = GPIO_NUM_6;
static constexpr gpio_num_t PIN_EN    = GPIO_NUM_7;

static constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_3;
static constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_9;

static constexpr gpio_num_t PIN_PWM_U2 = GPIO_NUM_35;
static constexpr gpio_num_t PIN_PWM_V2 = GPIO_NUM_36;
static constexpr gpio_num_t PIN_PWM_W2 = GPIO_NUM_37;
static constexpr gpio_num_t PIN_EN2    = GPIO_NUM_38;

static constexpr gpio_num_t PIN_I2C_SDA2 = GPIO_NUM_2;
static constexpr gpio_num_t PIN_I2C_SCL2 = GPIO_NUM_1;

//static constexpr gpio_num_t PIN_I2C_IMU_SDA = GPIO_NUM_2;
//static constexpr gpio_num_t PIN_I2C_IMU_SCL = GPIO_NUM_1;

static constexpr int LED_PIN = GPIO_NUM_21;

// ============================================================
// Motor config
// ============================================================
static constexpr int   POLE_PAIRS     = 7;
static constexpr float SUPPLY_VOLTAGE = 12.0f;
static constexpr float VOLTAGE_LIMIT_INIT  = 2.0f;

// 车体几何（用于 v_fwd / w_yaw 映射，网页暂未用到，但参数里保留）
static constexpr float WHEEL_RADIUS_M = 0.035f; // 70mm 直径 -> R=0.035m
static constexpr float TRACK_WIDTH_M  = 0.18f;  // 轮距 0.18m

TwoWire Wire2 = TwoWire(1);

// ============================================================
// MPU6050 config
// ============================================================
static const uint8_t MPU_ADDR = 0x68; // AD0=0

struct ImuRaw {
  int16_t ax, ay, az;
  int16_t temp;
  int16_t gx, gy, gz;
  uint32_t t_us;
};

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuRead14(ImuRaw &out) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;

  int n = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (n != 14) return false;

  auto rd16 = [&]() -> int16_t {
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    return (int16_t)((hi << 8) | lo);
  };

  out.ax = rd16(); out.ay = rd16(); out.az = rd16();
  out.temp = rd16();
  out.gx = rd16(); out.gy = rd16(); out.gz = rd16();
  out.t_us = micros();
  return true;
}

// ============================================================
// balance_core (融合版，无malloc)
// ============================================================

typedef struct { float ax, ay, az; float gx, gy, gz; bool valid; } bc_imu_t;
typedef struct { float wL, wR; bool valid; } bc_wheelsense_t;
typedef struct { float dt; } bc_time_t;
typedef struct { float v_fwd; float w_yaw; bool enable; } bc_user_cmd_t;
typedef struct { bc_imu_t imu; bc_wheelsense_t wheel; bc_time_t time; bc_user_cmd_t cmd; } bc_input_t;

typedef struct { int mode; float left; float right; bool ok; } bc_output_t;

typedef struct {
  float comp_alpha;
  uint8_t pitch_gyro_axis; int8_t pitch_gyro_sign;
  uint8_t pitch_acc_axis;  int8_t pitch_acc_sign;

  float pitch_target;
  float Kp, Ki, Kd;
  float integrator_limit;

  float v2speed_gain;
  float yaw2diff_gain;

  float max_out;
  float max_tilt;
  float sensor_timeout_s;

  float deadband;         // rad，误差小于它就当 0
  float ramp_time_s;      // s，enable_gain 从 0->1 用多久

    // ===== outer-loop (keep v/x -> 0) =====
  float Kv;             // (rad / (rad/s))  velocity -> tilt
  float Kx;             // (rad / rad)      position -> tilt
  float tilt_cmd_max;   // (rad)            clamp for theta_cmd

  // ===== bias / pitch_ref self-learning =====
  float bias_learn_k;   // (rad / (rad/s*s))  pitch_ref += k * v * dt  (very small)
  float v_gate;         // (rad/s) only learn when |v| small
  float w_gate;         // (rad/s) only learn when |pitch_rate| small
} bc_params_t;

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
  float err_i;
  float sensor_bad_t;
  float pitch_ref;        // enable 时的参考姿态（rad）
  float enable_gain;      // 0->1 渐进接管
  float enable_t;         // enable 已持续时间（s）
  bool  was_enabled;  
} bc_ctx_t;

static inline float bc_clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
static inline float bc_v3_get(const float v[3], uint8_t axis) {
  if (axis > 2) axis = 0;
  return v[axis];
}
static float bc_pitch_from_acc(const bc_params_t* p, float ax, float ay, float az) {
  float a[3] = {ax, ay, az};
  float main = (float)p->pitch_acc_sign * bc_v3_get(a, p->pitch_acc_axis);
  float s = 0.f;
  for (int i=0;i<3;++i) {
    if ((uint8_t)i == p->pitch_acc_axis) continue;
    s += a[i]*a[i];
  }
  float denom = sqrtf(fmaxf(s, 1e-6f));
  return atan2f(main, denom); // rad
}
static float bc_pitch_rate_from_gyro(const bc_params_t* p, float gx, float gy, float gz) {
  float g[3] = {gx, gy, gz};
  return (float)p->pitch_gyro_sign * bc_v3_get(g, p->pitch_gyro_axis);
}
static void bc_set_defaults(bc_params_t* p) {
  memset(p, 0, sizeof(*p));
  p->comp_alpha = 0.98f;

  // 与你原 pitchAcc = atan2(-ax, sqrt(ay^2+az^2)) 对齐
  p->pitch_acc_axis = 0;   // x
  p->pitch_acc_sign = -1;  // -ax

  // 与你原 pitchGyro += gy*dt 对齐
  p->pitch_gyro_axis = 1;  // y
  p->pitch_gyro_sign = +1;

  p->pitch_target = 0.0f;

  // 初值（真实车上要小心调）
  p->Kp = 6.0f;
  p->Ki = 0.0f;
  p->Kd = 0.6f;
  p->integrator_limit = 2.0f;

  // cmd 采用物理单位：v_fwd(m/s), w_yaw(rad/s)
  p->v2speed_gain  = 1.0f / WHEEL_RADIUS_M;
  p->yaw2diff_gain = (TRACK_WIDTH_M * 0.5f) / WHEEL_RADIUS_M;

  p->max_out = VOLTAGE_LIMIT_INIT;
  p->max_tilt = 35.0f * (float)(M_PI / 180.0f);
  p->sensor_timeout_s = 0.2f;

  p->deadband    = 0.010f;  // ≈ 0.57°（上车很常用）
  p->ramp_time_s = 0.5f;    // 0.3~0.8s 都行，先用 0.5s

    // 外环：先保守一点，目标是“站住别跑”
  p->Kv = 0.10f;
  p->Kx = 0.50f;
  p->tilt_cmd_max = 0.30f;  // ~17deg

  // pitch_ref 自学习：非常慢，避免学歪
  p->v_gate = 0.25f;
  p->w_gate = 0.15f;
  p->bias_learn_k = 0.008f;
}
static void bc_init(bc_ctx_t* ctx) {
  memset(ctx, 0, sizeof(*ctx));
  bc_set_defaults(&ctx->p);
  ctx->d.state = BC_STATE_DISABLED;
}
static void bc_reset(bc_ctx_t* ctx) {
  ctx->pitch = 0.f;
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
static void bc_set_params(bc_ctx_t* ctx, const bc_params_t* p) { ctx->p = *p; }
static void bc_get_debug(const bc_ctx_t* ctx, bc_debug_t* out) { *out = ctx->d; }

bool g_pitch_inject_enable = 0;
float g_pitch_inject_deg = 0;

// =============================
// bc_step (完整)
// 依赖新增字段：
//   bc_params_t 增加：float deadband; float ramp_time_s;
//   bc_ctx_t    增加：float pitch_ref, enable_gain, enable_t; bool was_enabled;
// =============================
static bool bc_step(bc_ctx_t* ctx, const bc_input_t* in, bc_output_t* out) {
  out->mode  = 0;
  out->left  = 0.f;
  out->right = 0.f;
  out->ok    = false;

  float dt = in->time.dt;
  if (!(dt > 0.f && dt < 0.1f)) {
    ctx->d.state = BC_STATE_FAULT;
    ctx->d.fault_flags |= BC_FAULT_BAD_DT;
    // 禁用边沿状态复位，避免 dt 异常导致频繁上升沿误判
    ctx->was_enabled = false;
    return true;
  }

  // enable 逻辑
  bool en = in->cmd.enable;
  if (!en) {
    ctx->d.state = BC_STATE_DISABLED;
    ctx->d.fault_flags = 0;

    // 复位 enable 相关状态（下一次 enable 会重新校准 pitch_ref）
    ctx->was_enabled  = false;
    ctx->enable_gain  = 0.f;
    ctx->enable_t     = 0.f;
    ctx->err_i        = 0.f;
    return true;
  }

  // 传感器有效性
  if (!in->imu.valid) ctx->sensor_bad_t += dt;
  else                ctx->sensor_bad_t = 0.f;

  if (ctx->sensor_bad_t > ctx->p.sensor_timeout_s) {
    ctx->d.state = BC_STATE_FAULT;
    ctx->d.fault_flags |= BC_FAULT_SENSOR_LOST;

    // 进入 fault 时也复位 enable 边沿，避免来回抖动
    ctx->was_enabled = false;
    return true;
  }

  // 读取 gyro / acc 并融合
  float pitch_rate = bc_pitch_rate_from_gyro(&ctx->p, in->imu.gx, in->imu.gy, in->imu.gz); // rad/s
  float pitch_acc  = bc_pitch_from_acc(&ctx->p, in->imu.ax, in->imu.ay, in->imu.az);       // rad

  float pitch_pred = ctx->pitch + pitch_rate * dt;
  ctx->pitch = ctx->p.comp_alpha * pitch_pred + (1.f - ctx->p.comp_alpha) * pitch_acc;

  // 注入模式：仅用于验证方向/链路
  if (g_pitch_inject_enable) {
    ctx->err_i = 0.f;
    ctx->pitch = g_pitch_inject_deg * (float)(M_PI / 180.0f);
    pitch_rate = 0.f;
  }

  // 倾角保护
  if (fabsf(ctx->pitch) > ctx->p.max_tilt) {
    ctx->d.state = BC_STATE_FAULT;
    ctx->d.fault_flags |= BC_FAULT_TILT_TOO_LARGE;
    ctx->was_enabled = false;
    return true;
  }

  // ===== enable 上升沿：校准 pitch_ref + 清积分 + 渐进接管 =====
  if (en && !ctx->was_enabled) {
    ctx->err_i = 0.f;
    ctx->enable_t = 0.f;
    ctx->enable_gain = 0.f;

    // 关键：把“你扶正的当前姿态”当作直立零点
    ctx->pitch_ref = ctx->pitch;
  }

  ctx->d.state = BC_STATE_RUNNING;
  ctx->d.fault_flags = 0;

    // ===== outer-loop: use wheel speed to keep v -> 0 =====
  float v = 0.f;
  if (in->wheel.valid) {
    v = 0.5f * (in->wheel.wL + in->wheel.wR); // rad/s
  }

  // theta_cmd: want v -> 0  =>  tilt opposite to motion
  float theta_cmd = 0.f;
  if (in->wheel.valid) {
    theta_cmd = -ctx->p.Kv * v;
    theta_cmd = bc_clampf(theta_cmd, -ctx->p.tilt_cmd_max, ctx->p.tilt_cmd_max);
  }

  // ===== pitch_ref self-learning (optional, very slow) =====
  // When nearly stable, slightly adjust pitch_ref to cancel residual drift
  if (in->wheel.valid) {
    if (fabsf(v) < ctx->p.v_gate && fabsf(pitch_rate) < ctx->p.w_gate) {
      // if v>0 (rolling forward), increase pitch_ref a bit (so target tilts back)
      ctx->pitch_ref += ctx->p.bias_learn_k * v * dt;
      // keep it within reasonable range (e.g., +/- 25 deg around current)
      float lim = 25.f * (float)(M_PI / 180.0f);
      ctx->pitch_ref = bc_clampf(ctx->pitch_ref, -lim, +lim);
    }
  }

  // ===== 误差：目标 = pitch_ref + pitch_target（通常 pitch_target=0）=====
  float target = ctx->pitch_ref + ctx->p.pitch_target + theta_cmd;
  float err = target - ctx->pitch;

  // deadband：小误差不驱动，防止车自己跑
  if (fabsf(err) < ctx->p.deadband) err = 0.f;

  // 积分（建议初期 Ki=0；但保留实现）
  if (ctx->p.Ki != 0.f && !g_pitch_inject_enable) {
    ctx->err_i += err * dt;
    ctx->err_i = bc_clampf(ctx->err_i, -ctx->p.integrator_limit, ctx->p.integrator_limit);
  } else {
    ctx->err_i = 0.f;
  }

  // PID（D 用 -pitch_rate 做阻尼）
  Serial.println(err);
  float u_balance = ctx->p.Kp * err + ctx->p.Ki * ctx->err_i + ctx->p.Kd * (-pitch_rate);

  // ===== 渐进接管 ramp-in：enable 初期不要猛冲 =====
  ctx->enable_t += dt;
  float g = 1.f;
  if (ctx->p.ramp_time_s > 1e-3f) {
    g = bc_clampf(ctx->enable_t / ctx->p.ramp_time_s, 0.f, 1.f);
  }
  ctx->enable_gain = g;
  u_balance *= g;

  // 前进/转向（目前你上层没用到 yaw diff，所以保持 diff=0）
  float u_total = u_balance + ctx->p.v2speed_gain * in->cmd.v_fwd;
  float diff    = ctx->p.yaw2diff_gain * in->cmd.w_yaw;

  float left  = u_total - diff;
  float right = u_total + diff;

  left  = bc_clampf(left,  -ctx->p.max_out, ctx->p.max_out);
  right = bc_clampf(right, -ctx->p.max_out, ctx->p.max_out);

  out->left  = left;
  out->right = right;
  out->ok    = true;

  // debug
  ctx->d.pitch      = ctx->pitch;
  ctx->d.pitch_rate = pitch_rate;
  ctx->d.err        = err;
  ctx->d.err_i      = ctx->err_i;
  ctx->d.u_balance  = u_balance;
  ctx->d.u_left     = left;
  ctx->d.u_right    = right;

  // 更新 enable 记忆
  ctx->was_enabled = en;

  return true;
}

// ============================================================
// SimpleFOC objects
// ============================================================
MagneticSensorI2C lsensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C rsensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor lmotor(POLE_PAIRS);
BLDCMotor rmotor(POLE_PAIRS);
BLDCDriver3PWM ldriver(PIN_PWM_U, PIN_PWM_V, PIN_PWM_W, PIN_EN);
BLDCDriver3PWM rdriver(PIN_PWM_U2, PIN_PWM_V2, PIN_PWM_W2, PIN_EN2);

// ============================================================
// Shared runtime state (task-safe by design: single writer / single reader patterns)
// ============================================================

// IMU latest sample (written by imuTask, read by balanceTask)
static volatile bool  g_imu_valid = false;
static volatile float g_ax = 0, g_ay = 0, g_az = 0; // m/s^2
static volatile float g_gx = 0, g_gy = 0, g_gz = 0; // rad/s
static volatile float g_dt = 0.005f;

// for 3D output (deg)
static volatile float g_pitch_deg = 0, g_roll_deg = 0, g_yaw_deg = 0;

// balance core
static bc_ctx_t   g_bc;
static bc_params_t g_p;

// balance outputs
static volatile float gl_balance_target = 0.0f;
static volatile float gr_balance_target = 0.0f;
static volatile bool  g_balance_ok = false;

// enable flags
static volatile bool g_balance_enable = true; // BE1/BE0
static volatile bool g_motor_enable   = true; // ME1/ME0

// SimpleFOC command requests (written by cmdTask, applied by focTask)
static volatile int   g_req_motion_mode = 0;   // 0 torque, 1 velocity, 2 angle
static volatile int   g_req_torque_mode = 0;   // 0 volt, 2 current (SimpleFOC enum differs; we map)
static volatile bool  g_req_apply_mode  = true;

static volatile float g_manual_target = 0.0f;  // from "M..."
static volatile float g_req_vlimit = 0.0f;     // velocity_limit for angle mode
static volatile float g_req_tlimit = VOLTAGE_LIMIT_INIT; // voltage_limit
static volatile bool  g_req_apply_limits = true;

static volatile bool  g_req_apply_pid = false;
static volatile float g_req_pid_p = 0.2f, g_req_pid_i = 20.0f, g_req_pid_d = 0.001f;

// wheel feedback (written by focTask, read by balanceTask)
static volatile bool  g_wheel_valid = false;
static volatile float g_wL = 0.0f, g_wR = 0.0f;      // rad/s
static volatile float g_xL = 0.0f, g_xR = 0.0f;      // rad (angle)

// Serial line buffer
static String g_line;

// ============================================================
// Helpers: prints
// ============================================================
static void printHelp() {
  Serial.println("HELP:");
  Serial.println("  ME1/ME0         motor enable/disable");
  Serial.println("  BE1/BE0         balance enable/disable");
  Serial.println("  BC?             print balance params (BCP,...)");
  Serial.println("  BS?             print balance status (BAL,...)");
  Serial.println("  BC <Key> <Val>  set balance param, then print BCP");
  Serial.println("     Keys: Kp Ki Kd Alpha MaxOut MaxTiltDeg PitchTargetDeg V2Speed Yaw2Diff");
  Serial.println("  MC0/1/2         motion: torque/velocity/angle");
  Serial.println("  MT0/2           torque mode: volt/current");
  Serial.println("  M<target> [..]  set target; angle mode supports: M<ang> <vlimit> <tlimit>");
  Serial.println("                  velocity mode supports: M<vel> <tlimit>");
  Serial.println("  M P/I/D <val>   set motor.PID_velocity P/I/D");
}

static void printBCP() {
  // BCP,kp,ki,kd,alpha,max_out,max_tilt,pitch_target,v2speed,yaw2diff
  Serial.print("BCP,");
  Serial.print(g_p.Kp, 6); Serial.print(",");
  Serial.print(g_p.Ki, 6); Serial.print(",");
  Serial.print(g_p.Kd, 6); Serial.print(",");
  Serial.print(g_p.comp_alpha, 6); Serial.print(",");
  Serial.print(g_p.max_out, 6); Serial.print(",");
  Serial.print(g_p.max_tilt, 6); Serial.print(",");
  Serial.print(g_p.pitch_target, 6); Serial.print(",");
  Serial.print(g_p.v2speed_gain, 6); Serial.print(",");
  Serial.print(g_p.yaw2diff_gain, 6);
  Serial.println();
}

static void printBAL() {
  bc_debug_t d;
  bc_get_debug(&g_bc, &d);
  Serial.print("BAL,pitch="); Serial.print(d.pitch, 6);
  Serial.print(",pr=");       Serial.print(d.pitch_rate, 6);
  Serial.print(",uL=");       Serial.print(d.u_left, 2);
  Serial.print(",lTgt=");     Serial.print(gl_balance_target);
  Serial.print(",rTgt=");     Serial.print(gr_balance_target);  
  Serial.print(",pinject=");           Serial.print(g_pitch_inject_enable ? 1 : 0, 2);
  Serial.print(",pinject_deg=");       Serial.print(g_pitch_inject_deg, 6);
  Serial.print(",state=");    Serial.print((int)d.state);
  Serial.print(",ok=");       Serial.print((int)(g_balance_ok ? 1 : 0));
  Serial.println();
}

// ============================================================
// Command parsing (line-based)
// ============================================================
static void trimLeft(char* &p) { while (*p==' '||*p=='\t') ++p; }

// Parse M command variants:
// - "M10" (no spaces)
// - "M10 60 0.5" (angle mode)
// - "M10 3" (velocity mode set tlimit)
// - "M P 0.2" etc
static void handleM(char* line) {
  // line begins with 'M'
  // Support: "M P 0.2" / "M I 20" / "M D 0.001"
  if (line[1] == ' ' || line[1] == '\t') {
    char* p = line + 1;
    trimLeft(p);
    if (*p == 'P' || *p == 'I' || *p == 'D') {
      char which = *p;
      p++;
      trimLeft(p);
      float v = (float)atof(p);
      if (which=='P') g_req_pid_p = v;
      if (which=='I') g_req_pid_i = v;
      if (which=='D') g_req_pid_d = v;
      g_req_apply_pid = true;
      Serial.printf("ACK,MPID,%c=%.6f\n", which, v);
      return;
    }
  }

  // Numeric target: starts right after 'M'
  char* p = line + 1;
  float target = (float)atof(p);

  // Tokenize by space for optional args
  // Example: "M10 60 0.5"
  // We'll parse all numbers present
  float nums[3] = {0};
  int count = 0;

  // First number is already parsed as target; still parse properly:
  // We'll use strtok on a copy
  char buf[96];
  strncpy(buf, line, sizeof(buf)-1);
  buf[sizeof(buf)-1] = 0;

  // Replace leading 'M' with space so strtok reads the first number cleanly
  buf[0] = ' ';
  char* tok = strtok(buf, " \t");
  while (tok && count < 3) {
    nums[count++] = (float)atof(tok);
    tok = strtok(nullptr, " \t");
  }

  if (count >= 1) target = nums[0];

  g_manual_target = target;

  // Apply limits depending on current motion mode request
  // - angle mode: M ang vlimit tlimit
  // - velocity mode: M vel tlimit
  // - torque mode: M torque tlimit (optional, treat as voltage_limit)
  if (g_req_motion_mode == 2) {
    if (count >= 2) g_req_vlimit = nums[1];
    if (count >= 3) g_req_tlimit = nums[2];
    g_req_apply_limits = true;
  } else if (g_req_motion_mode == 1) {
    if (count >= 2) g_req_tlimit = nums[1];
    g_req_apply_limits = true;
  } else { // torque
    if (count >= 2) g_req_tlimit = nums[1];
    g_req_apply_limits = true;
  }

  Serial.printf("ACK,M,target=%.3f\n", target);
}

static void handleMC(char* line) {
  // "MC0"/"MC1"/"MC2"
  int v = atoi(line + 2);
  if (v < 0) v = 0;
  if (v > 2) v = 2;
  g_req_motion_mode = v;
  g_req_apply_mode = true;
  Serial.printf("ACK,MC=%d\n", v);
}

static void handleMT(char* line) {
  // "MT0" / "MT2"
  int v = atoi(line + 2);
  g_req_torque_mode = v;
  g_req_apply_mode = true;
  Serial.printf("ACK,MT=%d\n", v);
}

static void handleME(char* line) {
  int v = atoi(line + 2);
  g_motor_enable = (v != 0);
  Serial.printf("ACK,ME=%d\n", g_motor_enable ? 1 : 0);
}
static void handleBE(char* line) {
  int v = atoi(line + 2);
  g_balance_enable = (v != 0);
  Serial.printf("ACK,BE=%d\n", g_balance_enable ? 1 : 0);
}

static void handleBC(char* line) {
  // BC?  OR  BC <Key> <Val>
  if (strcmp(line, "BC?") == 0) { printBCP(); return; }

  // tokenize: "BC Key Val"
  char buf[128];
  strncpy(buf, line, sizeof(buf)-1);
  buf[sizeof(buf)-1] = 0;

  char* tok = strtok(buf, " \t");
  if (!tok || strcmp(tok, "BC") != 0) return;
  char* key = strtok(nullptr, " \t");
  char* val = strtok(nullptr, " \t");
  if (!key || !val) { Serial.println("ERR,BC,usage: BC <Key> <Val>"); return; }

  float f = (float)atof(val);

  if      (strcmp(key, "Kp") == 0) g_p.Kp = f;
  else if (strcmp(key, "Ki") == 0) g_p.Ki = f;
  else if (strcmp(key, "Kd") == 0) g_p.Kd = f;
  else if (strcmp(key, "Alpha") == 0) g_p.comp_alpha = f;
  else if (strcmp(key, "MaxOut") == 0) g_p.max_out = f;
  else if (strcmp(key, "MaxTiltDeg") == 0) g_p.max_tilt = f * (float)(M_PI/180.0f);
  else if (strcmp(key, "PitchTargetDeg") == 0) g_p.pitch_target = f * (float)(M_PI/180.0f);
  else if (strcmp(key, "V2Speed") == 0) g_p.v2speed_gain = f;
  else if (strcmp(key, "Yaw2Diff") == 0) g_p.yaw2diff_gain = f;
   else if (strcmp(key, "Kv") == 0) g_p.Kv = f;
  else if (strcmp(key, "Kx") == 0) g_p.Kx = f;
  else if (strcmp(key, "TiltCmdMaxDeg") == 0) g_p.tilt_cmd_max = f * (float)(M_PI/180.0f);
  else if (strcmp(key, "BiasLearnK") == 0) g_p.bias_learn_k = f;
  else if (strcmp(key, "VGate") == 0) g_p.v_gate = f;
  else if (strcmp(key, "WGate") == 0) g_p.w_gate = f;
  else { Serial.printf("ERR,BC,unknown_key=%s\n", key); return; }

  bc_set_params(&g_bc, &g_p);
  printBCP();
}

static void handleBS(char* line) {
  // BS? 输出一次 BAL 状态
  if (strcmp(line, "BS?") == 0) { printBAL(); return; }
  Serial.println("ERR,BS,usage: BS?");
}

static void handlePIM(char* line) {
  // "PIM 0" / "PIM 1"
  int v = atoi(line + 4);           
  g_pitch_inject_enable = (v != 0);
  Serial.printf("ACK,PIM=%d\n", g_pitch_inject_enable ? 1 : 0);
}

static void handlePSET(char* line) {
  // "PSET 5" / "PSET -5" / "PSET 12.34"
  float deg = (float)atof(line + 5);
  g_pitch_inject_deg = deg;
  Serial.printf("ACK,PSET=%.2f\n", g_pitch_inject_deg);
}

static void dispatchLine(const String& s) {
  if (s.length() == 0) return;

  // Copy to C string
  char line[160];
  size_t n = s.length();
  if (n >= sizeof(line)) n = sizeof(line)-1;
  memcpy(line, s.c_str(), n);
  line[n] = 0;

  // Note: BAL/IMU 是 MCU 输出，不是命令；即使用户误发，也忽略
  if (strncmp(line, "BAL,", 4) == 0) return;
  if (strncmp(line, "IMU,", 4) == 0) return;

  if (strcmp(line, "?") == 0) { printHelp(); return; }

  if (strncmp(line, "ME", 2) == 0) { handleME(line); return; }
  if (strncmp(line, "BE", 2) == 0) { handleBE(line); return; }

  if (strncmp(line, "BC", 2) == 0) { handleBC(line); return; }
  if (strncmp(line, "BS", 2) == 0) { handleBS(line); return; }

  if (strncmp(line, "MC", 2) == 0) { handleMC(line); return; }
  if (strncmp(line, "MT", 2) == 0) { handleMT(line); return; }

  if (strncmp(line, "PIM", 3) == 0) { handlePIM(line); return; }
  if (strncmp(line, "PSET", 4) == 0) { handlePSET(line); return; }
  if (line[0] == 'M') { handleM(line); return; }

  Serial.printf("ERR,unknown_cmd=%s\n", line);
}

static void pollSerialLines() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      g_line.trim();
      dispatchLine(g_line);
      g_line = "";
    } else {
      if (g_line.length() < 150) g_line += c;
    }
  }
}

// ============================================================
// Tasks
// ============================================================

void ledTask(void* arg) {
  pinMode(LED_PIN, OUTPUT);
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// imuTask: 200Hz read + complementary filter + 50Hz IMU print
void imuTask(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(5); // 200Hz
  TickType_t last = xTaskGetTickCount();

  float roll = 0.f, pitch = 0.f, yaw = 0.f;  // deg
  const float alpha = 0.98f;
  uint32_t lastUs = micros();
  uint32_t lastPrintMs = 0;

  while (true) {
    ImuRaw s;
    bool ok = mpuRead14(s);

    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    if (!(dt > 0.f && dt < 0.1f)) dt = 0.005f;
    lastUs = nowUs;

    if (ok) {
      // accel raw -> g
      float ax_g = s.ax / 16384.0f;
      float ay_g = s.ay / 16384.0f;
      float az_g = s.az / 16384.0f;

      // gyro raw -> deg/s
      float gx_dps = s.gx / 131.0f;
      float gy_dps = s.gy / 131.0f;
      float gz_dps = s.gz / 131.0f;

      // accel angles
      float rollAcc  = atan2f(ay_g, az_g) * 180.0f / (float)M_PI;
      float pitchAcc = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / (float)M_PI;

      // integrate gyro
      float rollGyro  = roll  + gx_dps * dt;
      float pitchGyro = pitch + gy_dps * dt;
      float yawGyro   = yaw   + gz_dps * dt;

      // complementary
      roll  = alpha * rollGyro  + (1.f - alpha) * rollAcc;
      pitch = alpha * pitchGyro + (1.f - alpha) * pitchAcc;
      yaw   = yawGyro;

      // Publish IMU scaled for balance_core
      // accel g -> m/s^2, gyro dps -> rad/s
      float ax = ax_g * 9.80665f;
      float ay = ay_g * 9.80665f;
      float az = az_g * 9.80665f;

      float gx = gx_dps * (float)(M_PI / 180.0f);
      float gy = gy_dps * (float)(M_PI / 180.0f);
      float gz = gz_dps * (float)(M_PI / 180.0f);

      g_ax = ax; g_ay = ay; g_az = az;
      g_gx = gx; g_gy = gy; g_gz = gz;
      g_dt = dt;
      g_imu_valid = true;

      g_pitch_deg = pitch;
      g_roll_deg  = roll;
      g_yaw_deg   = yaw;

      // 50Hz output for 3D
      uint32_t ms = millis();
      if (ms - lastPrintMs >= 200) {
        lastPrintMs = ms;
        Serial.printf("IMU,%.2f,%.2f,%.2f\n", pitch, roll, yaw);
      }
    } else {
      g_imu_valid = false;
    }

    vTaskDelayUntil(&last, period);
  }
}

// balanceTask: 200Hz bc_step -> g_balance_target; 10Hz BAL print
void balanceTask(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(5); // 200Hz
  TickType_t last = xTaskGetTickCount();
  uint32_t lastPrintMs = 0;

  bc_reset(&g_bc);

  while (true) {
    bc_input_t in;
    memset(&in, 0, sizeof(in));

    in.time.dt = g_dt;
    in.cmd.enable = g_balance_enable;

    // 先不做前进/转向（网页目前没给）
    in.cmd.v_fwd = 0.0f;
    in.cmd.w_yaw = 0.0f;

    in.imu.valid = g_imu_valid;
    in.imu.ax = g_ax; in.imu.ay = g_ay; in.imu.az = g_az;
    in.imu.gx = g_gx; in.imu.gy = g_gy; in.imu.gz = g_gz;

     // wheel feedback for outer-loop
    in.wheel.valid = g_wheel_valid;
    in.wheel.wL = g_wL;
    in.wheel.wR = g_wR;

    bc_output_t out;
    bc_step(&g_bc, &in, &out);

    if (out.ok && g_bc.d.state == BC_STATE_RUNNING && g_balance_enable) {
      gl_balance_target = out.left; // 单电机用 left
      gr_balance_target = out.right;
      g_balance_ok = true;
    } else {
      gl_balance_target = 0.0f;
      gr_balance_target = 0.0f;
      g_balance_ok = false;
    }

    // 10Hz print BAL（网页 Telemetry 面板用；你也可以改到 20Hz）
    uint32_t ms = millis();
    if (ms - lastPrintMs >= 100) {
      lastPrintMs = ms;
      // printBAL();
    }

    vTaskDelayUntil(&last, period);
  }
}

// =============================
// focTask (完整)
// 关键改动：
//   1) 不要每 1ms 重复 enable()/disable() ——只在状态变化时调用
//   2) 平衡启用且 ok 时，强制 torque+voltage（避免误切 velocity/angle）
//   3) PID_velocity 只在 velocity/angle 用；torque 下不需要每圈写
// =============================
void focTask(void* arg) {
  // ==== left motor ====
  lsensor.init();
  lmotor.linkSensor(&lsensor);

  ldriver.voltage_power_supply = SUPPLY_VOLTAGE;
  // 如果你驱动类支持 pwm_frequency，建议设到 25k+ 降低可听噪音（不同版本字段可能不同）
  // ldriver.pwm_frequency = 32000;
  ldriver.init();
  lmotor.linkDriver(&ldriver);

  lmotor.controller = MotionControlType::torque;
  lmotor.torque_controller = TorqueControlType::voltage;
  lmotor.voltage_limit  = VOLTAGE_LIMIT_INIT;  // 10V 初值（确认供电能力）
  lmotor.velocity_limit = 100.0f;

  lmotor.init();
  if (!lmotor.initFOC()) {
    Serial.println("Left FOC init failed!");
    vTaskDelete(nullptr);
    return;
  }

  // ==== right motor ====
  rsensor.init(&Wire2);
  rmotor.linkSensor(&rsensor);

  rdriver.voltage_power_supply = SUPPLY_VOLTAGE;
  // rdriver.pwm_frequency = 32000;
  rdriver.init();
  rmotor.linkDriver(&rdriver);

  rmotor.controller = MotionControlType::torque;
  rmotor.torque_controller = TorqueControlType::voltage;
  rmotor.voltage_limit  = VOLTAGE_LIMIT_INIT;
  rmotor.velocity_limit = 100.0f;

  rmotor.init();
  if (!rmotor.initFOC()) {
    Serial.println("Right FOC init failed!");
    vTaskDelete(nullptr);
    return;
  }

  Serial.println("FOC ready");

  const TickType_t period = pdMS_TO_TICKS(1); // 1kHz
  TickType_t last = xTaskGetTickCount();
  uint32_t lastPrintMs = 0;

  // 只在变化时 enable/disable（减少噪音、避免奇怪抖动）
  bool last_motor_enable = false;

  while (true) {
    // ===== enable/disable only on edge =====
    bool me = g_motor_enable;
    if (me != last_motor_enable) {
      last_motor_enable = me;
      if (me) {
        ldriver.enable(); lmotor.enable();
        rdriver.enable(); rmotor.enable();
      } else {
        lmotor.disable(); ldriver.disable();
        rmotor.disable(); rdriver.disable();
      }
    }

    // ===== apply mode changes safely inside this task =====
    if (g_req_apply_mode) {
      // motion controller
      if (g_req_motion_mode == 0) {
        lmotor.controller = MotionControlType::torque;
        rmotor.controller = MotionControlType::torque;
      } else if (g_req_motion_mode == 1) {
        lmotor.controller = MotionControlType::velocity;
        rmotor.controller = MotionControlType::velocity;
      } else {
        lmotor.controller = MotionControlType::angle;
        rmotor.controller = MotionControlType::angle;
      }

      // torque mode (SimpleFOC enum)
      if (g_req_torque_mode == 0) {
        lmotor.torque_controller = TorqueControlType::voltage;
        rmotor.torque_controller = TorqueControlType::voltage;
      } else if (g_req_torque_mode == 2) {
        lmotor.torque_controller = TorqueControlType::foc_current;
        rmotor.torque_controller = TorqueControlType::foc_current;
      }

      g_req_apply_mode = false;
      Serial.printf("ACK,APPLY,MC=%d,MT=%d\n", (int)g_req_motion_mode, (int)g_req_torque_mode);
    }

    // ===== apply limits =====
    if (g_req_apply_limits) {
      lmotor.voltage_limit = g_req_tlimit;
      rmotor.voltage_limit = g_req_tlimit;

      if (g_req_vlimit > 0) {
        lmotor.velocity_limit = g_req_vlimit;
        rmotor.velocity_limit = g_req_vlimit;
      }

      g_req_apply_limits = false;
      Serial.printf("ACK,APPLY,LIMIT,l_vlim=%.2f,l_tlim=%.2f,r_vlim=%.2f,r_tlim=%.2f\n",
        lmotor.velocity_limit, lmotor.voltage_limit, rmotor.velocity_limit, rmotor.voltage_limit);
    }

    // ===== apply PID only when requested =====
    // 注意：torque 模式下 PID_velocity 不参与，但你手动切 velocity/angle 时需要它
    if (g_req_apply_pid) {
      lmotor.PID_velocity.P = g_req_pid_p;
      lmotor.PID_velocity.I = g_req_pid_i;
      lmotor.PID_velocity.D = g_req_pid_d;

      rmotor.PID_velocity.P = g_req_pid_p;
      rmotor.PID_velocity.I = g_req_pid_i;
      rmotor.PID_velocity.D = g_req_pid_d;

      g_req_apply_pid = false;
      Serial.printf("ACK,APPLY,PID,lP=%.6f,lI=%.6f,lD=%.6f,rP=%.6f,rI=%.6f,rD=%.6f\n",
        lmotor.PID_velocity.P, lmotor.PID_velocity.I, lmotor.PID_velocity.D,
        rmotor.PID_velocity.P, rmotor.PID_velocity.I, rmotor.PID_velocity.D);
    }

    // ===== run FOC =====
    lmotor.loopFOC();
    rmotor.loopFOC();

    g_wL = lsensor.getVelocity();
    g_wR = rsensor.getVelocity();
    g_xL = lsensor.getAngle();
    g_xR = rsensor.getAngle();
    g_wheel_valid = true;

    // ===== choose target =====
    float ltarget = 0.0f;
    float rtarget = 0.0f;

    if (g_balance_enable && g_balance_ok) {
      // 平衡时强制 torque+voltage（避免误切 velocity/angle 导致叠环抖动）
      lmotor.controller = MotionControlType::torque;
      rmotor.controller = MotionControlType::torque;
      lmotor.torque_controller = TorqueControlType::voltage;
      rmotor.torque_controller = TorqueControlType::voltage;

      ltarget = gl_balance_target;   // bc_step 输出的“电压/力矩等效”
      rtarget = gr_balance_target;
    } else {
      ltarget = g_manual_target;
      rtarget = g_manual_target;
    }

    // ===== apply motion =====
    // torque+voltage: move(x) 的 x 当作电压（V）
    // velocity: move(x) 的 x 当作 rad/s
    // angle:   move(x) 的 x 当作 rad
    lmotor.move(ltarget);
    rmotor.move(rtarget);

    // ===== debug (1Hz) =====
    uint32_t ms = millis();
    if (ms - lastPrintMs >= 1000) {
      lastPrintMs = ms;
      Serial.print("MOT,ctrl=");
      Serial.print((int)lmotor.controller);
      Serial.print(",ltgt=");
      Serial.print(ltarget, 3);
      Serial.print(",lvel=");
      Serial.print(lsensor.getVelocity(), 3);
      Serial.print(",langle=");
      Serial.print(lsensor.getAngle(), 3);
      Serial.print(",rtgt=");
      Serial.print(rtarget, 3);
      Serial.print(",rvel=");
      Serial.print(rsensor.getVelocity(), 3);
      Serial.print(",rangle=");
      Serial.println(rsensor.getAngle(), 3);
    }

    vTaskDelayUntil(&last, period);
  }
}

// cmdTask: poll serial lines at 100Hz
void cmdTask(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t last = xTaskGetTickCount();
  while (true) {
    pollSerialLines();
    vTaskDelayUntil(&last, period);
  }
}

static void scanBus(TwoWire& w, const char* name) {
  Serial.printf("SCAN,%s,begin\n", name);
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    w.beginTransmission(addr);
    uint8_t err = w.endTransmission();
    if (err == 0) {
      Serial.printf("SCAN,%s,addr=0x%02X\n", name, addr);
      found++;
    }
  }
  Serial.printf("SCAN,%s,found=%d\n", name, found);
}


// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  SimpleFOCDebug::enable(&Serial);

   // I2C sensor for AS5600
  Wire.begin((int)PIN_I2C_SDA, (int)PIN_I2C_SCL);
  Wire.setClock(400000);

  Wire2.begin((int)PIN_I2C_SDA2, (int)PIN_I2C_SCL2);
  Wire2.setClock(400000);

  scanBus(Wire, "Wire(8,9)");
  mpuWrite(0x6B, 0x00); // wake

  // init balance core
  bc_init(&g_bc);
  g_p = g_bc.p; // local copy used for BC commands

  // Start tasks
  xTaskCreatePinnedToCore(ledTask, "LED", 2048, nullptr, 4, nullptr, 0);

  xTaskCreatePinnedToCore(imuTask, "IMU", 8192, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(balanceTask, "BAL", 8192, nullptr, 1, nullptr, 0);

  xTaskCreatePinnedToCore(focTask, "FOC", 8192, nullptr, 1, nullptr, 1);

  xTaskCreatePinnedToCore(cmdTask, "CMD", 4096, nullptr, 4, nullptr, 1);

  Serial.println("Tasks started");
  printHelp();
}

void loop() {
  // 不在 loop 里处理串口，避免干扰 realtime tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
