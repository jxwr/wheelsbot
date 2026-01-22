#ifndef BALANCE_CORE_H
#define BALANCE_CORE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================== Input =====================
// 推荐单位：
// - 加速度: m/s^2
// - 角速度: rad/s
// - dt: s
// - 轮速: rad/s

typedef struct {
  float ax, ay, az;   // m/s^2 (body frame)
  float gx, gy, gz;   // rad/s  (body frame)
  bool  valid;        // I2C/数据有效
} bc_imu_t;

typedef struct {
  float wL;           // left wheel speed (rad/s)
  float wR;           // right wheel speed (rad/s)
  bool  valid;
} bc_wheelsense_t;

typedef struct {
  float dt;           // seconds
} bc_time_t;

// 注意：cmd 的单位由你定义。
// 推荐：v_fwd = m/s, w_yaw = rad/s（这样映射可计算，不靠瞎调）
typedef struct {
  float v_fwd;        // forward command
  float w_yaw;        // yaw rate command
  bool  enable;       // enable controller
} bc_user_cmd_t;

typedef struct {
  bc_imu_t        imu;
  bc_wheelsense_t wheel;
  bc_time_t       time;
  bc_user_cmd_t   cmd;
} bc_input_t;

// ===================== Output =====================
typedef enum {
  BC_OUT_WHEEL_VELOCITY = 0, // 输出左右轮目标角速度(rad/s)
  BC_OUT_WHEEL_TORQUE   = 1  // 输出左右轮扭矩/电压等命令（由电机层解释）
} bc_output_mode_t;

typedef struct {
  bc_output_mode_t mode;
  float left;         // rad/s or torque_cmd
  float right;        // rad/s or torque_cmd
  bool  ok;           // 本次输出是否可信
} bc_output_t;

// ===================== Params =====================
typedef struct {
  // ---- attitude estimation: complementary filter ----
  float comp_alpha;        // e.g. 0.98

  // pitch axis mapping
  uint8_t pitch_gyro_axis; // 0=x,1=y,2=z
  int8_t  pitch_gyro_sign; // +1 or -1
  uint8_t pitch_acc_axis;  // 0=x,1=y,2=z
  int8_t  pitch_acc_sign;  // +1 or -1

  // ---- balance outer loop (pitch -> wheel speed/torque) ----
  float pitch_target;      // rad (0 = upright)
  float Kp, Ki, Kd;
  float integrator_limit;

  // ---- command mixing ----
  // 约定：u_total = u_balance + v2speed_gain * cmd.v_fwd
  //       diff    = yaw2diff_gain * cmd.w_yaw
  // left = u_total - diff, right = u_total + diff
  float v2speed_gain;
  float yaw2diff_gain;

  // ---- limits & safety ----
  float max_out;           // clamp output
  float max_tilt;          // rad, beyond -> fault
  float sensor_timeout_s;  // imu invalid longer than this -> fault
} bc_params_t;

// ===================== Debug =====================
typedef enum {
  BC_STATE_DISABLED = 0,
  BC_STATE_RUNNING  = 1,
  BC_STATE_FAULT    = 2
} bc_state_t;

typedef struct {
  // estimated attitude
  float pitch;        // rad
  float pitch_rate;   // rad/s

  // controller terms
  float err;
  float err_i;
  float u_balance;
  float u_left;
  float u_right;

  // state/fault
  bc_state_t state;
  uint32_t fault_flags;
} bc_debug_t;

// opaque context
typedef struct bc_ctx bc_ctx_t;

// ===================== API =====================
bc_ctx_t*  bc_create(void);
void       bc_destroy(bc_ctx_t* ctx);

void       bc_reset(bc_ctx_t* ctx);

void       bc_set_params(bc_ctx_t* ctx, const bc_params_t* p);
void       bc_get_params(const bc_ctx_t* ctx, bc_params_t* out);

bc_state_t bc_get_state(const bc_ctx_t* ctx);
void       bc_get_debug(const bc_ctx_t* ctx, bc_debug_t* out);

// One step: input -> output
bool       bc_step(bc_ctx_t* ctx, const bc_input_t* in, bc_output_t* out);

#ifdef __cplusplus
}
#endif
#endif
