/*
 * robot.h - 平衡机器人数据结构和接口定义
 *
 * 所有状态、参数、调试数据都在这里定义。
 * 这是唯一的"数据头文件"。
 */
#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 平衡控制参数
 */
struct BalanceParams {
    float angle_kp;
    float gyro_kp;
    float distance_kp;
    float speed_kp;
    float yaw_angle_kp;
    float yaw_gyro_kp;
    float lqr_u_kp;
    float lqr_u_ki;
    float zeropoint_kp;
    float lpf_target_vel_tf;
    float lpf_zeropoint_tf;
    float ff_gain;
    float max_tilt_deg;
    float pitch_offset;
    float pid_limit;
    float lift_accel_thresh;
    float lift_vel_thresh;
};

/*
 * 默认参数 (从 balance_controller_defaults.h)
 */
static const struct BalanceParams DEFAULT_PARAMS = {
    .angle_kp          = 1.0f,
    .gyro_kp           = 0.08f,
    .distance_kp       = 0.3f,
    .speed_kp          = 1.0f,
    .yaw_angle_kp      = 1.0f,
    .yaw_gyro_kp       = 0.04f,
    .lqr_u_kp          = 1.0f,
    .lqr_u_ki          = 0.0f,
    .zeropoint_kp      = 0.002f,
    .lpf_target_vel_tf = 0.6f,
    .lpf_zeropoint_tf  = 0.1f,
    .ff_gain           = 0.0f,
    .max_tilt_deg      = 60.0f,
    .pitch_offset      = 0.0f,
    .pid_limit         = 10.0f,
    .lift_accel_thresh = 50.0f,
    .lift_vel_thresh   = 150.0f,
};

/*
 * IMU 状态
 */
struct ImuState {
    volatile bool valid;
    volatile float ax, ay, az;   /* m/s^2 */
    volatile float gx, gy, gz;   /* rad/s */
    volatile float pitch;        /* deg */
    volatile float roll;         /* deg */
    volatile float yaw;          /* deg */
};

/*
 * 轮子状态
 */
struct WheelState {
    volatile bool valid;
    volatile float w_l, w_r;     /* rad/s */
    volatile float x_l, x_r;     /* rad */
};

/*
 * 平衡输出
 */
struct BalanceOutput {
    volatile float motor_l;
    volatile float motor_r;
    volatile bool ok;
};

/*
 * 命令状态
 */
struct CommandState {
    volatile bool balance_en;
    volatile bool motor_en;
    volatile int req_mot_mode;       /* 0=torque, 1=velocity, 2=angle */
    volatile int req_trq_mode;       /* 0=voltage, 2=foc_current */
    volatile bool apply_mode;
    volatile float manual_tgt;
    volatile float v_limit;
    volatile float t_limit;
    volatile bool apply_limits;
    volatile bool apply_pid;
    volatile float pid_p, pid_i, pid_d;
};

/*
 * 调试数据
 */
struct BalanceDebug {
    /* IMU */
    float ax, ay, az;
    float gx, gy, gz;
    float pitch, pitch_rate, roll, yaw_rate;

    /* 轮子 */
    float wheel_vel;
    float w_l, w_r, x_l, x_r;

    /* 各环贡献 */
    float angle_ctrl;
    float gyro_ctrl;
    float distance_ctrl;
    float speed_ctrl;
    float lqr_raw;
    float lqr_comp;

    /* 偏航 */
    float yaw_out;
    float heading;

    /* 零点 */
    float pitch_offset;
    float distance_zero;

    /* 输出 */
    float motor_l;
    float motor_r;

    /* 状态 */
    bool running;
    uint32_t fault_flags;
    bool lifted;

    /* CoG 自适应 */
    bool cog_active;
    float cog_dist_ctrl;
    float cog_adj;
    float cog_lqr;
    float cog_speed;
    bool cog_joy;
};

/*
 * 全局实例声明 (在 main.cpp 中定义)
 */
extern struct BalanceParams g_params;
extern struct ImuState       g_imu;
extern struct WheelState     g_wheel;
extern struct BalanceOutput  g_out;
extern struct CommandState   g_cmd;
extern struct BalanceDebug   g_debug;
extern volatile float        g_tgt_vel;
extern volatile float        g_tgt_yaw;

/*
 * 平衡控制接口
 */
void balance_init(void);
void balance_step(float dt);
void balance_reset(void);
void balance_get_debug(struct BalanceDebug* out);
void balance_get_params(struct BalanceParams* out);
void balance_set_params(const struct BalanceParams* p);

/*
 * IMU 接口
 */
void imu_init(void);
void imu_read(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_H */
