/*
 * balance.cpp - Balance control algorithm
 *
 * Translated from BalanceController::step(), logic preserved exactly.
 * Uses SimpleFOC PIDController and LowPassFilter.
 */

#include "robot.h"
#include <SimpleFOC.h>
#include <math.h>

/* Constants */
static constexpr float PI_F = 3.14159265358979f;
static constexpr float WHEEL_RADIUS = 0.03f;

/* PID controllers */
static PIDController pid_angle     {1, 0, 0, 100000, 1};
static PIDController pid_gyro      {1, 0, 0, 100000, 1};
static PIDController pid_distance  {1, 0, 0, 100000, 1};
static PIDController pid_speed     {1, 0, 0, 100000, 1};
static PIDController pid_yaw_angle {1, 0, 0, 100000, 1};
static PIDController pid_yaw_gyro  {1, 0, 0, 100000, 1};
static PIDController pid_lqr_u     {1, 1, 0, 100000, 1};
static PIDController pid_zeropoint {1, 0, 0, 100000, 1};

/* Low-pass filters */
static LowPassFilter lpf_tgt_vel {0.2f};
static LowPassFilter lpf_zero    {0.1f};

/* Internal state */
static float distance_zero = -256.0f;
static float spd_prev = 0;
static float prev_tgt_vel = 0;
static float yaw_cur = 0;
static float yaw_tgt = 0;
static int   unctl = 0;
static bool  had_joy = false;
static bool  had_yaw = false;

/* PID reset helper */
static void reset_pid(PIDController& pid) {
    float p = pid.P, i = pid.I, d = pid.D;
    float ramp = pid.output_ramp, lim = pid.limit;
    pid.~PIDController();
    new (&pid) PIDController(p, i, d, ramp, lim);
}

static void reset_all_pids(void) {
    reset_pid(pid_angle);
    reset_pid(pid_gyro);
    reset_pid(pid_distance);
    reset_pid(pid_speed);
    reset_pid(pid_yaw_angle);
    reset_pid(pid_yaw_gyro);
    reset_pid(pid_lqr_u);
    reset_pid(pid_zeropoint);
}

void balance_init(void) {
    balance_set_params(&DEFAULT_PARAMS);
}

void balance_step(float dt) {
    /*
     * Translated line-by-line from BalanceController::step()
     * Logic preserved exactly
     */

    /* ===== 1. Safety check ===== */
    if (!g_cmd.balance_en || !g_imu.valid || !g_wheel.valid) {
        g_out.motor_l = 0;
        g_out.motor_r = 0;
        g_out.ok = false;
        g_debug.running = false;
        g_debug.fault_flags = !g_cmd.balance_en ? 0x04 : 0x02;
        reset_all_pids();
        return;
    }

    float pitch_deg = g_imu.pitch;

    /* ===== 2. Tilt protection + recovery logic ===== */
    if (fabsf(pitch_deg) > g_params.max_tilt_deg) {
        unctl = 1;
    }
    if (unctl != 0) {
        if (fabsf(pitch_deg) < (g_params.max_tilt_deg * 0.4f)) {
            unctl++;
        }
        if (unctl > 200) {
            unctl = 0;
        }
    }
    if (unctl != 0) {
        g_out.motor_l = 0;
        g_out.motor_r = 0;
        g_out.ok = false;
        g_debug.running = false;
        g_debug.fault_flags = 0x01;
        g_debug.pitch = g_imu.pitch * (PI_F / 180.0f);
        g_debug.pitch_rate = g_imu.gy;
        reset_all_pids();
        return;
    }

    /* ===== 3. Joystick filter + feedforward ===== */
    float tgt_vel_filt = lpf_tgt_vel(g_tgt_vel);

    float accel = 0.0f;
    if (dt > 0.0001f) {
        accel = (tgt_vel_filt - prev_tgt_vel) / dt;
    }
    prev_tgt_vel = tgt_vel_filt;

    float ff_angle = (accel / 9.8f) * (180.0f / PI_F) * g_params.ff_gain;
    ff_angle = constrain(ff_angle, -10.0f, 10.0f);

    float tgt_wheel_vel = tgt_vel_filt / WHEEL_RADIUS;

    /* ===== 4. Angle loop + gyro loop ===== */
    float angle_sp = g_params.pitch_offset + ff_angle;
    float angle_err = pitch_deg - angle_sp;
    float angle_ctrl = pid_angle(angle_err);

    float pitch_rate_dps = g_imu.gy * (180.0f / PI_F);
    float gyro_ctrl = pid_gyro(pitch_rate_dps);

    /* ===== 5. Position loop + velocity loop ===== */
    float distance = (g_wheel.x_l + g_wheel.x_r) * 0.5f;
    float speed = (g_wheel.w_l + g_wheel.w_r) * 0.5f;

    /* Initialize distance zero point */
    if (distance_zero < -200.0f) {
        distance_zero = distance;
    }

    /* Distance zero point management */
    bool has_joy = (fabsf(g_tgt_vel) > 0.01f);

    if (has_joy) {
        distance_zero = distance;
        reset_pid(pid_lqr_u);
    }

    /* Fast push detection */
    if (fabsf(speed) > 30.0f) {
        distance_zero = distance;
    }

    had_joy = has_joy;

    float dist_ctrl = pid_distance(distance - distance_zero);
    float speed_ctrl = pid_speed(speed - tgt_wheel_vel);

    /* ===== 6. Wheel lift detection (disabled) ===== */
    float spd_accel = 0.0f;
    if (dt > 0.0001f) {
        spd_accel = fabsf(speed - spd_prev) / dt;
    }
    spd_prev = speed;

    bool lifted = false;  /* Disabled */

    float lqr_u;
    if (lifted) {
        distance_zero = distance;
        lqr_u = angle_ctrl + gyro_ctrl;
        reset_pid(pid_lqr_u);
    } else {
        lqr_u = angle_ctrl + gyro_ctrl + dist_ctrl + speed_ctrl;
    }

    g_debug.lqr_raw = lqr_u;

    /* ===== 7. CoG adaptation ===== */
    bool cog_act = fabsf(lqr_u) < 5.0f && !has_joy
                && fabsf(speed) < 2.0f && !lifted;

    if (cog_act) {
        lqr_u = pid_lqr_u(lqr_u);
        float zero_adj = pid_zeropoint(lpf_zero(dist_ctrl));
        g_params.pitch_offset -= zero_adj;

        if (g_params.pitch_offset > 10.0f) g_params.pitch_offset = 10.0f;
        if (g_params.pitch_offset < -10.0f) g_params.pitch_offset = -10.0f;

        g_debug.cog_active = true;
        g_debug.cog_adj = zero_adj;
    } else {
        g_debug.cog_active = false;
        g_debug.cog_adj = 0.0f;
        reset_pid(pid_lqr_u);
    }

    g_debug.cog_dist_ctrl = dist_ctrl;
    g_debug.cog_lqr = lqr_u;
    g_debug.cog_speed = speed;
    g_debug.cog_joy = has_joy;

    /* ===== 8. Yaw control ===== */
    float yaw_out = 0.0f;
    bool has_yaw = (fabsf(g_tgt_yaw) > 0.05f);

    if (has_yaw) {
        yaw_tgt += g_tgt_yaw * dt;
        while (yaw_tgt > PI_F)  yaw_tgt -= 2.0f * PI_F;
        while (yaw_tgt < -PI_F) yaw_tgt += 2.0f * PI_F;

        if (!had_yaw) {
            yaw_cur = yaw_tgt;
        }

        yaw_cur += g_imu.gz * dt;
        while (yaw_cur > PI_F)  yaw_cur -= 2.0f * PI_F;
        while (yaw_cur < -PI_F) yaw_cur += 2.0f * PI_F;

        float yaw_err = yaw_tgt - yaw_cur;
        if (yaw_err > PI_F)  yaw_err -= 2.0f * PI_F;
        if (yaw_err < -PI_F) yaw_err += 2.0f * PI_F;

        float yaw_angle_ctrl = pid_yaw_angle(yaw_err);
        float yaw_gyro_ctrl = -g_params.yaw_gyro_kp * g_imu.gz;
        yaw_out = yaw_angle_ctrl + yaw_gyro_ctrl;
    } else {
        yaw_out = -g_params.yaw_gyro_kp * g_imu.gz;
        yaw_cur = 0;
        yaw_tgt = 0;
        reset_pid(pid_yaw_angle);
    }
    had_yaw = has_yaw;

    /* ===== 9. Differential output ===== */
    float l_cmd = -0.5f * (lqr_u + yaw_out);
    float r_cmd = -0.5f * (lqr_u - yaw_out);

    g_out.motor_l = l_cmd;
    g_out.motor_r = r_cmd;
    g_out.ok = true;

    /* ===== 10. Update debug data ===== */
    g_debug.ax = g_imu.ax;
    g_debug.ay = g_imu.ay;
    g_debug.az = g_imu.az;
    g_debug.gx = g_imu.gx;
    g_debug.gy = g_imu.gy;
    g_debug.gz = g_imu.gz;

    g_debug.pitch = g_imu.pitch * (PI_F / 180.0f);
    g_debug.pitch_rate = g_imu.gy;
    g_debug.roll = g_imu.roll * (PI_F / 180.0f);
    g_debug.yaw_rate = g_imu.gz;

    g_debug.wheel_vel = speed;
    g_debug.w_l = g_wheel.w_l;
    g_debug.w_r = g_wheel.w_r;
    g_debug.x_l = g_wheel.x_l;
    g_debug.x_r = g_wheel.x_r;

    g_debug.angle_ctrl = angle_ctrl;
    g_debug.gyro_ctrl = gyro_ctrl;
    g_debug.distance_ctrl = dist_ctrl;
    g_debug.speed_ctrl = speed_ctrl;
    g_debug.lqr_comp = lqr_u;
    g_debug.yaw_out = yaw_out;

    g_debug.pitch_offset = g_params.pitch_offset;
    g_debug.distance_zero = distance_zero;
    g_debug.heading = yaw_cur;

    g_debug.motor_l = l_cmd;
    g_debug.motor_r = r_cmd;

    g_debug.running = true;
    g_debug.fault_flags = 0;
    g_debug.lifted = lifted;
}

void balance_reset(void) {
    reset_all_pids();
    yaw_cur = 0;
    yaw_tgt = 0;
    distance_zero = -256.0f;
    spd_prev = 0;
    prev_tgt_vel = 0;
    unctl = 0;
    had_joy = false;
    had_yaw = false;
}

void balance_get_debug(struct BalanceDebug* out) {
    *out = g_debug;
}

void balance_get_params(struct BalanceParams* out) {
    *out = g_params;
}

void balance_set_params(const struct BalanceParams* p) {
    g_params = *p;

    pid_angle.P     = p->angle_kp;
    pid_gyro.P      = p->gyro_kp;
    pid_distance.P  = p->distance_kp;
    pid_speed.P     = p->speed_kp;
    pid_yaw_angle.P = p->yaw_angle_kp;
    pid_yaw_gyro.P  = p->yaw_gyro_kp;
    pid_lqr_u.P     = p->lqr_u_kp;
    pid_lqr_u.I     = p->lqr_u_ki;
    pid_zeropoint.P = p->zeropoint_kp;

    lpf_tgt_vel.Tf = p->lpf_target_vel_tf;
    lpf_zero.Tf    = p->lpf_zeropoint_tf;

    float lim = p->pid_limit;
    pid_angle.limit     = lim;
    pid_gyro.limit      = lim;
    pid_distance.limit  = lim;
    pid_speed.limit     = lim;
    pid_yaw_angle.limit = lim;
    pid_yaw_gyro.limit  = lim;
    pid_lqr_u.limit     = lim;
}
