#include <unity.h>
#include "control/cascade_controller.h"

using namespace wheelsbot::control;

void setUp(void) {}
void tearDown(void) {}

// ==================== 基本状态测试 ====================

void test_cascade_initial_state(void) {
    CascadeController ctrl;

    TEST_ASSERT_FALSE(ctrl.isRunning());
    TEST_ASSERT_EQUAL(0, ctrl.getFaultFlags());
}

void test_cascade_disabled_when_not_enabled(void) {
    CascadeController ctrl;
    CascadeInput in = {};
    in.enabled = false;
    in.sensors_valid = true;
    in.dt = 0.005f;

    CascadeOutput out;
    bool ok = ctrl.step(in, out);

    TEST_ASSERT_TRUE(ok);  // 处理了（返回disabled状态）
    TEST_ASSERT_FALSE(out.valid);
    TEST_ASSERT_FALSE(ctrl.isRunning());
}

void test_cascade_runs_when_enabled(void) {
    CascadeController ctrl;
    CascadeInput in = {};
    in.velocity_reference = 0.0f;
    in.velocity_measurement = 0.0f;
    in.pitch_measurement = 0.0f;  // 直立
    in.pitch_rate = 0.0f;
    in.dt = 0.005f;
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out;
    bool ok = ctrl.step(in, out);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(out.valid);
    TEST_ASSERT_TRUE(ctrl.isRunning());
}

// ==================== 安全检查测试 ====================

void test_cascade_fault_on_tilt_too_large(void) {
    CascadeController ctrl;
    CascadeInput in = {};
    in.pitch_measurement = 0.8f;  // 约46度，超过默认35度限制
    in.dt = 0.005f;
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out;
    ctrl.step(in, out);

    TEST_ASSERT_FALSE(ctrl.isRunning());
    TEST_ASSERT_EQUAL(CASCADE_FAULT_TILT_TOO_LARGE, ctrl.getFaultFlags());
    TEST_ASSERT_FALSE(out.valid);
}

void test_cascade_fault_on_bad_dt(void) {
    CascadeController ctrl;
    CascadeInput in = {};
    in.pitch_measurement = 0.0f;
    in.dt = 0.0f;  // 无效dt
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out;
    ctrl.step(in, out);

    TEST_ASSERT_EQUAL(CASCADE_FAULT_BAD_DT, ctrl.getFaultFlags());
}

void test_cascade_fault_on_large_dt(void) {
    CascadeController ctrl;
    CascadeInput in = {};
    in.pitch_measurement = 0.0f;
    in.dt = 0.2f;  // 太大
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out;
    ctrl.step(in, out);

    TEST_ASSERT_EQUAL(CASCADE_FAULT_BAD_DT, ctrl.getFaultFlags());
}

void test_cascade_fault_on_sensor_timeout(void) {
    CascadeController ctrl;
    ctrl.setMaxTilt(1.0f);  // 放宽倾斜限制以便测试

    CascadeInput in = {};
    in.pitch_measurement = 0.0f;
    in.enabled = true;
    in.sensors_valid = false;  // 传感器无效
    in.dt = 0.05f;  // 50ms每步，累积超时

    CascadeOutput out;

    // 第一步：sensor_bad_time = 0.05（首次调用会重置为0再+0.05）
    ctrl.step(in, out);
    // 此时应该还在运行
    TEST_ASSERT_TRUE(ctrl.isRunning());

    // 第二步：sensor_bad_time = 0.10
    ctrl.step(in, out);
    TEST_ASSERT_TRUE(ctrl.isRunning());

    // 第三步：sensor_bad_time = 0.15
    ctrl.step(in, out);
    TEST_ASSERT_TRUE(ctrl.isRunning());

    // 第四步：sensor_bad_time = 0.20
    ctrl.step(in, out);
    // 刚好等于timeout，还未超过

    // 第五步：sensor_bad_time = 0.25 > 0.2，触发超时
    ctrl.step(in, out);

    TEST_ASSERT_FALSE(ctrl.isRunning());
    TEST_ASSERT_EQUAL(CASCADE_FAULT_SENSOR_LOST, ctrl.getFaultFlags());
}

// ==================== Ramp-in测试 ====================

void test_cascade_ramp_in_starts_at_zero(void) {
    CascadeController ctrl;
    ctrl.setRampTime(0.1f);  // 100ms ramp

    CascadeInput in = {};
    in.velocity_reference = 0.0f;
    in.velocity_measurement = 0.0f;
    in.pitch_measurement = 0.1f;  // 小倾斜产生控制输出
    in.dt = 0.005f;
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out;

    // 第一次调用（rising edge）
    ctrl.step(in, out);

    CascadeDebug dbg;
    ctrl.getDebug(dbg);

    // Ramp-in gain应该从小于1开始
    TEST_ASSERT_TRUE(dbg.enable_gain < 1.0f);
    TEST_ASSERT_TRUE(dbg.enable_gain >= 0.0f);
}

void test_cascade_ramp_in_reaches_full(void) {
    CascadeController ctrl;
    ctrl.setRampTime(0.01f);  // 10ms ramp

    CascadeInput in = {};
    in.velocity_reference = 0.0f;
    in.velocity_measurement = 0.0f;
    in.pitch_measurement = 0.1f;
    in.dt = 0.005f;
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out;

    // 调用多次
    for (int i = 0; i < 10; i++) {
        ctrl.step(in, out);
    }

    CascadeDebug dbg;
    ctrl.getDebug(dbg);

    // Ramp应该完成
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, dbg.enable_gain);
}

// ==================== 控制器组合测试 ====================

void test_cascade_velocity_generates_pitch_cmd(void) {
    CascadeController ctrl;

    // 设置外环增益
    ctrl.velocityLoop().setGains(0.1f, 0.0f, 0.0f);
    ctrl.angleLoop().setGains(0.0f, 0.0f, 0.0f);  // 内环无输出

    CascadeInput in = {};
    in.velocity_reference = 1.0f;   // 期望 1 rad/s
    in.velocity_measurement = 0.0f; // 当前 0
    in.pitch_measurement = 0.0f;
    in.dt = 0.005f;
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out;
    ctrl.step(in, out);

    // pitch_cmd 应该是正的（前倾加速）
    TEST_ASSERT_TRUE(out.pitch_cmd > 0.0f);
}

void test_cascade_angle_tracks_reference(void) {
    CascadeController ctrl;
    ctrl.angleLoop().setGains(10.0f, 0.0f, 0.0f);  // 高P增益

    CascadeInput in = {};
    in.velocity_reference = 0.0f;
    in.velocity_measurement = 0.0f;
    in.pitch_measurement = 0.1f;  // 向后倾斜
    in.pitch_rate = 0.0f;
    in.dt = 0.005f;
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out;
    ctrl.step(in, out);

    // 由于 PID: output = Kp * (ref - meas) = 10 * (0 - 0.1) = -1.0
    // 电机输出应该是负的（这是当前实现的行为）
    // 注意：在实际物理系统中可能需要反转，但这里测试的是实现行为
    TEST_ASSERT_TRUE(out.left_motor < 0.0f);
    TEST_ASSERT_TRUE(out.right_motor < 0.0f);
}

// ==================== 参数设置测试 ====================

void test_cascade_params_round_trip(void) {
    CascadeController ctrl;

    CascadeController::Params p;
    p.angle_kp = 7.5f;
    p.angle_ki = 0.2f;
    p.angle_kd = 0.8f;
    p.angle_d_alpha = 0.9f;
    p.angle_max_out = 8.0f;
    p.angle_integrator_limit = 3.0f;
    p.velocity_kp = 0.15f;
    p.velocity_max_tilt = 0.4f;
    p.max_tilt = 0.8f;
    p.ramp_time = 1.0f;
    p.pitch_offset = 0.05f;

    ctrl.setParams(p);

    CascadeController::Params p2;
    ctrl.getParams(p2);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.angle_kp, p2.angle_kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.angle_ki, p2.angle_ki);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.angle_kd, p2.angle_kd);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.angle_d_alpha, p2.angle_d_alpha);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.angle_max_out, p2.angle_max_out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.angle_integrator_limit, p2.angle_integrator_limit);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.velocity_kp, p2.velocity_kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.velocity_max_tilt, p2.velocity_max_tilt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.max_tilt, p2.max_tilt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.ramp_time, p2.ramp_time);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, p.pitch_offset, p2.pitch_offset);
}

void test_cascade_pitch_offset_applied(void) {
    CascadeController ctrl;
    ctrl.setPitchOffset(0.1f);  // 静态偏移

    // 设置外环使pitch_cmd = offset
    ctrl.velocityLoop().setGains(0.0f, 0.0f, 0.0f);

    CascadeInput in = {};
    in.velocity_reference = 0.0f;
    in.velocity_measurement = 0.0f;
    in.pitch_measurement = 0.0f;
    in.dt = 0.005f;
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out;
    ctrl.step(in, out);

    // pitch_cmd 应该包含 offset
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, out.pitch_cmd);
}

// ==================== Debug输出测试 ====================

void test_cascade_debug_output(void) {
    CascadeController ctrl;
    ctrl.angleLoop().setGains(10.0f, 0.0f, 0.0f);

    CascadeInput in = {};
    in.velocity_reference = 0.0f;
    in.velocity_measurement = 0.0f;
    in.pitch_measurement = 0.2f;  // 误差 = -0.2 (假设pitch_cmd=0)
    in.pitch_rate = 1.5f;
    in.dt = 0.005f;
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out;
    ctrl.step(in, out);

    CascadeDebug dbg;
    ctrl.getDebug(dbg);

    TEST_ASSERT_TRUE(dbg.running);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, dbg.pitch);  // 当前姿态
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, dbg.pitch_rate_used);  // D项输入
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.2f, dbg.pitch_error);  // 误差
    TEST_ASSERT_TRUE(dbg.motor_output != 0.0f);
    TEST_ASSERT_EQUAL(0, dbg.fault_flags);
}

// ==================== 主函数 ====================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // 基本状态
    RUN_TEST(test_cascade_initial_state);
    RUN_TEST(test_cascade_disabled_when_not_enabled);
    RUN_TEST(test_cascade_runs_when_enabled);

    // 安全检查
    RUN_TEST(test_cascade_fault_on_tilt_too_large);
    RUN_TEST(test_cascade_fault_on_bad_dt);
    RUN_TEST(test_cascade_fault_on_large_dt);
    RUN_TEST(test_cascade_fault_on_sensor_timeout);

    // Ramp-in
    RUN_TEST(test_cascade_ramp_in_starts_at_zero);
    RUN_TEST(test_cascade_ramp_in_reaches_full);

    // 控制器组合
    RUN_TEST(test_cascade_velocity_generates_pitch_cmd);
    RUN_TEST(test_cascade_angle_tracks_reference);

    // 参数
    RUN_TEST(test_cascade_params_round_trip);
    RUN_TEST(test_cascade_pitch_offset_applied);

    // Debug
    RUN_TEST(test_cascade_debug_output);

    return UNITY_END();
}
