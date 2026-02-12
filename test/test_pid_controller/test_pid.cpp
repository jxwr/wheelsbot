#include <unity.h>
#include "control/pid_controller.h"

using namespace wheelsbot::control;

void setUp(void) {
    // 在每个测试前执行
}

void tearDown(void) {
    // 在每个测试后执行
}

// ==================== 基本功能测试 ====================

void test_pid_proportional_only(void) {
    PidController pid("test", 2.0f, 0.0f, 0.0f);
    ControlInput in = {5.0f, 3.0f, 0.0f, 0.01f};  // ref=5, meas=3
    ControlOutput out;

    bool ok = pid.step(in, out);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(out.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f, out.control);  // 2.0 * (5-3) = 4
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out.error);    // 5-3 = 2
}

void test_pid_zero_error(void) {
    PidController pid("test", 2.0f, 0.0f, 0.0f);
    ControlInput in = {5.0f, 5.0f, 0.0f, 0.01f};  // ref=meas
    ControlOutput out;

    pid.step(in, out);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.control);
}

void test_pid_negative_error(void) {
    PidController pid("test", 2.0f, 0.0f, 0.0f);
    ControlInput in = {3.0f, 5.0f, 0.0f, 0.01f};  // ref < meas
    ControlOutput out;

    pid.step(in, out);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, -4.0f, out.control);  // 2.0 * (3-5) = -4
}

// ==================== 积分测试 ====================

void test_pid_integral_accumulation(void) {
    PidController pid("test", 0.0f, 10.0f, 0.0f);
    ControlInput in = {1.0f, 0.0f, 0.0f, 0.01f};  // error=1
    ControlOutput out;

    // 第一步：积分 = 1 * 0.01 = 0.01, 输出 = 10 * 0.01 = 0.1
    pid.step(in, out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, out.control);

    // 第二步：积分 = 0.01 + 0.01 = 0.02, 输出 = 10 * 0.02 = 0.2
    pid.step(in, out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, out.control);

    // 第三步：验证积分器状态
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.02f, pid.getIntegral());
}

void test_pid_integral_windup_limit(void) {
    PidController pid("test", 0.0f, 100.0f, 0.0f);
    pid.setIntegralLimit(0.5f);  // 积分限幅 ±0.5
    ControlInput in = {10.0f, 0.0f, 0.0f, 0.01f};  // error=10
    ControlOutput out;

    // 多次迭代，积分应该被限制
    for (int i = 0; i < 100; i++) {
        pid.step(in, out);
    }

    // 验证积分被限制在 ±0.5
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, pid.getIntegral());
    // 输出 = 100 * 0.5 = 50
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, out.control);
}

void test_pid_integral_zero_when_ki_zero(void) {
    PidController pid("test", 1.0f, 0.0f, 0.0f);
    pid.setIntegralLimit(10.0f);
    ControlInput in = {1.0f, 0.0f, 0.0f, 0.01f};
    ControlOutput out;

    pid.step(in, out);

    // Ki=0时积分器应该保持为0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.getIntegral());
}

// ==================== 输出限幅测试 ====================

void test_pid_output_saturation(void) {
    PidController pid("test", 100.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-5.0f, 5.0f);
    ControlInput in = {10.0f, 0.0f, 0.0f, 0.01f};  // error=10
    ControlOutput out;

    pid.step(in, out);

    // 100 * 10 = 1000, 但应该被限制在 5
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, out.control);
}

void test_pid_output_negative_saturation(void) {
    PidController pid("test", 100.0f, 0.0f, 0.0f);
    pid.setOutputLimits(-5.0f, 5.0f);
    ControlInput in = {0.0f, 10.0f, 0.0f, 0.01f};  // error=-10
    ControlOutput out;

    pid.step(in, out);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, -5.0f, out.control);
}

// ==================== D项测试 ====================

void test_pid_derivative_fallback_to_error_diff(void) {
    // 当measurement_rate=0时，应使用误差微分
    PidController pid("test", 0.0f, 0.0f, 1.0f);
    pid.setDFilterAlpha(1.0f);  // 无滤波，方便测试
    ControlInput in1 = {5.0f, 3.0f, 0.0f, 0.01f};  // error=2
    ControlOutput out;

    pid.step(in1, out);  // 第一次，last_error_=2

    ControlInput in2 = {5.0f, 4.0f, 0.0f, 0.01f};  // error=1, de=-1
    pid.step(in2, out);  // D = 1.0 * (-1/0.01) = -100

    // 输出 = P(1) + D(-100) = -99
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -99.0f, out.control);
}

void test_pid_derivative_use_measurement_rate(void) {
    // 当measurement_rate!=0时，应使用测量微分
    PidController pid("test", 0.0f, 0.0f, 1.0f);
    pid.setDFilterAlpha(1.0f);
    ControlInput in = {5.0f, 3.0f, 2.0f, 0.01f};  // measurement_rate=2
    ControlOutput out;

    pid.step(in, out);

    // D = Kd * (-measurement_rate) = 1.0 * (-2) = -2
    // P = 2.0 * (5-3) = 4 (如果Kp=2)
    // 等等，这里Kp=0
    // D = -2, P = 0, 输出 = -2
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -2.0f, out.control);
}

// ==================== 边界条件测试 ====================

void test_pid_invalid_dt(void) {
    PidController pid("test", 1.0f, 1.0f, 1.0f);
    ControlInput in = {5.0f, 3.0f, 0.0f, 0.0f};  // dt=0，无效
    ControlOutput out;

    bool ok = pid.step(in, out);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_FALSE(out.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.control);
}

void test_pid_large_dt(void) {
    PidController pid("test", 1.0f, 1.0f, 1.0f);
    ControlInput in = {5.0f, 3.0f, 0.0f, 0.2f};  // dt=0.2，超过0.1限制
    ControlOutput out;

    bool ok = pid.step(in, out);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_FALSE(out.valid);
}

void test_pid_reset(void) {
    PidController pid("test", 0.0f, 10.0f, 0.0f);
    ControlInput in = {1.0f, 0.0f, 0.0f, 0.01f};
    ControlOutput out;

    pid.step(in, out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.01f, pid.getIntegral());

    pid.reset();

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.getIntegral());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pid.getLastError());
}

// ==================== 参数获取测试 ====================

void test_pid_getters(void) {
    PidController pid("test", 1.0f, 2.0f, 3.0f);
    pid.setOutputLimits(-5.0f, 5.0f);
    pid.setIntegralLimit(10.0f);
    pid.setDFilterAlpha(0.8f);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, pid.getKp());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, pid.getKi());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.0f, pid.getKd());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, pid.getOutputMax());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -5.0f, pid.getOutputMin());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, pid.getIntegralLimit());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8f, pid.getDFilterAlpha());
}

// ==================== 主函数 ====================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // 基本功能
    RUN_TEST(test_pid_proportional_only);
    RUN_TEST(test_pid_zero_error);
    RUN_TEST(test_pid_negative_error);

    // 积分
    RUN_TEST(test_pid_integral_accumulation);
    RUN_TEST(test_pid_integral_windup_limit);
    RUN_TEST(test_pid_integral_zero_when_ki_zero);

    // 输出限幅
    RUN_TEST(test_pid_output_saturation);
    RUN_TEST(test_pid_output_negative_saturation);

    // D项
    RUN_TEST(test_pid_derivative_fallback_to_error_diff);
    RUN_TEST(test_pid_derivative_use_measurement_rate);

    // 边界条件
    RUN_TEST(test_pid_invalid_dt);
    RUN_TEST(test_pid_large_dt);
    RUN_TEST(test_pid_reset);

    // 参数获取
    RUN_TEST(test_pid_getters);

    return UNITY_END();
}
