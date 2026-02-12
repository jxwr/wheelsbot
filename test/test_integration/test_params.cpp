#include <unity.h>
#include "control/cascade_controller.h"
#include <stdio.h>
#include <string.h>

using namespace wheelsbot::control;

// 模拟文件系统用于测试
static char mock_file_buffer[1024];
static bool mock_file_exists = false;

// 模拟保存函数
bool mockSaveParams(const CascadeController::Params& p, const char* filename) {
    int n = snprintf(mock_file_buffer, sizeof(mock_file_buffer),
        "{\"angle_kp\":%.4f,\"angle_ki\":%.4f,\"angle_kd\":%.4f,"
        "\"angle_d_alpha\":%.3f,\"angle_max_out\":%.2f,\"angle_integrator_limit\":%.2f,"
        "\"velocity_kp\":%.4f,\"velocity_max_tilt\":%.3f,"
        "\"max_tilt\":%.3f,\"ramp_time\":%.3f,\"pitch_offset\":%.4f}",
        p.angle_kp, p.angle_ki, p.angle_kd,
        p.angle_d_alpha, p.angle_max_out, p.angle_integrator_limit,
        p.velocity_kp, p.velocity_max_tilt,
        p.max_tilt, p.ramp_time, p.pitch_offset);

    if (n > 0 && n < (int)sizeof(mock_file_buffer)) {
        mock_file_exists = true;
        return true;
    }
    return false;
}

// 模拟加载函数
bool mockLoadParams(CascadeController::Params& p, const char* filename) {
    if (!mock_file_exists) return false;

    int matched = sscanf(mock_file_buffer,
        "{\"angle_kp\":%f,\"angle_ki\":%f,\"angle_kd\":%f,"
        "\"angle_d_alpha\":%f,\"angle_max_out\":%f,\"angle_integrator_limit\":%f,"
        "\"velocity_kp\":%f,\"velocity_max_tilt\":%f,"
        "\"max_tilt\":%f,\"ramp_time\":%f,\"pitch_offset\":%f}",
        &p.angle_kp, &p.angle_ki, &p.angle_kd,
        &p.angle_d_alpha, &p.angle_max_out, &p.angle_integrator_limit,
        &p.velocity_kp, &p.velocity_max_tilt,
        &p.max_tilt, &p.ramp_time, &p.pitch_offset);

    return matched == 11;
}

void setUp(void) {
    mock_file_exists = false;
    memset(mock_file_buffer, 0, sizeof(mock_file_buffer));
}

void tearDown(void) {}

// ==================== 序列化测试 ====================

void test_params_serialization_format(void) {
    CascadeController::Params p;
    p.angle_kp = 6.5f;
    p.angle_ki = 0.1f;
    p.angle_kd = 0.7f;
    p.angle_d_alpha = 0.75f;
    p.angle_max_out = 7.0f;
    p.angle_integrator_limit = 2.5f;
    p.velocity_kp = 0.15f;
    p.velocity_max_tilt = 0.35f;
    p.max_tilt = 0.65f;
    p.ramp_time = 0.6f;
    p.pitch_offset = 0.02f;

    bool ok = mockSaveParams(p, "test.json");
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(mock_file_exists);

    // 验证JSON格式包含所有字段
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"angle_kp\":6.5000"));
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"angle_ki\":0.1000"));
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"angle_kd\":0.7000"));
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"angle_d_alpha\":0.750"));
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"angle_max_out\":7.00"));
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"angle_integrator_limit\":2.50"));
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"velocity_kp\":0.1500"));
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"velocity_max_tilt\":0.350"));
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"max_tilt\":0.650"));
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"ramp_time\":0.600"));
    TEST_ASSERT_NOT_NULL(strstr(mock_file_buffer, "\"pitch_offset\":0.0200"));
}

void test_params_deserialization(void) {
    // 预设JSON
    snprintf(mock_file_buffer, sizeof(mock_file_buffer),
        "{\"angle_kp\":5.5000,\"angle_ki\":0.2000,\"angle_kd\":0.6000,"
        "\"angle_d_alpha\":0.800,\"angle_max_out\":6.00,\"angle_integrator_limit\":1.50,"
        "\"velocity_kp\":0.2500,\"velocity_max_tilt\":0.450,"
        "\"max_tilt\":0.750,\"ramp_time\":0.400,\"pitch_offset\":0.0300}");
    mock_file_exists = true;

    CascadeController::Params p;
    bool ok = mockLoadParams(p, "test.json");

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f, p.angle_kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, p.angle_ki);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.6f, p.angle_kd);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8f, p.angle_d_alpha);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, p.angle_max_out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, p.angle_integrator_limit);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.25f, p.velocity_kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.45f, p.velocity_max_tilt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.75f, p.max_tilt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, p.ramp_time);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.03f, p.pitch_offset);
}

void test_params_round_trip(void) {
    CascadeController::Params original;
    original.angle_kp = 7.1234f;
    original.angle_ki = 0.5678f;
    original.angle_kd = 0.9012f;
    original.angle_d_alpha = 0.654f;
    original.angle_max_out = 8.50f;
    original.angle_integrator_limit = 3.25f;
    original.velocity_kp = 0.3456f;
    original.velocity_max_tilt = 0.278f;
    original.max_tilt = 0.543f;
    original.ramp_time = 0.789f;
    original.pitch_offset = 0.0123f;

    // 保存
    bool save_ok = mockSaveParams(original, "test.json");
    TEST_ASSERT_TRUE(save_ok);

    // 加载
    CascadeController::Params loaded;
    bool load_ok = mockLoadParams(loaded, "test.json");
    TEST_ASSERT_TRUE(load_ok);

    // 验证所有字段
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, original.angle_kp, loaded.angle_kp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, original.angle_ki, loaded.angle_ki);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, original.angle_kd, loaded.angle_kd);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, original.angle_d_alpha, loaded.angle_d_alpha);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, original.angle_max_out, loaded.angle_max_out);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, original.angle_integrator_limit, loaded.angle_integrator_limit);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, original.velocity_kp, loaded.velocity_kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, original.velocity_max_tilt, loaded.velocity_max_tilt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, original.max_tilt, loaded.max_tilt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, original.ramp_time, loaded.ramp_time);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, original.pitch_offset, loaded.pitch_offset);
}

void test_params_load_nonexistent_file(void) {
    CascadeController::Params p;
    bool ok = mockLoadParams(p, "nonexistent.json");

    TEST_ASSERT_FALSE(ok);
}

// ==================== 控制器应用测试 ====================

void test_params_applied_to_controller(void) {
    CascadeController ctrl;

    CascadeController::Params p;
    p.angle_kp = 8.0f;
    p.angle_ki = 0.5f;
    p.angle_kd = 0.8f;
    p.angle_d_alpha = 0.6f;
    p.angle_max_out = 5.0f;
    p.angle_integrator_limit = 1.5f;
    p.velocity_kp = 0.2f;
    p.velocity_max_tilt = 0.4f;
    p.max_tilt = 0.7f;
    p.ramp_time = 0.8f;
    p.pitch_offset = 0.05f;

    ctrl.setParams(p);

    // 验证参数应用到控制器
    CascadeController::Params actual;
    ctrl.getParams(actual);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 8.0f, actual.angle_kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, actual.angle_ki);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.8f, actual.angle_kd);
}

void test_params_affect_controller_behavior(void) {
    CascadeController ctrl1, ctrl2;

    // 两个控制器，不同参数
    CascadeController::Params p1;
    p1.angle_kp = 10.0f;  // 高增益
    p1.angle_ki = 0.0f;
    p1.angle_kd = 0.0f;
    p1.angle_max_out = 10.0f;
    p1.ramp_time = 0.001f;  // 几乎无ramp

    CascadeController::Params p2;
    p2.angle_kp = 5.0f;   // 低增益
    p2.angle_ki = 0.0f;
    p2.angle_kd = 0.0f;
    p2.angle_max_out = 10.0f;
    p2.ramp_time = 0.001f;

    ctrl1.setParams(p1);
    ctrl2.setParams(p2);

    CascadeInput in = {};
    in.velocity_reference = 0.0f;
    in.velocity_measurement = 0.0f;
    in.pitch_measurement = 0.1f;  // 相同误差
    in.dt = 0.005f;
    in.enabled = true;
    in.sensors_valid = true;

    CascadeOutput out1, out2;
    ctrl1.step(in, out1);
    ctrl2.step(in, out2);

    // 高增益控制器的输出应该更大
    TEST_ASSERT_TRUE(fabsf(out1.left_motor) > fabsf(out2.left_motor));
}

// ==================== 默认值测试 ====================

void test_params_default_values(void) {
    CascadeController::Params p;  // 使用默认构造函数

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, p.angle_kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, p.angle_ki);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.6f, p.angle_kd);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.7f, p.angle_d_alpha);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, p.angle_max_out);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, p.angle_integrator_limit);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, p.velocity_kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, p.velocity_max_tilt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 35.0f * 3.14159f / 180.0f, p.max_tilt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, p.ramp_time);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, p.pitch_offset);
}

// ==================== 边界值测试 ====================

void test_params_extreme_values(void) {
    CascadeController::Params p;
    p.angle_kp = 0.0f;  // 零增益
    p.angle_ki = 0.0f;
    p.angle_kd = 0.0f;
    p.angle_d_alpha = 0.0f;  // 完全滤波
    p.angle_max_out = 0.0f;  // 无输出
    p.angle_integrator_limit = 0.0f;  // 无积分
    p.velocity_kp = 0.0f;
    p.velocity_max_tilt = 0.0f;
    p.max_tilt = 0.01f;  // 极严格
    p.ramp_time = 0.0f;  // 无ramp
    p.pitch_offset = 0.0f;

    bool ok = mockSaveParams(p, "extreme.json");
    TEST_ASSERT_TRUE(ok);

    CascadeController::Params loaded;
    bool load_ok = mockLoadParams(loaded, "extreme.json");
    TEST_ASSERT_TRUE(load_ok);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, loaded.angle_kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, loaded.ramp_time);
}

void test_params_large_values(void) {
    CascadeController::Params p;
    p.angle_kp = 1000.0f;
    p.angle_ki = 500.0f;
    p.angle_kd = 100.0f;
    p.angle_max_out = 100.0f;
    p.max_tilt = 1.5f;  // 约86度

    bool ok = mockSaveParams(p, "large.json");
    TEST_ASSERT_TRUE(ok);

    CascadeController::Params loaded;
    bool load_ok = mockLoadParams(loaded, "large.json");
    TEST_ASSERT_TRUE(load_ok);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000.0f, loaded.angle_kp);
}

// ==================== 主函数 ====================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // 序列化
    RUN_TEST(test_params_serialization_format);
    RUN_TEST(test_params_deserialization);
    RUN_TEST(test_params_round_trip);
    RUN_TEST(test_params_load_nonexistent_file);

    // 控制器应用
    RUN_TEST(test_params_applied_to_controller);
    RUN_TEST(test_params_affect_controller_behavior);

    // 默认值
    RUN_TEST(test_params_default_values);

    // 边界值
    RUN_TEST(test_params_extreme_values);
    RUN_TEST(test_params_large_values);

    return UNITY_END();
}
