/*
 * main.cpp - Balance robot main program
 *
 * Hardware instances + FreeRTOS task scheduling + Serial commands
 */

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "robot.h"
#include "pins.h"
#include "wifi_ctrl.h"

/*
 * Global state instances
 */
struct BalanceParams g_params = DEFAULT_PARAMS;
struct ImuState       g_imu = {};
struct WheelState     g_wheel = {};
struct BalanceOutput  g_out = {};
struct CommandState   g_cmd = {
    .balance_en   = true,
    .motor_en     = true,
    .req_mot_mode = 0,
    .req_trq_mode = 0,
    .apply_mode   = true,
    .manual_tgt   = 0.0f,
    .v_limit      = 0.0f,
    .t_limit      = 8.0f,
    .apply_limits = true,
    .apply_pid    = false,
    .pid_p        = 0.2f,
    .pid_i        = 20.0f,
    .pid_d        = 0.001f,
};
struct BalanceDebug   g_debug = {};
volatile float        g_tgt_vel = 0.0f;
volatile float        g_tgt_yaw = 0.0f;

/*
 * Hardware instances
 */
BLDCMotor motor_left(POLE_PAIRS);
BLDCMotor motor_right(POLE_PAIRS);
BLDCDriver3PWM driver_left(PIN_PWM_U, PIN_PWM_V, PIN_PWM_W, PIN_EN);
BLDCDriver3PWM driver_right(PIN_PWM_U2, PIN_PWM_V2, PIN_PWM_W2, PIN_EN2);
MagneticSensorI2C sensor_left(AS5600_I2C);
MagneticSensorI2C sensor_right(AS5600_I2C);

TwoWire Wire2(1);
SemaphoreHandle_t wire_mutex = nullptr;

/*
 * LED task
 */
static void ledTask(void*) {
    pinMode(LED_PIN, OUTPUT);
    while (true) {
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(LED_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/*
 * IMU task - 200Hz
 */
static void imuTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(5);
    TickType_t last = xTaskGetTickCount();

    while (true) {
        xSemaphoreTake(wire_mutex, portMAX_DELAY);
        imu_read();
        xSemaphoreGive(wire_mutex);

        vTaskDelayUntil(&last, period);
    }
}

/*
 * Balance task - 200Hz
 */
static void balanceTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(5);
    TickType_t last = xTaskGetTickCount();
    uint32_t lastUs = micros();

    balance_init();
    balance_reset();

    while (true) {
        uint32_t nowUs = micros();
        float dt = (nowUs - lastUs) * 1e-6f;
        if (!(dt > 0.0f && dt < 0.1f)) dt = 0.005f;
        lastUs = nowUs;

        balance_step(dt);

        vTaskDelayUntil(&last, period);
    }
}

/*
 * FOC task - 1kHz
 */
static void focTask(void*) {
    /* Left motor init (Wire - shared with IMU, needs mutex) */
    xSemaphoreTake(wire_mutex, portMAX_DELAY);
    sensor_left.init(&Wire);
    motor_left.linkSensor(&sensor_left);

    driver_left.voltage_power_supply = SUPPLY_VOLTAGE;
    driver_left.init();
    motor_left.linkDriver(&driver_left);

    motor_left.controller = MotionControlType::torque;
    motor_left.torque_controller = TorqueControlType::voltage;
    motor_left.voltage_limit = VOLTAGE_LIMIT_INIT;
    motor_left.velocity_limit = 100.0f;

    motor_left.init();
    bool lfoc_ok = motor_left.initFOC();
    xSemaphoreGive(wire_mutex);

    if (!lfoc_ok) {
        Serial.println("Left FOC init failed!");
        vTaskDelete(nullptr);
        return;
    }

    /* Right motor init (Wire2 - independent) */
    sensor_right.init(&Wire2);
    motor_right.linkSensor(&sensor_right);

    driver_right.voltage_power_supply = SUPPLY_VOLTAGE;
    driver_right.init();
    motor_right.linkDriver(&driver_right);

    motor_right.controller = MotionControlType::torque;
    motor_right.torque_controller = TorqueControlType::voltage;
    motor_right.voltage_limit = VOLTAGE_LIMIT_INIT;
    motor_right.velocity_limit = 100.0f;

    motor_right.init();
    if (!motor_right.initFOC()) {
        Serial.println("Right FOC init failed!");
        vTaskDelete(nullptr);
        return;
    }

    Serial.println("FOC ready");

    const TickType_t period = pdMS_TO_TICKS(1);
    TickType_t last = xTaskGetTickCount();
    uint32_t lastPrintMs = 0;
    bool lastMotorEnable = false;

    while (true) {
        /* Motor enable/disable */
        bool me = g_cmd.motor_en;
        if (me != lastMotorEnable) {
            lastMotorEnable = me;
            if (me) {
                driver_left.enable(); motor_left.enable();
                driver_right.enable(); motor_right.enable();
            } else {
                motor_left.disable(); driver_left.disable();
                motor_right.disable(); driver_right.disable();
            }
        }

        /* Apply mode changes */
        if (g_cmd.apply_mode) {
            if (g_cmd.req_mot_mode == 0) {
                motor_left.controller = MotionControlType::torque;
                motor_right.controller = MotionControlType::torque;
            } else if (g_cmd.req_mot_mode == 1) {
                motor_left.controller = MotionControlType::velocity;
                motor_right.controller = MotionControlType::velocity;
            } else {
                motor_left.controller = MotionControlType::angle;
                motor_right.controller = MotionControlType::angle;
            }

            if (g_cmd.req_trq_mode == 0) {
                motor_left.torque_controller = TorqueControlType::voltage;
                motor_right.torque_controller = TorqueControlType::voltage;
            } else if (g_cmd.req_trq_mode == 2) {
                motor_left.torque_controller = TorqueControlType::foc_current;
                motor_right.torque_controller = TorqueControlType::foc_current;
            }

            g_cmd.apply_mode = false;
        }

        /* Apply limits */
        if (g_cmd.apply_limits) {
            motor_left.voltage_limit = g_cmd.t_limit;
            motor_right.voltage_limit = g_cmd.t_limit;

            if (g_cmd.v_limit > 0) {
                motor_left.velocity_limit = g_cmd.v_limit;
                motor_right.velocity_limit = g_cmd.v_limit;
            }

            g_cmd.apply_limits = false;
        }

        /* Apply PID */
        if (g_cmd.apply_pid) {
            motor_left.PID_velocity.P = g_cmd.pid_p;
            motor_left.PID_velocity.I = g_cmd.pid_i;
            motor_left.PID_velocity.D = g_cmd.pid_d;

            motor_right.PID_velocity.P = g_cmd.pid_p;
            motor_right.PID_velocity.I = g_cmd.pid_i;
            motor_right.PID_velocity.D = g_cmd.pid_d;

            g_cmd.apply_pid = false;
        }

        /* FOC loop */
        xSemaphoreTake(wire_mutex, portMAX_DELAY);
        motor_left.loopFOC();
        xSemaphoreGive(wire_mutex);
        motor_right.loopFOC();

        /* Update wheel state */
        {
            constexpr float VEL_ALPHA = 0.2f;
            float wL_raw = sensor_left.getVelocity();   /* Left wheel: no inversion */
            float wR_raw = -sensor_right.getVelocity(); /* Right wheel: inverted, consistent with main */

            g_wheel.w_l += VEL_ALPHA * (wL_raw - g_wheel.w_l);
            g_wheel.w_r += VEL_ALPHA * (wR_raw - g_wheel.w_r);

            g_wheel.x_l = sensor_left.getAngle();    /* Left wheel: no inversion */
            g_wheel.x_r = -sensor_right.getAngle();  /* Right wheel: inverted */
            g_wheel.valid = true;
        }

        /* Select target */
        float ltarget = 0.0f;
        float rtarget = 0.0f;

        if (g_cmd.balance_en && g_out.ok) {
            motor_left.controller = MotionControlType::torque;
            motor_right.controller = MotionControlType::torque;
            motor_left.torque_controller = TorqueControlType::voltage;
            motor_right.torque_controller = TorqueControlType::voltage;

            ltarget = g_out.motor_l;
            rtarget = g_out.motor_r;
        } else {
            ltarget = g_cmd.manual_tgt;
            rtarget = g_cmd.manual_tgt;
        }

        motor_left.move(ltarget);
        motor_right.move(rtarget);

        /* 1Hz debug output */
        uint32_t ms = millis();
        if (ms - lastPrintMs >= 1000) {
            lastPrintMs = ms;
            Serial.printf("MOT,ctrl=%d,ltgt=%.3f,lvel=%.3f,rtgt=%.3f,rvel=%.3f\n",
                (int)motor_left.controller, ltarget, sensor_left.getVelocity(),
                rtarget, sensor_right.getVelocity());
        }

        vTaskDelayUntil(&last, period);
    }
}

/*
 * I2C bus scan
 */
static void scanBus(TwoWire& w, const char* name) {
    Serial.printf("SCAN,%s,begin\n", name);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        w.beginTransmission(addr);
        if (w.endTransmission() == 0) {
            Serial.printf("SCAN,%s,addr=0x%02X\n", name, addr);
            found++;
        }
    }
    Serial.printf("SCAN,%s,found=%d\n", name, found);
}

/*
 * Parameter persistence
 */
static bool loadBalanceParams(struct BalanceParams& p) {
    if (!LittleFS.exists("/params/balance.json")) {
        return false;
    }
    File f = LittleFS.open("/params/balance.json", "r");
    if (!f) return false;
    char buf[512];
    size_t n = f.read((uint8_t*)buf, sizeof(buf) - 1);
    buf[n] = '\0';
    f.close();

    int matched = sscanf(buf,
        "{\"angle_kp\":%f,\"gyro_kp\":%f,"
        "\"distance_kp\":%f,\"speed_kp\":%f,"
        "\"yaw_angle_kp\":%f,\"yaw_gyro_kp\":%f,"
        "\"lqr_u_kp\":%f,\"lqr_u_ki\":%f,"
        "\"zeropoint_kp\":%f,"
        "\"lpf_target_vel_tf\":%f,\"lpf_zeropoint_tf\":%f,"
        "\"ff_gain\":%f,"
        "\"max_tilt_deg\":%f,\"pitch_offset\":%f,"
        "\"pid_limit\":%f,"
        "\"lift_accel_thresh\":%f,\"lift_vel_thresh\":%f}",
        &p.angle_kp, &p.gyro_kp,
        &p.distance_kp, &p.speed_kp,
        &p.yaw_angle_kp, &p.yaw_gyro_kp,
        &p.lqr_u_kp, &p.lqr_u_ki,
        &p.zeropoint_kp,
        &p.lpf_target_vel_tf, &p.lpf_zeropoint_tf,
        &p.ff_gain,
        &p.max_tilt_deg, &p.pitch_offset,
        &p.pid_limit,
        &p.lift_accel_thresh, &p.lift_vel_thresh);
    return matched >= 15;
}

static bool saveBalanceParams(struct BalanceParams& p) {
    if (!LittleFS.exists("/params")) {
        LittleFS.mkdir("/params");
    }
    File f = LittleFS.open("/params/balance.json", "w");
    if (!f) return false;
    f.printf("{\"angle_kp\":%.4f,\"gyro_kp\":%.4f,"
             "\"distance_kp\":%.4f,\"speed_kp\":%.4f,"
             "\"yaw_angle_kp\":%.4f,\"yaw_gyro_kp\":%.4f,"
             "\"lqr_u_kp\":%.4f,\"lqr_u_ki\":%.4f,"
             "\"zeropoint_kp\":%.6f,"
             "\"lpf_target_vel_tf\":%.3f,\"lpf_zeropoint_tf\":%.3f,"
             "\"ff_gain\":%.3f,"
             "\"max_tilt_deg\":%.2f,\"pitch_offset\":%.4f,"
             "\"pid_limit\":%.2f,"
             "\"lift_accel_thresh\":%.2f,\"lift_vel_thresh\":%.2f}",
             p.angle_kp, p.gyro_kp,
             p.distance_kp, p.speed_kp,
             p.yaw_angle_kp, p.yaw_gyro_kp,
             p.lqr_u_kp, p.lqr_u_ki,
             p.zeropoint_kp,
             p.lpf_target_vel_tf, p.lpf_zeropoint_tf,
             p.ff_gain,
             p.max_tilt_deg, p.pitch_offset,
             p.pid_limit,
             p.lift_accel_thresh, p.lift_vel_thresh);
    f.close();
    return true;
}

/*
 * Serial command handler
 */
static void handleSerialCmd(const char* cmd) {
    if (strlen(cmd) == 0) return;

    if (strcmp(cmd, "help") == 0) {
        Serial.println("\n=== Serial Commands ===");
        Serial.println("angle_kp=6.5    - set parameter");
        Serial.println("angle_kp?       - query parameter");
        Serial.println("dump            - show all parameters");
        Serial.println("telem           - show current telemetry");
        Serial.println("save            - save params to flash");
        Serial.println("reset           - reset controller");
        Serial.println("");
        Serial.println("Params: angle_kp, gyro_kp, distance_kp, speed_kp,");
        Serial.println("         yaw_angle_kp, yaw_gyro_kp, lqr_u_kp, lqr_u_ki,");
        Serial.println("         zeropoint_kp, max_tilt_deg, pitch_offset, pid_limit");
        return;
    }

    if (strcmp(cmd, "dump") == 0) {
        Serial.println("\n=== Current Params ===");
        Serial.printf("angle_kp=%.2f\n", g_params.angle_kp);
        Serial.printf("gyro_kp=%.4f\n", g_params.gyro_kp);
        Serial.printf("distance_kp=%.2f\n", g_params.distance_kp);
        Serial.printf("speed_kp=%.4f\n", g_params.speed_kp);
        Serial.printf("yaw_angle_kp=%.2f\n", g_params.yaw_angle_kp);
        Serial.printf("yaw_gyro_kp=%.4f\n", g_params.yaw_gyro_kp);
        Serial.printf("lqr_u_kp=%.2f\n", g_params.lqr_u_kp);
        Serial.printf("lqr_u_ki=%.2f\n", g_params.lqr_u_ki);
        Serial.printf("zeropoint_kp=%.4f\n", g_params.zeropoint_kp);
        Serial.printf("max_tilt_deg=%.1f\n", g_params.max_tilt_deg);
        Serial.printf("pitch_offset=%.2f\n", g_params.pitch_offset);
        Serial.printf("pid_limit=%.2f\n", g_params.pid_limit);
        return;
    }

    if (strcmp(cmd, "telem") == 0) {
        Serial.println("\n=== Telemetry ===");
        Serial.printf("PITCH:%.2fdeg  ROLL:%.2fdeg  YAW_RATE:%.2fdeg/s\n",
            g_debug.pitch * 180.0f / 3.14159f,
            g_debug.roll * 180.0f / 3.14159f,
            g_debug.yaw_rate * 180.0f / 3.14159f);
        Serial.printf("PITCH_RATE:%.2frad/s  PITCH_OFFSET:%.2fdeg\n",
            g_debug.pitch_rate, g_debug.pitch_offset);
        Serial.printf("w_l:%.2f  w_r:%.2f rad/s  |  x_l:%.1f  x_r:%.1f rad\n",
            g_debug.w_l, g_debug.w_r, g_debug.x_l, g_debug.x_r);
        Serial.printf("DIST_ZERO:%.2f rad\n", g_debug.distance_zero);
        Serial.printf("ANGLE:%.2f  GYRO:%.2f  DIST:%.2f  SPD:%.2f\n",
            g_debug.angle_ctrl, g_debug.gyro_ctrl,
            g_debug.distance_ctrl, g_debug.speed_ctrl);
        Serial.printf("LQR_RAW:%.2f  LQR_COMP:%.2f\n",
            g_debug.lqr_raw, g_debug.lqr_comp);
        Serial.printf("MOTOR_L:%.2fV  MOTOR_R:%.2fV\n",
            g_debug.motor_l, g_debug.motor_r);
        Serial.printf("STATE:%s  FAULT:0x%02X  LIFTED:%d\n",
            g_debug.running ? "RUN" : "STOP",
            (unsigned)g_debug.fault_flags, g_debug.lifted ? 1 : 0);
        return;
    }

    if (strcmp(cmd, "save") == 0) {
        bool ok = saveBalanceParams(g_params);
        if (ok) {
            Serial.println("Parameters saved to flash");
        } else {
            Serial.println("ERROR: Failed to save");
        }
        return;
    }

    if (strcmp(cmd, "reset") == 0) {
        balance_reset();
        Serial.println("Controller reset");
        return;
    }

    /* Parameter set/query */
    char* eq = strchr((char*)cmd, '=');
    char* q = strchr((char*)cmd, '?');

    char key[32] = {0};
    float value = 0;
    bool isQuery = false;

    if (q && (!eq || q < eq)) {
        int len = q - cmd;
        if (len > 0 && len < 31) {
            memcpy(key, cmd, len);
            key[len] = '\0';
        }
        isQuery = true;
    } else if (eq) {
        int len = eq - cmd;
        if (len > 0 && len < 31) {
            memcpy(key, cmd, len);
            key[len] = '\0';
        }
        value = atof(eq + 1);
    } else {
        Serial.println("Unknown command. Type 'help'");
        return;
    }

    /* Match parameter */
    bool found = true;
    float* pfield = nullptr;

    if (strcmp(key, "angle_kp") == 0)      pfield = &g_params.angle_kp;
    else if (strcmp(key, "gyro_kp") == 0)   pfield = &g_params.gyro_kp;
    else if (strcmp(key, "distance_kp") == 0) pfield = &g_params.distance_kp;
    else if (strcmp(key, "speed_kp") == 0)  pfield = &g_params.speed_kp;
    else if (strcmp(key, "yaw_angle_kp") == 0) pfield = &g_params.yaw_angle_kp;
    else if (strcmp(key, "yaw_gyro_kp") == 0) pfield = &g_params.yaw_gyro_kp;
    else if (strcmp(key, "lqr_u_kp") == 0)   pfield = &g_params.lqr_u_kp;
    else if (strcmp(key, "lqr_u_ki") == 0)   pfield = &g_params.lqr_u_ki;
    else if (strcmp(key, "zeropoint_kp") == 0) pfield = &g_params.zeropoint_kp;
    else if (strcmp(key, "max_tilt_deg") == 0) pfield = &g_params.max_tilt_deg;
    else if (strcmp(key, "pitch_offset") == 0) pfield = &g_params.pitch_offset;
    else if (strcmp(key, "pid_limit") == 0)  pfield = &g_params.pid_limit;
    else found = false;

    if (found && pfield) {
        if (!isQuery) {
            *pfield = value;
            balance_set_params(&g_params);
            Serial.printf("Set %s = %.4f\n", key, value);
        } else {
            Serial.printf("%s=%.4f\n", key, *pfield);
        }
    } else {
        Serial.printf("Unknown param: %s\n", key);
    }
}

/*
 * setup
 */
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== Boot ===");

    SimpleFOCDebug::enable(&Serial);

    /* I2C buses */
    Serial.println("I2C init...");
    Wire.begin((int)PIN_I2C_SDA, (int)PIN_I2C_SCL);
    Wire.setClock(400000);
    Wire2.begin((int)PIN_I2C_SDA2, (int)PIN_I2C_SCL2);
    Wire2.setClock(400000);
    Serial.println("I2C ready");

    scanBus(Wire, "Wire(3,9)");
    scanBus(Wire2, "Wire2(2,1)");

    /* Create mutex */
    wire_mutex = xSemaphoreCreateMutex();

    /* Initialize IMU */
    imu_init();

    /* WiFi debug */
    wifi_ctrl_init();

    /* OTA */
    ArduinoOTA.setHostname("balancebot");
    ArduinoOTA.setPassword("balancebot");
    ArduinoOTA.begin();
    Serial.println("OTA Ready");

    /* Use code-default params */
    Serial.println("Using code-default balance params");

    /* Start tasks */
    xTaskCreatePinnedToCore(ledTask,     "LED", 2048, nullptr, 0, nullptr, 0);
    xTaskCreatePinnedToCore(imuTask,     "IMU", 8192, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(balanceTask, "BAL", 8192, nullptr, 4, nullptr, 0);
    xTaskCreatePinnedToCore(wifi_ctrl_task, "WST", 8192, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(focTask,     "FOC", 8192, nullptr, 5, nullptr, 1);

    Serial.println("Tasks started");
}

/*
 * loop
 */
void loop() {
    ArduinoOTA.handle();

    /* Serial commands */
    static char cmdbuf[64];
    static int cmdlen = 0;
    while (Serial.available() && cmdlen < 63) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdlen > 0) {
                cmdbuf[cmdlen] = '\0';
                handleSerialCmd(cmdbuf);
                cmdlen = 0;
            }
        } else {
            cmdbuf[cmdlen++] = c;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}
