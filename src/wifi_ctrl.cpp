/*
 * wifi_ctrl.cpp - WiFi 控制和遥测
 *
 * 使用 robot.h 中的全局状态，不再依赖 AppContext。
 */

#include "wifi_ctrl.h"
#include "robot.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

/* 服务器实例 */
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

/* 参数路径 */
static constexpr const char* PARAMS_PATH = "/params/balance.json";

/*
 * 参数持久化
 */
static bool saveParams(void) {
    struct BalanceParams p;
    balance_get_params(&p);

    if (!LittleFS.exists("/params")) {
        LittleFS.mkdir("/params");
    }

    File f = LittleFS.open(PARAMS_PATH, "w");
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
 * 发送遥测数据
 */
static void sendTelemetry(void) {
    bool has_ready_client = false;
    for (auto& c : ws.getClients()) {
        if (c->status() == WS_CONNECTED && c->canSend()) {
            has_ready_client = true;
            break;
        }
    }
    if (!has_ready_client) return;

    struct BalanceDebug dbg;
    balance_get_debug(&dbg);

    float pitch_deg = dbg.pitch * (180.0f / 3.14159f);
    float roll_deg = dbg.roll * (180.0f / 3.14159f);
    float yaw_deg = dbg.yaw_rate * (180.0f / 3.14159f);

    char buf[1024];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"telem\",\"t\":%lu,"
        "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
        "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
        "\"pitch\":%.4f,\"pitch_deg\":%.2f,\"pr\":%.4f,"
        "\"roll\":%.4f,\"roll_deg\":%.2f,\"yr\":%.4f,"
        "\"wv\":%.4f,\"w_l\":%.4f,\"w_r\":%.4f,\"x_l\":%.4f,\"x_r\":%.4f,"
        "\"angle\":%.4f,\"gyro\":%.4f,\"dist\":%.4f,\"spd\":%.4f,"
        "\"lqr_raw\":%.4f,\"lqr_comp\":%.4f,"
        "\"yaw\":%.4f,\"heading\":%.3f,"
        "\"pofs\":%.4f,\"dzp\":%.2f,"
        "\"lm\":%.3f,\"rm\":%.3f,"
        "\"state\":%d,\"fault\":%u,\"lifted\":%d,"
        "\"cog_active\":%d,\"cog_dist_ctrl\":%.4f,\"cog_adj\":%.6f,"
        "\"cog_lqr_u\":%.4f,\"cog_speed\":%.4f,\"cog_joy\":%d,"
        "\"target_vel\":%.3f}",
        (unsigned long)millis(),
        dbg.ax, dbg.ay, dbg.az,
        dbg.gx, dbg.gy, dbg.gz,
        dbg.pitch, pitch_deg, dbg.pitch_rate,
        dbg.roll, roll_deg, dbg.yaw_rate,
        dbg.wheel_vel, dbg.w_l, dbg.w_r, dbg.x_l, dbg.x_r,
        dbg.angle_ctrl, dbg.gyro_ctrl,
        dbg.distance_ctrl, dbg.speed_ctrl,
        dbg.lqr_raw, dbg.lqr_comp,
        dbg.yaw_out, dbg.heading,
        dbg.pitch_offset, dbg.distance_zero,
        dbg.motor_l, dbg.motor_r,
        dbg.running ? 1 : 0, (unsigned)dbg.fault_flags,
        dbg.lifted ? 1 : 0,
        dbg.cog_active ? 1 : 0, dbg.cog_dist_ctrl, dbg.cog_adj,
        dbg.cog_lqr, dbg.cog_speed, dbg.cog_joy ? 1 : 0,
        (float)g_tgt_vel);

    if (ws.count() > 0) {
        ws.textAll(buf);
    }
}

/*
 * 发送参数
 */
static void sendParams(AsyncWebSocketClient* client) {
    struct BalanceParams p;
    balance_get_params(&p);

    char buf[1024];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"params\",\"balance\":{"
        "\"angle_kp\":%.4f,\"gyro_kp\":%.4f,"
        "\"distance_kp\":%.4f,\"speed_kp\":%.4f,"
        "\"yaw_angle_kp\":%.4f,\"yaw_gyro_kp\":%.4f,"
        "\"lqr_u_kp\":%.4f,\"lqr_u_ki\":%.4f,"
        "\"zeropoint_kp\":%.6f,"
        "\"lpf_target_vel_tf\":%.3f,\"lpf_zeropoint_tf\":%.3f,"
        "\"ff_gain\":%.3f,"
        "\"max_tilt_deg\":%.2f,\"pitch_offset\":%.4f,"
        "\"pid_limit\":%.2f,"
        "\"lift_accel_thresh\":%.2f,\"lift_vel_thresh\":%.2f},"
        "\"foc\":{\"voltage_limit\":%.2f,\"velocity_limit\":%.2f,"
        "\"pid_p\":%.6f,\"pid_i\":%.6f,\"pid_d\":%.6f},"
        "\"ctrl\":{\"motor_enable\":%s,\"balance_enable\":%s,"
        "\"manual_target\":%.3f,\"motion_mode\":%d,\"torque_mode\":%d}}",
        p.angle_kp, p.gyro_kp,
        p.distance_kp, p.speed_kp,
        p.yaw_angle_kp, p.yaw_gyro_kp,
        p.lqr_u_kp, p.lqr_u_ki,
        p.zeropoint_kp,
        p.lpf_target_vel_tf, p.lpf_zeropoint_tf,
        p.ff_gain,
        p.max_tilt_deg, p.pitch_offset,
        p.pid_limit,
        p.lift_accel_thresh, p.lift_vel_thresh,
        (float)g_cmd.t_limit, (float)g_cmd.v_limit,
        (float)g_cmd.pid_p, (float)g_cmd.pid_i, (float)g_cmd.pid_d,
        g_cmd.motor_en  ? "true" : "false",
        g_cmd.balance_en ? "true" : "false",
        (float)g_cmd.manual_tgt,
        (int)g_cmd.req_mot_mode, (int)g_cmd.req_trq_mode);

    client->text(buf);
}

/*
 * JSON 解析辅助
 */
static float parseValue(const char* buf) {
    const char* p = strstr(buf, "\"value\":");
    if (!p) return 0.f;
    p += 8;
    while (*p == ' ') p++;
    if (strncmp(p, "true", 4) == 0) return 1.f;
    if (strncmp(p, "false", 5) == 0) return 0.f;
    return (float)atof(p);
}

static bool parseKey(const char* buf, char* out, size_t maxlen) {
    const char* p = strstr(buf, "\"key\":\"");
    if (!p) return false;
    p += 7;
    const char* end = strchr(p, '"');
    if (!end) return false;
    size_t len = (size_t)(end - p);
    if (len >= maxlen) len = maxlen - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

/*
 * 参数设置处理
 */
static void handleSetAngle(const char* buf, AsyncWebSocketClient* client) {
    char key[32];
    if (!parseKey(buf, key, sizeof(key))) return;
    float val = parseValue(buf);

    struct BalanceParams p;
    balance_get_params(&p);

    if      (strcmp(key, "angle_kp") == 0)    p.angle_kp = val;
    else if (strcmp(key, "gyro_kp") == 0)     p.gyro_kp = val;

    balance_set_params(&p);

    char ack[128];
    snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_angle\",\"key\":\"%s\",\"value\":%.4f}", key, val);
    client->text(ack);
}

static void handleSetVelocity(const char* buf, AsyncWebSocketClient* client) {
    char key[32];
    if (!parseKey(buf, key, sizeof(key))) return;
    float val = parseValue(buf);

    struct BalanceParams p;
    balance_get_params(&p);

    if      (strcmp(key, "distance_kp") == 0)  p.distance_kp = val;
    else if (strcmp(key, "speed_kp") == 0)     p.speed_kp = val;

    balance_set_params(&p);

    char ack[128];
    snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_velocity\",\"key\":\"%s\",\"value\":%.4f}", key, val);
    client->text(ack);
}

static void handleSetYaw(const char* buf, AsyncWebSocketClient* client) {
    char key[32];
    if (!parseKey(buf, key, sizeof(key))) return;
    float val = parseValue(buf);

    struct BalanceParams p;
    balance_get_params(&p);

    if      (strcmp(key, "yaw_angle_kp") == 0)  p.yaw_angle_kp = val;
    else if (strcmp(key, "yaw_gyro_kp") == 0)   p.yaw_gyro_kp = val;

    balance_set_params(&p);

    char ack[128];
    snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_yaw\",\"key\":\"%s\",\"value\":%.4f}", key, val);
    client->text(ack);
}

static void handleSetCompensation(const char* buf, AsyncWebSocketClient* client) {
    char key[32];
    if (!parseKey(buf, key, sizeof(key))) return;
    float val = parseValue(buf);

    struct BalanceParams p;
    balance_get_params(&p);

    if      (strcmp(key, "lqr_u_kp") == 0)          p.lqr_u_kp = val;
    else if (strcmp(key, "lqr_u_ki") == 0)          p.lqr_u_ki = val;
    else if (strcmp(key, "zeropoint_kp") == 0)      p.zeropoint_kp = val;
    else if (strcmp(key, "lpf_target_vel_tf") == 0) p.lpf_target_vel_tf = val;
    else if (strcmp(key, "lpf_zeropoint_tf") == 0)  p.lpf_zeropoint_tf = val;
    else if (strcmp(key, "ff_gain") == 0)           p.ff_gain = val;

    balance_set_params(&p);

    char ack[128];
    snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_compensation\",\"key\":\"%s\",\"value\":%.6f}", key, val);
    client->text(ack);
}

static void handleSetSafety(const char* buf, AsyncWebSocketClient* client) {
    char key[32];
    if (!parseKey(buf, key, sizeof(key))) return;
    float val = parseValue(buf);

    struct BalanceParams p;
    balance_get_params(&p);

    if      (strcmp(key, "max_tilt_deg") == 0)       p.max_tilt_deg = val;
    else if (strcmp(key, "pitch_offset") == 0)       p.pitch_offset = val;
    else if (strcmp(key, "pid_limit") == 0)          p.pid_limit = val;
    else if (strcmp(key, "lift_accel_thresh") == 0)  p.lift_accel_thresh = val;
    else if (strcmp(key, "lift_vel_thresh") == 0)    p.lift_vel_thresh = val;

    balance_set_params(&p);

    char ack[128];
    snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_safety\",\"key\":\"%s\",\"value\":%.4f}", key, val);
    client->text(ack);
}

static void handleSetFoc(const char* buf, AsyncWebSocketClient* client) {
    char key[32];
    if (!parseKey(buf, key, sizeof(key))) return;
    float val = parseValue(buf);

    if (strcmp(key, "voltage_limit") == 0) {
        g_cmd.t_limit = val;
        g_cmd.apply_limits = true;
    } else if (strcmp(key, "velocity_limit") == 0) {
        g_cmd.v_limit = val;
        g_cmd.apply_limits = true;
    } else if (strcmp(key, "pid_p") == 0) {
        g_cmd.pid_p = val;
        g_cmd.apply_pid = true;
    } else if (strcmp(key, "pid_i") == 0) {
        g_cmd.pid_i = val;
        g_cmd.apply_pid = true;
    } else if (strcmp(key, "pid_d") == 0) {
        g_cmd.pid_d = val;
        g_cmd.apply_pid = true;
    } else if (strcmp(key, "motion_mode") == 0) {
        g_cmd.req_mot_mode = (int)val;
        g_cmd.apply_mode = true;
    } else if (strcmp(key, "torque_mode") == 0) {
        g_cmd.req_trq_mode = (int)val;
        g_cmd.apply_mode = true;
    }

    char ack[128];
    snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_foc\",\"key\":\"%s\",\"value\":%.4f}", key, val);
    client->text(ack);
}

static void handleSetCtrl(const char* buf, AsyncWebSocketClient* client) {
    char key[32];
    if (!parseKey(buf, key, sizeof(key))) return;
    float val = parseValue(buf);

    if      (strcmp(key, "motor_enable") == 0)    g_cmd.motor_en   = (val != 0.f);
    else if (strcmp(key, "balance_enable") == 0)  g_cmd.balance_en = (val != 0.f);

    char ack[128];
    snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_ctrl\",\"key\":\"%s\",\"value\":%.0f}", key, val);
    client->text(ack);
}

static void handleSetTarget(const char* buf, AsyncWebSocketClient* client) {
    char key[32];
    if (!parseKey(buf, key, sizeof(key))) return;
    float val = parseValue(buf);

    if (strcmp(key, "linear_vel") == 0) {
        g_tgt_vel = val;
    } else if (strcmp(key, "yaw_rate") == 0) {
        g_tgt_yaw = val;
    } else {
        g_cmd.manual_tgt = val;
    }

    char ack[128];
    snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_target\",\"key\":\"%s\",\"value\":%.3f}", key, val);
    client->text(ack);
}

static void handleSaveParams(AsyncWebSocketClient* client) {
    bool ok = saveParams();

    char ack[64];
    snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"save_params\",\"ok\":%s}", ok ? "true" : "false");
    client->text(ack);
}

/*
 * 命令分发
 */
static void handleCommand(uint8_t* data, size_t len, AsyncWebSocketClient* client) {
    char buf[512];
    size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, data, n);
    buf[n] = '\0';

    if      (strstr(buf, "\"set_angle\""))        handleSetAngle(buf, client);
    else if (strstr(buf, "\"set_velocity\""))     handleSetVelocity(buf, client);
    else if (strstr(buf, "\"set_yaw\""))          handleSetYaw(buf, client);
    else if (strstr(buf, "\"set_compensation\"")) handleSetCompensation(buf, client);
    else if (strstr(buf, "\"set_safety\""))       handleSetSafety(buf, client);
    else if (strstr(buf, "\"set_foc\""))          handleSetFoc(buf, client);
    else if (strstr(buf, "\"set_ctrl\""))         handleSetCtrl(buf, client);
    else if (strstr(buf, "\"set_target\""))       handleSetTarget(buf, client);
    else if (strstr(buf, "\"save_params\""))      handleSaveParams(client);
    else if (strstr(buf, "\"get_params\""))       sendParams(client);
}

/*
 * WebSocket 事件处理
 */
static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WS client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        sendParams(client);
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WS client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->opcode == WS_TEXT && info->final && info->index == 0 && info->len == len) {
            handleCommand(data, len, client);
        }
    }
}

/*
 * 初始化
 */
void wifi_ctrl_init(void) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("BalanceBot");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed");
    }

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    server.on("/control", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/control.html", "text/html");
    });

    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest* request) {
        request->send(200, "text/plain", "Upload OK");
    }, [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
        static File fsUploadFile;
        if (!index) {
            Serial.printf("Upload Start: %s\n", filename.c_str());
            if (LittleFS.exists(filename)) {
                LittleFS.remove(filename);
            }
            fsUploadFile = LittleFS.open(filename, "w");
        }
        if (fsUploadFile) {
            fsUploadFile.write(data, len);
        }
        if (final) {
            if (fsUploadFile) {
                fsUploadFile.close();
                Serial.printf("Upload Complete: %s, %u bytes\n", filename.c_str(), index + len);
            }
        }
    });

    server.on("/upload", HTTP_GET, [](AsyncWebServerRequest* request) {
        const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head><meta charset="UTF-8"><title>Upload</title></head>
<body>
<h2>Upload File to LittleFS</h2>
<form method="POST" enctype="multipart/form-data">
<input type="file" name="file"><br><br>
<input type="submit" value="Upload">
</form>
</body>
</html>
)rawliteral";
        request->send(200, "text/html", html);
    });

    server.begin();
    Serial.println("HTTP + WebSocket server started");
    Serial.println("Main page: http://192.168.4.1/");
    Serial.println("Control page: http://192.168.4.1/control");
}

/*
 * WiFi 任务
 */
void wifi_ctrl_task(void* arg) {
    const TickType_t period = pdMS_TO_TICKS(50);
    TickType_t last = xTaskGetTickCount();

    while (true) {
        ws.cleanupClients();
        sendTelemetry();
        vTaskDelayUntil(&last, period);
    }
}
