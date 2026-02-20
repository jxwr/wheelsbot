#include "wifi_debug.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include "app_context.h"
#include "pins.h"

using namespace wheelsbot::control;

// ============================================================
// File-scope context pointer (set at runtime by wifi_debug_init)
// ============================================================
static AppContext* s_ctx = nullptr;

// ============================================================
// Server objects
// ============================================================
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

// ============================================================
// Persistence helpers
// ============================================================
static constexpr const char* PARAMS_PATH = "/params/balance.json";

static bool saveBalanceParams(const wheelsbot::control::BalanceController::Params& p) {
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

static bool loadBalanceParams(wheelsbot::control::BalanceController::Params& p) {
  if (!LittleFS.exists(PARAMS_PATH)) {
    return false;
  }

  File f = LittleFS.open(PARAMS_PATH, "r");
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

  return matched == 17;
}

// ============================================================
// JSON helpers (no ArduinoJson — hand-rolled with snprintf)
// ============================================================

static void sendTelemetry() {
  if (!s_ctx) return;

  // Check if any client is connected and ready
  bool has_ready_client = false;
  for (auto& c : ws.getClients()) {
    if (c->status() == WS_CONNECTED && c->canSend()) {
      has_ready_client = true;
      break;
    }
  }
  if (!has_ready_client) return;

  BalanceDebug dbg;
  s_ctx->balance.getDebug(dbg);

  // 计算角度 (degrees)
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
    "\"wv\":%.4f,\"wL\":%.4f,\"wR\":%.4f,\"xL\":%.4f,\"xR\":%.4f,"
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
    dbg.wheel_velocity, dbg.wL, dbg.wR, dbg.xL, dbg.xR,
    dbg.angle_contribution, dbg.gyro_contribution,
    dbg.distance_contribution, dbg.speed_contribution,
    dbg.lqr_u_raw, dbg.lqr_u_compensated,
    dbg.yaw_output, dbg.heading,
    dbg.pitch_offset, dbg.distance_zeropoint,
    dbg.left_motor, dbg.right_motor,
    dbg.running ? 1 : 0, (unsigned)dbg.fault_flags,
    dbg.wheel_lifted ? 1 : 0,
    dbg.cog_adapt_active ? 1 : 0, dbg.cog_distance_ctrl, dbg.cog_adjustment,
    dbg.cog_lqr_u, dbg.cog_speed, dbg.cog_has_joystick ? 1 : 0,
    s_ctx->target_linear_vel);

  // Double-check before sending
  if (ws.count() > 0) {
    ws.textAll(buf);
  }
}

static void sendParams(AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  BalanceController::Params p;
  s_ctx->balance.getParams(p);

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
    (float)s_ctx->cmd_state.req_tlimit, (float)s_ctx->cmd_state.req_vlimit,
    (float)s_ctx->cmd_state.req_pid_p, (float)s_ctx->cmd_state.req_pid_i, (float)s_ctx->cmd_state.req_pid_d,
    s_ctx->cmd_state.motor_enable  ? "true" : "false",
    s_ctx->cmd_state.balance_enable ? "true" : "false",
    (float)s_ctx->cmd_state.manual_target,
    (int)s_ctx->cmd_state.req_motion_mode, (int)s_ctx->cmd_state.req_torque_mode);

  client->text(buf);
}

// ============================================================
// Command parser
// ============================================================

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

static void handleSetAngle(const char* buf, AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  BalanceController::Params p;
  s_ctx->balance.getParams(p);

  if      (strcmp(key, "angle_kp") == 0)    p.angle_kp = val;
  else if (strcmp(key, "gyro_kp") == 0)     p.gyro_kp = val;

  s_ctx->balance.setParams(p);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_angle\",\"key\":\"%s\",\"value\":%.4f}", key, val);
  client->text(ack);
}

static void handleSetVelocity(const char* buf, AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  BalanceController::Params p;
  s_ctx->balance.getParams(p);

  if      (strcmp(key, "distance_kp") == 0)  p.distance_kp = val;
  else if (strcmp(key, "speed_kp") == 0)     p.speed_kp = val;

  s_ctx->balance.setParams(p);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_velocity\",\"key\":\"%s\",\"value\":%.4f}", key, val);
  client->text(ack);
}

static void handleSetYaw(const char* buf, AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  BalanceController::Params p;
  s_ctx->balance.getParams(p);

  if      (strcmp(key, "yaw_angle_kp") == 0)  p.yaw_angle_kp = val;
  else if (strcmp(key, "yaw_gyro_kp") == 0)   p.yaw_gyro_kp = val;

  s_ctx->balance.setParams(p);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_yaw\",\"key\":\"%s\",\"value\":%.4f}", key, val);
  client->text(ack);
}

static void handleSetCompensation(const char* buf, AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  BalanceController::Params p;
  s_ctx->balance.getParams(p);

  if      (strcmp(key, "lqr_u_kp") == 0)      p.lqr_u_kp = val;
  else if (strcmp(key, "lqr_u_ki") == 0)      p.lqr_u_ki = val;
  else if (strcmp(key, "zeropoint_kp") == 0)   p.zeropoint_kp = val;
  else if (strcmp(key, "lpf_target_vel_tf") == 0)  p.lpf_target_vel_tf = val;
  else if (strcmp(key, "lpf_zeropoint_tf") == 0)   p.lpf_zeropoint_tf = val;
  else if (strcmp(key, "ff_gain") == 0)            p.ff_gain = val;

  s_ctx->balance.setParams(p);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_compensation\",\"key\":\"%s\",\"value\":%.6f}", key, val);
  client->text(ack);
}

static void handleSetSafety(const char* buf, AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  BalanceController::Params p;
  s_ctx->balance.getParams(p);

  if      (strcmp(key, "max_tilt_deg") == 0)       p.max_tilt_deg = val;
  else if (strcmp(key, "pitch_offset") == 0)       p.pitch_offset = val;
  else if (strcmp(key, "pid_limit") == 0)          p.pid_limit = val;
  else if (strcmp(key, "lift_accel_thresh") == 0)   p.lift_accel_thresh = val;
  else if (strcmp(key, "lift_vel_thresh") == 0)     p.lift_vel_thresh = val;

  s_ctx->balance.setParams(p);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_safety\",\"key\":\"%s\",\"value\":%.4f}", key, val);
  client->text(ack);
}

static void handleSetFoc(const char* buf, AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  if (strcmp(key, "voltage_limit") == 0) {
    s_ctx->cmd_state.req_tlimit = val;
    s_ctx->cmd_state.req_apply_limits = true;
  } else if (strcmp(key, "velocity_limit") == 0) {
    s_ctx->cmd_state.req_vlimit = val;
    s_ctx->cmd_state.req_apply_limits = true;
  } else if (strcmp(key, "pid_p") == 0) {
    s_ctx->cmd_state.req_pid_p = val;
    s_ctx->cmd_state.req_apply_pid = true;
  } else if (strcmp(key, "pid_i") == 0) {
    s_ctx->cmd_state.req_pid_i = val;
    s_ctx->cmd_state.req_apply_pid = true;
  } else if (strcmp(key, "pid_d") == 0) {
    s_ctx->cmd_state.req_pid_d = val;
    s_ctx->cmd_state.req_apply_pid = true;
  } else if (strcmp(key, "motion_mode") == 0) {
    s_ctx->cmd_state.req_motion_mode = (int)val;
    s_ctx->cmd_state.req_apply_mode = true;
  } else if (strcmp(key, "torque_mode") == 0) {
    s_ctx->cmd_state.req_torque_mode = (int)val;
    s_ctx->cmd_state.req_apply_mode = true;
  }

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_foc\",\"key\":\"%s\",\"value\":%.4f}", key, val);
  client->text(ack);
}

static void handleSetCtrl(const char* buf, AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  if      (strcmp(key, "motor_enable") == 0)    s_ctx->cmd_state.motor_enable   = (val != 0.f);
  else if (strcmp(key, "balance_enable") == 0)  s_ctx->cmd_state.balance_enable = (val != 0.f);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_ctrl\",\"key\":\"%s\",\"value\":%.0f}", key, val);
  client->text(ack);
}

static void handleSetTarget(const char* buf, AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  if (strcmp(key, "linear_vel") == 0) {
    s_ctx->target_linear_vel = val;
  } else if (strcmp(key, "yaw_rate") == 0) {
    s_ctx->target_yaw_rate = val;
  } else {
    s_ctx->cmd_state.manual_target = val;
  }

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_target\",\"key\":\"%s\",\"value\":%.3f}", key, val);
  client->text(ack);
}

static void handleSaveParams(AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  BalanceController::Params p;
  s_ctx->balance.getParams(p);
  bool ok = saveBalanceParams(p);

  char ack[64];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"save_params\",\"ok\":%s}", ok ? "true" : "false");
  client->text(ack);
}

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

// ============================================================
// WebSocket event handler
// ============================================================
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

// ============================================================
// Public API
// ============================================================
void wifi_debug_init(AppContext& ctx) {
  s_ctx = &ctx;

  WiFi.mode(WIFI_AP);
  WiFi.softAP("BalanceBot");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Main page and control page
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.on("/control", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/control.html", "text/html");
  });

  // File upload endpoint for updating data files OTA
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

  // Simple upload form page
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

void wifiDebugTask(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(50); // 20Hz
  TickType_t last = xTaskGetTickCount();

  while (true) {
    ws.cleanupClients();
    sendTelemetry();
    vTaskDelayUntil(&last, period);
  }
}
