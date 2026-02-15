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

bool saveBalanceParams(const BalanceController::Params& p) {
  // Ensure directory exists
  if (!LittleFS.exists("/params")) {
    LittleFS.mkdir("/params");
  }

  File f = LittleFS.open(PARAMS_PATH, "w");
  if (!f) return false;

  // Simple JSON serialization
  f.printf("{\"velocity_kp\":%.4f,\"velocity_ki\":%.4f,\"velocity_kd\":%.4f,\"velocity_max_tilt\":%.3f,"
           "\"angle_kp\":%.4f,\"angle_ki\":%.4f,\"angle_gyro_kd\":%.4f,\"angle_d_alpha\":%.3f,\"angle_max_out\":%.2f,"
           "\"yaw_kd\":%.4f,"
           "\"max_tilt\":%.3f,\"ramp_time\":%.3f,\"pitch_offset\":%.4f,"
           "\"pitch_cmd_rate_limit\":%.3f,\"sensor_timeout\":%.3f}",
           p.velocity_kp, p.velocity_ki, p.velocity_kd, p.velocity_max_tilt,
           p.angle_kp, p.angle_ki, p.angle_gyro_kd, p.angle_d_alpha, p.angle_max_out,
           p.yaw_kd,
           p.max_tilt, p.ramp_time, p.pitch_offset,
           p.pitch_cmd_rate_limit, p.sensor_timeout);
  f.close();
  return true;
}

bool loadBalanceParams(BalanceController::Params& p) {
  if (!LittleFS.exists(PARAMS_PATH)) {
    return false;  // No saved params
  }

  File f = LittleFS.open(PARAMS_PATH, "r");
  if (!f) return false;

  char buf[512];
  size_t n = f.read((uint8_t*)buf, sizeof(buf) - 1);
  buf[n] = '\0';
  f.close();

  // Parse JSON format
  int matched = sscanf(buf, "{\"velocity_kp\":%f,\"velocity_ki\":%f,\"velocity_kd\":%f,\"velocity_max_tilt\":%f,"
             "\"angle_kp\":%f,\"angle_ki\":%f,\"angle_gyro_kd\":%f,\"angle_d_alpha\":%f,\"angle_max_out\":%f,"
             "\"yaw_kd\":%f,"
             "\"max_tilt\":%f,\"ramp_time\":%f,\"pitch_offset\":%f,"
             "\"pitch_cmd_rate_limit\":%f,\"sensor_timeout\":%f}",
         &p.velocity_kp, &p.velocity_ki, &p.velocity_kd, &p.velocity_max_tilt,
         &p.angle_kp, &p.angle_ki, &p.angle_gyro_kd, &p.angle_d_alpha, &p.angle_max_out,
         &p.yaw_kd,
         &p.max_tilt, &p.ramp_time, &p.pitch_offset,
         &p.pitch_cmd_rate_limit, &p.sensor_timeout);

  if (matched == 15) return true;

  return false;
}

// ============================================================
// JSON helpers (no ArduinoJson — hand-rolled with snprintf)
// ============================================================


static void sendTelemetry() {
  if (ws.count() == 0 || !s_ctx) return;

  // Skip this frame if any client's send queue is full.
  // Prevents "Too many messages queued" when WiFi is slow or tab is in background.
  for (auto& c : ws.getClients()) {
    if (c->status() == WS_CONNECTED && !c->canSend()) return;
  }

  BalanceDebug dbg;
  s_ctx->balance.getDebug(dbg);

  FrequencyStats freq;
  s_ctx->balance.getFrequencyStats(freq);

  LimitStatus limits;
  s_ctx->balance.getLimitStatus(limits);

  RuntimeStats runtime;
  s_ctx->balance.getRuntimeStats(runtime);

  // Build saturation bitmask (velocity=bit0, angle=bit1, motor=bit2)
  uint8_t saturated = 0;
  if (limits.velocity_saturated) saturated |= 1;
  if (limits.angle_saturated) saturated |= 2;
  if (limits.motor_saturated) saturated |= 4;

  char buf[1024];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"telem\",\"t\":%lu,"
    "\"pitch\":%.4f,\"pitch_deg\":%.2f,\"pr\":%.4f,"
    "\"roll_deg\":%.2f,\"yaw_deg\":%.2f,"
    "\"vel\":%.2f,\"vel_target\":%.2f,\"vel_error\":%.4f,\"vel_i\":%.4f,\"pitch_cmd\":%.4f,"
    "\"pitch_error\":%.4f,\"pitch_i\":%.4f,\"motor\":%.3f,"
    "\"heading\":%.3f,"
    "\"vel_hz\":%.1f,\"ang_hz\":%.1f,\"saturated\":%u,"
    "\"state\":%d,\"fault\":%u,\"eg\":%.2f,"
    "\"runtime\":%lu,\"fault_cnt\":%u,\"max_pitch\":%.3f}",
    (unsigned long)millis(),
    dbg.pitch, (float)(dbg.pitch * 180.0f / 3.14159f), dbg.pitch_rate_used,
    (float)s_ctx->imu_state.roll_deg, (float)s_ctx->imu_state.yaw_deg,
    dbg.velocity_error + (s_ctx->wheel_state.valid ? (s_ctx->wheel_state.wL + s_ctx->wheel_state.wR) * 0.5f : 0.0f),
    dbg.velocity_error + (s_ctx->wheel_state.valid ? (s_ctx->wheel_state.wL + s_ctx->wheel_state.wR) * 0.5f : 0.0f) - dbg.velocity_error,
    dbg.velocity_error, dbg.velocity_integrator, dbg.pitch_cmd,
    dbg.pitch_error, dbg.pitch_integrator, dbg.motor_output,
    s_ctx->heading,
    freq.velocity_hz, freq.angle_hz, saturated,
    dbg.running ? 1 : 0, (unsigned)dbg.fault_flags, dbg.enable_gain,
    runtime.total_runtime_sec, runtime.fault_count_total, runtime.max_pitch_ever);

  ws.textAll(buf);
}

static void sendParams(AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  BalanceController::Params p;
  s_ctx->balance.getParams(p);

  char buf[1024];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"params\",\"balance\":{"
    "\"velocity_kp\":%.4f,\"velocity_ki\":%.4f,\"velocity_kd\":%.4f,\"velocity_max_tilt\":%.3f,"
    "\"angle_kp\":%.4f,\"angle_ki\":%.4f,\"angle_gyro_kd\":%.4f,\"angle_d_alpha\":%.3f,\"angle_max_out\":%.2f,"
    "\"yaw_kd\":%.4f,"
    "\"max_tilt\":%.3f,\"ramp_time\":%.3f,\"pitch_offset\":%.4f,"
    "\"pitch_cmd_rate_limit\":%.3f,\"sensor_timeout\":%.3f},"
    "\"foc\":{\"voltage_limit\":%.2f,\"velocity_limit\":%.2f,"
    "\"pid_p\":%.6f,\"pid_i\":%.6f,\"pid_d\":%.6f},"
    "\"ctrl\":{\"motor_enable\":%s,\"balance_enable\":%s,"
    "\"manual_target\":%.3f,\"motion_mode\":%d,\"torque_mode\":%d}}",
    p.velocity_kp, p.velocity_ki, p.velocity_kd, p.velocity_max_tilt,
    p.angle_kp, p.angle_ki, p.angle_gyro_kd, p.angle_d_alpha, p.angle_max_out,
    p.yaw_kd,
    p.max_tilt, p.ramp_time, p.pitch_offset,
    p.pitch_cmd_rate_limit, p.sensor_timeout,
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

// Extract a float after "value": in the buffer
static float parseValue(const char* buf) {
  const char* p = strstr(buf, "\"value\":");
  if (!p) return 0.f;
  p += 8;
  while (*p == ' ') p++;
  // handle boolean true/false
  if (strncmp(p, "true", 4) == 0) return 1.f;
  if (strncmp(p, "false", 5) == 0) return 0.f;
  return (float)atof(p);
}

// Extract the "key":"..." value
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

  if      (strcmp(key, "angle_kp") == 0)        p.angle_kp = val;
  else if (strcmp(key, "angle_ki") == 0)        p.angle_ki = val;
  else if (strcmp(key, "angle_gyro_kd") == 0)   p.angle_gyro_kd = val;
  else if (strcmp(key, "angle_d_alpha") == 0)   p.angle_d_alpha = val;
  else if (strcmp(key, "angle_max_out") == 0)   p.angle_max_out = val;

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

  if (strcmp(key, "velocity_kp") == 0)          p.velocity_kp = val;
  else if (strcmp(key, "velocity_ki") == 0)     p.velocity_ki = val;
  else if (strcmp(key, "velocity_kd") == 0)     p.velocity_kd = val;
  else if (strcmp(key, "velocity_max_tilt") == 0) p.velocity_max_tilt = val;

  s_ctx->balance.setParams(p);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_velocity\",\"key\":\"%s\",\"value\":%.4f}", key, val);
  client->text(ack);
}

static void handleSetSafety(const char* buf, AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  BalanceController::Params p;
  s_ctx->balance.getParams(p);

  if      (strcmp(key, "max_tilt") == 0)             p.max_tilt = val;
  else if (strcmp(key, "ramp_time") == 0)            p.ramp_time = val;
  else if (strcmp(key, "pitch_offset") == 0)         p.pitch_offset = val;
  else if (strcmp(key, "pitch_cmd_rate_limit") == 0) p.pitch_cmd_rate_limit = val;
  else if (strcmp(key, "sensor_timeout") == 0)       p.sensor_timeout = val;

  s_ctx->balance.setParams(p);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_safety\",\"key\":\"%s\",\"value\":%.4f}", key, val);
  client->text(ack);
}

static void handleSetYaw(const char* buf, AsyncWebSocketClient* client) {
  if (!s_ctx) return;
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  BalanceController::Params p;
  s_ctx->balance.getParams(p);

  if (strcmp(key, "yaw_kd") == 0) p.yaw_kd = val;

  s_ctx->balance.setParams(p);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_yaw\",\"key\":\"%s\",\"value\":%.4f}", key, val);
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
  else if (strcmp(key, "remote_mode") == 0)     s_ctx->remote_mode = (val != 0.f);

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
    // Linear velocity command for position/velocity control
    s_ctx->target_linear_vel = val;
  } else if (strcmp(key, "yaw_rate") == 0) {
    s_ctx->target_yaw_rate = val;
  } else {
    // Legacy single-value target
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

  if      (strstr(buf, "\"set_angle\""))     handleSetAngle(buf, client);
  else if (strstr(buf, "\"set_velocity\""))  handleSetVelocity(buf, client);
  else if (strstr(buf, "\"set_safety\""))    handleSetSafety(buf, client);
  else if (strstr(buf, "\"set_yaw\""))       handleSetYaw(buf, client);
  else if (strstr(buf, "\"set_foc\""))       handleSetFoc(buf, client);
  else if (strstr(buf, "\"set_ctrl\""))      handleSetCtrl(buf, client);
  else if (strstr(buf, "\"set_target\""))    handleSetTarget(buf, client);
  else if (strstr(buf, "\"save_params\""))   handleSaveParams(client);
  else if (strstr(buf, "\"get_params\""))    sendParams(client);
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
      // Start of upload
      Serial.printf("Upload Start: %s\n", filename.c_str());
      // Remove existing file
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
