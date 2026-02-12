#include "wifi_debug.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

#include "shared_state.h"
#include "balance_core.h"
#include "pins.h"

// ============================================================
// Server objects
// ============================================================
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

// ============================================================
// JSON helpers (no ArduinoJson — hand-rolled with snprintf)
// ============================================================

static void sendTelemetry() {
  if (ws.count() == 0) return;

  bc_debug_t dbg;
  bc_get_debug(&g_bc, &dbg);

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"telem\",\"t\":%lu,\"pitch\":%.4f,\"pr\":%.4f,"
    "\"pitch_deg\":%.2f,\"roll_deg\":%.2f,\"yaw_deg\":%.2f,"
    "\"err\":%.4f,\"err_i\":%.4f,\"u_bal\":%.3f,"
    "\"u_l\":%.3f,\"u_r\":%.3f,"
    "\"wL\":%.2f,\"wR\":%.2f,"
    "\"state\":%d,\"fault\":%u,\"eg\":%.2f}",
    (unsigned long)millis(),
    dbg.pitch, dbg.pitch_rate,
    (float)g_imu.pitch_deg, (float)g_imu.roll_deg, (float)g_imu.yaw_deg,
    dbg.err, dbg.err_i, dbg.u_balance,
    dbg.u_left, dbg.u_right,
    (float)g_wheel.wL, (float)g_wheel.wR,
    (int)dbg.state, (unsigned)dbg.fault_flags,
    g_bc.enable_gain);

  ws.textAll(buf);
}

static void sendParams(AsyncWebSocketClient* client) {
  const bc_params_t& p = g_bc.p;

  char buf[1024];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"params\",\"bc\":{"
    "\"Kp\":%.4f,\"Ki\":%.4f,\"Kd\":%.4f,\"d_lpf_alpha\":%.3f,"
    "\"comp_alpha\":%.3f,\"max_out\":%.2f,\"pitch_target\":%.4f,"
    "\"integrator_limit\":%.2f,\"deadband\":%.4f,"
    "\"Kv\":%.4f,\"Kx\":%.4f,\"tilt_cmd_max\":%.3f,"
    "\"bias_learn_k\":%.5f,\"v_gate\":%.3f,\"w_gate\":%.3f,"
    "\"max_tilt\":%.3f,\"sensor_timeout_s\":%.3f,\"ramp_time_s\":%.3f,"
    "\"v2speed_gain\":%.3f,\"yaw2diff_gain\":%.3f},"
    "\"foc\":{\"voltage_limit\":%.2f,\"velocity_limit\":%.2f,"
    "\"pid_p\":%.6f,\"pid_i\":%.6f,\"pid_d\":%.6f},"
    "\"ctrl\":{\"motor_enable\":%s,\"balance_enable\":%s,"
    "\"manual_target\":%.3f,\"motion_mode\":%d,\"torque_mode\":%d}}",
    p.Kp, p.Ki, p.Kd, p.d_lpf_alpha,
    p.comp_alpha, p.max_out, p.pitch_target,
    p.integrator_limit, p.deadband,
    p.Kv, p.Kx, p.tilt_cmd_max,
    p.bias_learn_k, p.v_gate, p.w_gate,
    p.max_tilt, p.sensor_timeout_s, p.ramp_time_s,
    p.v2speed_gain, p.yaw2diff_gain,
    (float)g_cmd.req_tlimit, (float)g_cmd.req_vlimit,
    (float)g_cmd.req_pid_p, (float)g_cmd.req_pid_i, (float)g_cmd.req_pid_d,
    g_cmd.motor_enable  ? "true" : "false",
    g_cmd.balance_enable ? "true" : "false",
    (float)g_cmd.manual_target,
    (int)g_cmd.req_motion_mode, (int)g_cmd.req_torque_mode);

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

static void handleSetBc(const char* buf, AsyncWebSocketClient* client) {
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  bc_params_t p = g_bc.p;

  if      (strcmp(key, "Kp") == 0)              p.Kp = val;
  else if (strcmp(key, "Ki") == 0)              p.Ki = val;
  else if (strcmp(key, "Kd") == 0)              p.Kd = val;
  else if (strcmp(key, "d_lpf_alpha") == 0)     p.d_lpf_alpha = val;
  else if (strcmp(key, "comp_alpha") == 0)       p.comp_alpha = val;
  else if (strcmp(key, "max_out") == 0)          p.max_out = val;
  else if (strcmp(key, "pitch_target") == 0)     p.pitch_target = val;
  else if (strcmp(key, "integrator_limit") == 0) p.integrator_limit = val;
  else if (strcmp(key, "deadband") == 0)         p.deadband = val;
  else if (strcmp(key, "Kv") == 0)              p.Kv = val;
  else if (strcmp(key, "Kx") == 0)              p.Kx = val;
  else if (strcmp(key, "tilt_cmd_max") == 0)     p.tilt_cmd_max = val;
  else if (strcmp(key, "bias_learn_k") == 0)     p.bias_learn_k = val;
  else if (strcmp(key, "v_gate") == 0)           p.v_gate = val;
  else if (strcmp(key, "w_gate") == 0)           p.w_gate = val;
  else if (strcmp(key, "max_tilt") == 0)         p.max_tilt = val;
  else if (strcmp(key, "sensor_timeout_s") == 0) p.sensor_timeout_s = val;
  else if (strcmp(key, "ramp_time_s") == 0)      p.ramp_time_s = val;
  else if (strcmp(key, "v2speed_gain") == 0)     p.v2speed_gain = val;
  else if (strcmp(key, "yaw2diff_gain") == 0)    p.yaw2diff_gain = val;

  bc_set_params(&g_bc, &p);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_bc\",\"key\":\"%s\",\"value\":%.4f}", key, val);
  client->text(ack);
}

static void handleSetFoc(const char* buf, AsyncWebSocketClient* client) {
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  if (strcmp(key, "voltage_limit") == 0) {
    g_cmd.req_tlimit = val;
    g_cmd.req_apply_limits = true;
  } else if (strcmp(key, "velocity_limit") == 0) {
    g_cmd.req_vlimit = val;
    g_cmd.req_apply_limits = true;
  } else if (strcmp(key, "pid_p") == 0) {
    g_cmd.req_pid_p = val;
    g_cmd.req_apply_pid = true;
  } else if (strcmp(key, "pid_i") == 0) {
    g_cmd.req_pid_i = val;
    g_cmd.req_apply_pid = true;
  } else if (strcmp(key, "pid_d") == 0) {
    g_cmd.req_pid_d = val;
    g_cmd.req_apply_pid = true;
  } else if (strcmp(key, "motion_mode") == 0) {
    g_cmd.req_motion_mode = (int)val;
    g_cmd.req_apply_mode = true;
  } else if (strcmp(key, "torque_mode") == 0) {
    g_cmd.req_torque_mode = (int)val;
    g_cmd.req_apply_mode = true;
  }

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_foc\",\"key\":\"%s\",\"value\":%.4f}", key, val);
  client->text(ack);
}

static void handleSetCtrl(const char* buf, AsyncWebSocketClient* client) {
  char key[32];
  if (!parseKey(buf, key, sizeof(key))) return;
  float val = parseValue(buf);

  if      (strcmp(key, "motor_enable") == 0)   g_cmd.motor_enable   = (val != 0.f);
  else if (strcmp(key, "balance_enable") == 0)  g_cmd.balance_enable = (val != 0.f);

  char ack[128];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_ctrl\",\"key\":\"%s\",\"value\":%.0f}", key, val);
  client->text(ack);
}

static void handleSetTarget(const char* buf, AsyncWebSocketClient* client) {
  float val = parseValue(buf);
  g_cmd.manual_target = val;

  char ack[64];
  snprintf(ack, sizeof(ack), "{\"type\":\"ack\",\"cmd\":\"set_target\",\"value\":%.3f}", val);
  client->text(ack);
}

static void handleCommand(uint8_t* data, size_t len, AsyncWebSocketClient* client) {
  char buf[512];
  size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
  memcpy(buf, data, n);
  buf[n] = '\0';

  if      (strstr(buf, "\"set_bc\""))     handleSetBc(buf, client);
  else if (strstr(buf, "\"set_foc\""))    handleSetFoc(buf, client);
  else if (strstr(buf, "\"set_ctrl\""))   handleSetCtrl(buf, client);
  else if (strstr(buf, "\"set_target\"")) handleSetTarget(buf, client);
  else if (strstr(buf, "\"get_params\"")) sendParams(client);
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
void wifi_debug_init() {
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
  server.begin();
  Serial.println("HTTP + WebSocket server started");
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
