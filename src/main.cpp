#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>

#include "pins.h"
#include "app_context.h"
#include "wifi_debug.h"

// Forward declaration
void handleSerialCmd(const char* cmd);

// Global context pointer for serial commands
static AppContext* g_ctx = nullptr;

using namespace wheelsbot::hardware;
using namespace wheelsbot::control;

// ============================================================
// Tasks — all receive AppContext* via pvParameter
// ============================================================

static void ledTask(void* arg) {
  pinMode(LED_PIN, OUTPUT);
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// imuTask: 200Hz read via HAL (Core 0 — avoids Wire contention with focTask)
static void imuTask(void* arg) {
  auto* ctx = static_cast<AppContext*>(arg);
  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t last = xTaskGetTickCount();

  ImuSensor* imu = ctx->hw.imu();
  if (!imu) {
    Serial.println("IMU Task: no IMU available");
    vTaskDelete(nullptr);
    return;
  }

  while (true) {
    ImuData data;
    xSemaphoreTake(ctx->wire_mutex, portMAX_DELAY);
    bool ok = imu->read(data);
    xSemaphoreGive(ctx->wire_mutex);

    if (ok && data.valid) {
      ctx->imu_state.ax = data.ax;
      ctx->imu_state.ay = data.ay;
      ctx->imu_state.az = data.az;
      ctx->imu_state.gx = data.gx;
      ctx->imu_state.gy = data.gy;
      ctx->imu_state.gz = data.gz;
      ctx->imu_state.valid = true;

      ctx->imu_state.pitch_deg = data.pitch;
      ctx->imu_state.roll_deg  = data.roll;
      ctx->imu_state.yaw_deg   = data.yaw;
    } else {
      ctx->imu_state.valid = false;
    }

    vTaskDelayUntil(&last, period);
  }
}

// balanceTask: 200Hz LQR balance control
static void balanceTask(void* arg) {
  auto* ctx = static_cast<AppContext*>(arg);
  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t last = xTaskGetTickCount();

  ctx->balance.reset();
  uint32_t lastUs = micros();

  while (true) {
    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    if (!(dt > 0.f && dt < 0.1f)) dt = 0.005f;
    lastUs = nowUs;

    // Build BalanceInput from shared state
    // Note: right_enc already inverted in adapter layer (app_context.h:61)
    // Do NOT negate here to avoid double-inversion
    float wL = ctx->wheel_state.valid ? ctx->wheel_state.wL : 0.0f;
    float wR = ctx->wheel_state.valid ? ctx->wheel_state.wR : 0.0f;
    float xL = ctx->wheel_state.valid ? ctx->wheel_state.xL : 0.0f;
    float xR = ctx->wheel_state.valid ? ctx->wheel_state.xR : 0.0f;

    BalanceInput bin;
    // Raw IMU data
    bin.ax = ctx->imu_state.valid ? ctx->imu_state.ax : 0.0f;
    bin.ay = ctx->imu_state.valid ? ctx->imu_state.ay : 0.0f;
    bin.az = ctx->imu_state.valid ? ctx->imu_state.az : 0.0f;
    bin.gx = ctx->imu_state.valid ? ctx->imu_state.gx : 0.0f;
    bin.gy = ctx->imu_state.valid ? ctx->imu_state.gy : 0.0f;
    bin.gz = ctx->imu_state.valid ? ctx->imu_state.gz : 0.0f;

    // Fused angles
    bin.pitch          = ctx->imu_state.valid
        ? (ctx->imu_state.pitch_deg * (3.14159265358979f / 180.0f)) : 0.0f;
    bin.pitch_rate     = ctx->imu_state.valid ? ctx->imu_state.gy : 0.0f;
    bin.roll           = ctx->imu_state.valid
        ? (ctx->imu_state.roll_deg * (3.14159265358979f / 180.0f)) : 0.0f;
    bin.yaw_rate       = ctx->imu_state.valid ? ctx->imu_state.gz : 0.0f;

    bin.wheel_position = (xL + xR) * 0.5f;
    bin.wheel_velocity = (wL + wR) * 0.5f;
    bin.wL             = wL;
    bin.wR             = wR;
    bin.xL             = xL;
    bin.xR             = xR;

    bin.target_velocity = ctx->target_linear_vel;
    bin.target_yaw_rate = ctx->target_yaw_rate;
    bin.dt              = dt;
    bin.enabled         = ctx->cmd_state.balance_enable;
    bin.sensors_valid   = ctx->imu_state.valid && ctx->wheel_state.valid;

    BalanceOutput bout;
    ctx->balance.step(bin, bout);

    if (bout.valid) {
      ctx->bal_state.left_target  = bout.left_motor;
      ctx->bal_state.right_target = bout.right_motor;
      ctx->bal_state.ok = true;
    } else {
      ctx->bal_state.left_target  = 0;
      ctx->bal_state.right_target = 0;
      ctx->bal_state.ok = false;
    }

    vTaskDelayUntil(&last, period);
  }
}

// focTask: 1kHz FOC loop (Core 1 — sole owner of sensor I2C reads)
static void focTask(void* arg) {
  auto* ctx = static_cast<AppContext*>(arg);

  // ==== left motor (lsensor on Wire — shared with IMU, needs mutex) ====
  xSemaphoreTake(ctx->wire_mutex, portMAX_DELAY);
  ctx->lsensor.init();
  ctx->lmotor.linkSensor(&ctx->lsensor);

  ctx->ldriver.voltage_power_supply = SUPPLY_VOLTAGE;
  ctx->ldriver.init();
  ctx->lmotor.linkDriver(&ctx->ldriver);

  ctx->lmotor.controller = MotionControlType::torque;
  ctx->lmotor.torque_controller = TorqueControlType::voltage;
  ctx->lmotor.voltage_limit  = VOLTAGE_LIMIT_INIT;
  ctx->lmotor.velocity_limit = 100.0f;

  ctx->lmotor.init();
  bool lfoc_ok = ctx->lmotor.initFOC();
  xSemaphoreGive(ctx->wire_mutex);

  if (!lfoc_ok) {
    Serial.println("Left FOC init failed!");
    vTaskDelete(nullptr);
    return;
  }

  // ==== right motor (rsensor on Wire2) ====
  // Wire2 is passed via the static local in setup(), accessed here via extern
  extern TwoWire Wire2;
  ctx->rsensor.init(&Wire2);
  ctx->rmotor.linkSensor(&ctx->rsensor);

  ctx->rdriver.voltage_power_supply = SUPPLY_VOLTAGE;
  ctx->rdriver.init();
  ctx->rmotor.linkDriver(&ctx->rdriver);

  ctx->rmotor.controller = MotionControlType::torque;
  ctx->rmotor.torque_controller = TorqueControlType::voltage;
  ctx->rmotor.voltage_limit  = VOLTAGE_LIMIT_INIT;
  ctx->rmotor.velocity_limit = 100.0f;

  ctx->rmotor.init();
  if (!ctx->rmotor.initFOC()) {
    Serial.println("Right FOC init failed!");
    vTaskDelete(nullptr);
    return;
  }

  // Mark encoder adapters as initialized (SimpleFOC sensors are now ready)
  ctx->left_enc.init();
  ctx->right_enc.init();

  Serial.println("FOC ready");

  const TickType_t period = pdMS_TO_TICKS(1);
  TickType_t last = xTaskGetTickCount();
  uint32_t lastPrintMs = 0;

  bool last_motor_enable = false;

  while (true) {
    // enable/disable on edge only
    bool me = ctx->cmd_state.motor_enable;
    if (me != last_motor_enable) {
      last_motor_enable = me;
      if (me) {
        ctx->ldriver.enable(); ctx->lmotor.enable();
        ctx->rdriver.enable(); ctx->rmotor.enable();
      } else {
        ctx->lmotor.disable(); ctx->ldriver.disable();
        ctx->rmotor.disable(); ctx->rdriver.disable();
      }
    }

    // apply mode changes
    if (ctx->cmd_state.req_apply_mode) {
      if (ctx->cmd_state.req_motion_mode == 0) {
        ctx->lmotor.controller = MotionControlType::torque;
        ctx->rmotor.controller = MotionControlType::torque;
      } else if (ctx->cmd_state.req_motion_mode == 1) {
        ctx->lmotor.controller = MotionControlType::velocity;
        ctx->rmotor.controller = MotionControlType::velocity;
      } else {
        ctx->lmotor.controller = MotionControlType::angle;
        ctx->rmotor.controller = MotionControlType::angle;
      }

      if (ctx->cmd_state.req_torque_mode == 0) {
        ctx->lmotor.torque_controller = TorqueControlType::voltage;
        ctx->rmotor.torque_controller = TorqueControlType::voltage;
      } else if (ctx->cmd_state.req_torque_mode == 2) {
        ctx->lmotor.torque_controller = TorqueControlType::foc_current;
        ctx->rmotor.torque_controller = TorqueControlType::foc_current;
      }

      ctx->cmd_state.req_apply_mode = false;
    }

    // apply limits
    if (ctx->cmd_state.req_apply_limits) {
      ctx->lmotor.voltage_limit = ctx->cmd_state.req_tlimit;
      ctx->rmotor.voltage_limit = ctx->cmd_state.req_tlimit;

      if (ctx->cmd_state.req_vlimit > 0) {
        ctx->lmotor.velocity_limit = ctx->cmd_state.req_vlimit;
        ctx->rmotor.velocity_limit = ctx->cmd_state.req_vlimit;
      }

      ctx->cmd_state.req_apply_limits = false;
    }

    // apply PID
    if (ctx->cmd_state.req_apply_pid) {
      ctx->lmotor.PID_velocity.P = ctx->cmd_state.req_pid_p;
      ctx->lmotor.PID_velocity.I = ctx->cmd_state.req_pid_i;
      ctx->lmotor.PID_velocity.D = ctx->cmd_state.req_pid_d;

      ctx->rmotor.PID_velocity.P = ctx->cmd_state.req_pid_p;
      ctx->rmotor.PID_velocity.I = ctx->cmd_state.req_pid_i;
      ctx->rmotor.PID_velocity.D = ctx->cmd_state.req_pid_d;

      ctx->cmd_state.req_apply_pid = false;
    }

    // FOC loop (left motor uses Wire — protect with mutex)
    xSemaphoreTake(ctx->wire_mutex, portMAX_DELAY);
    ctx->lmotor.loopFOC();
    xSemaphoreGive(ctx->wire_mutex);
    ctx->rmotor.loopFOC();  // Wire2, no mutex needed

    // Update wheel state via SimpleFOC sensor adapters.
    // lsensor shares Wire with MPU6050 (imuTask) — I2C mutex contention can
    // cause occasional velocity spikes.  EMA low-pass filter attenuates
    // single-sample glitches while tracking real velocity (~30Hz cutoff at 1kHz).
    {
      constexpr float VEL_ALPHA = 0.2f;
      float wL_raw = ctx->left_enc.getVelocity();
      float wR_raw = ctx->right_enc.getVelocity();

      ctx->wheel_state.wL += VEL_ALPHA * (wL_raw - ctx->wheel_state.wL);
      ctx->wheel_state.wR += VEL_ALPHA * (wR_raw - ctx->wheel_state.wR);

      ctx->wheel_state.xL = ctx->left_enc.getAngle();
      ctx->wheel_state.xR = ctx->right_enc.getAngle();
      ctx->wheel_state.valid = true;
    }

    // choose target
    float ltarget = 0.0f;
    float rtarget = 0.0f;

    if (ctx->cmd_state.balance_enable && ctx->bal_state.ok) {
      ctx->lmotor.controller = MotionControlType::torque;
      ctx->rmotor.controller = MotionControlType::torque;
      ctx->lmotor.torque_controller = TorqueControlType::voltage;
      ctx->rmotor.torque_controller = TorqueControlType::voltage;

      ltarget = ctx->bal_state.left_target;
      rtarget = ctx->bal_state.right_target;
    } else {
      ltarget = ctx->cmd_state.manual_target;
      rtarget = ctx->cmd_state.manual_target;
    }

    ctx->lmotor.move(ltarget);
    ctx->rmotor.move(rtarget);

    // 1Hz debug print
    uint32_t ms = millis();
    if (ms - lastPrintMs >= 1000) {
      lastPrintMs = ms;
      Serial.printf("MOT,ctrl=%d,ltgt=%.3f,lvel=%.3f,rtgt=%.3f,rvel=%.3f\n",
        (int)ctx->lmotor.controller, ltarget, ctx->lsensor.getVelocity(),
        rtarget, ctx->rsensor.getVelocity());
    }

    vTaskDelayUntil(&last, period);
  }
}

// ============================================================
// I2C bus scan helper
// ============================================================
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

// ============================================================
// setup / loop
// ============================================================

// Wire2 declared at file scope so focTask can extern it
TwoWire Wire2(1);

// ============================================================
// Parameter persistence helpers (must be before setup)
// ============================================================
static bool loadBalanceParams(BalanceController::Params& p) {
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
    "\"max_tilt_deg\":%f,\"pitch_offset\":%f,"
    "\"pid_limit\":%f,"
    "\"lift_accel_thresh\":%f,\"lift_vel_thresh\":%f}",
    &p.angle_kp, &p.gyro_kp,
    &p.distance_kp, &p.speed_kp,
    &p.yaw_angle_kp, &p.yaw_gyro_kp,
    &p.lqr_u_kp, &p.lqr_u_ki,
    &p.zeropoint_kp,
    &p.lpf_target_vel_tf, &p.lpf_zeropoint_tf,
    &p.max_tilt_deg, &p.pitch_offset,
    &p.pid_limit,
    &p.lift_accel_thresh, &p.lift_vel_thresh);
  return matched >= 14;
}

static bool saveBalanceParams(BalanceController::Params& p) {
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
           "\"max_tilt_deg\":%.2f,\"pitch_offset\":%.4f,"
           "\"pid_limit\":%.2f,"
           "\"lift_accel_thresh\":%.2f,\"lift_vel_thresh\":%.2f}",
           p.angle_kp, p.gyro_kp,
           p.distance_kp, p.speed_kp,
           p.yaw_angle_kp, p.yaw_gyro_kp,
           p.lqr_u_kp, p.lqr_u_ki,
           p.zeropoint_kp,
           p.lpf_target_vel_tf, p.lpf_zeropoint_tf,
           p.max_tilt_deg, p.pitch_offset,
           p.pid_limit,
           p.lift_accel_thresh, p.lift_vel_thresh);
  f.close();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== Boot ===");
  Serial.flush();

  SimpleFOCDebug::enable(&Serial);

  // I2C buses
  Serial.println("I2C init...");
  Wire.begin((int)PIN_I2C_SDA, (int)PIN_I2C_SCL);
  Wire.setClock(400000);

  Wire2.begin((int)PIN_I2C_SDA2, (int)PIN_I2C_SCL2);
  Wire2.setClock(400000);
  Serial.println("I2C ready");

  scanBus(Wire, "Wire(3,9)");
  scanBus(Wire2, "Wire2(2,1)");

  // Construct AppContext — all hardware, control, and shared state
  static AppContext ctx(Wire, Wire2);
  g_ctx = &ctx;

  // Initialize IMU via HAL
  ctx.hw.initImu();

  // WiFi + WebSocket debug (initializes LittleFS first)
  wifi_debug_init(ctx);

  // Initialize ArduinoOTA for wireless firmware updates
  ArduinoOTA.setHostname("balancebot");
  ArduinoOTA.setPassword("balancebot");
  ArduinoOTA.onStart([]() {
    String type = ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem";
    Serial.println("OTA Update Start: " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Complete");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready - IP: " + WiFi.softAPIP().toString());

  // Load balance controller parameters from flash (if exists)
  BalanceController::Params params;
  if (loadBalanceParams(params)) {
    g_ctx->balance.setParams(params);
    Serial.println("Loaded balance params from flash");
  } else {
    Serial.println("Using default balance params (flash unavailable)");
  }

  // Start tasks — core assignment:
  // Core 0: imuTask(prio 5), balanceTask(prio 4), wifiDebugTask(prio 1), ledTask(prio 0)
  // Core 1: focTask(prio 5) — sole owner of sensor I2C reads
  xTaskCreatePinnedToCore(ledTask,       "LED", 2048, nullptr, 0, nullptr, 0);
  xTaskCreatePinnedToCore(imuTask,       "IMU", 8192, &ctx,    5, nullptr, 0);
  xTaskCreatePinnedToCore(balanceTask,   "BAL", 8192, &ctx,    4, nullptr, 0);
  xTaskCreatePinnedToCore(wifiDebugTask, "WST", 8192, &ctx,    1, nullptr, 0);
  xTaskCreatePinnedToCore(focTask,       "FOC", 8192, &ctx,    5, nullptr, 1);

  Serial.println("Tasks started");
}

// Forward declaration
void handleSerialCmd(const char* cmd);

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

  // Handle serial commands
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

// Serial command handler
void handleSerialCmd(const char* cmd) {
  // Format: <key>=<value> or <key>?
  // Examples:
  //   angle_kp=6.5      -> set angle_kp to 6.5
  //   angle_kp?         -> query angle_kp
  //   dump              -> dump all params
  //   telem             -> show current telemetry
  //   help              -> show help

  if (strlen(cmd) == 0) return;

  if (strcmp(cmd, "help") == 0) {
    Serial.println("\n=== Serial Commands ===");
    Serial.println("angle_kp=6.5    - set parameter");
    Serial.println("angle_kp?       - query parameter");
    Serial.println("dump            - show all parameters");
    Serial.println("telem           - show current telemetry (detailed)");
    Serial.println("watch           - continuous telemetry (1Hz, compact)");
    Serial.println("save            - save params to flash");
    Serial.println("reset           - reset controller");
    Serial.println("");
    Serial.println("Params: angle_kp, gyro_kp, distance_kp, speed_kp,");
    Serial.println("         yaw_angle_kp, yaw_gyro_kp, lqr_u_kp, lqr_u_ki,");
    Serial.println("         max_tilt_deg, pitch_offset, pid_limit");
    return;
  }

  if (strcmp(cmd, "dump") == 0) {
    BalanceController::Params p;
    g_ctx->balance.getParams(p);
    Serial.println("\n=== Current Params ===");
    Serial.printf("angle_kp=%.2f\n", p.angle_kp);
    Serial.printf("gyro_kp=%.4f\n", p.gyro_kp);
    Serial.printf("distance_kp=%.2f\n", p.distance_kp);
    Serial.printf("speed_kp=%.4f\n", p.speed_kp);
    Serial.printf("yaw_angle_kp=%.2f\n", p.yaw_angle_kp);
    Serial.printf("yaw_gyro_kp=%.4f\n", p.yaw_gyro_kp);
    Serial.printf("lqr_u_kp=%.2f\n", p.lqr_u_kp);
    Serial.printf("lqr_u_ki=%.2f\n", p.lqr_u_ki);
    Serial.printf("max_tilt_deg=%.1f\n", p.max_tilt_deg);
    Serial.printf("pitch_offset=%.2f\n", p.pitch_offset);
    Serial.printf("pid_limit=%.2f\n", p.pid_limit);
    return;
  }

  if (strcmp(cmd, "telem") == 0) {
    BalanceDebug dbg;
    g_ctx->balance.getDebug(dbg);
    Serial.println("\n=== Telemetry ===");

    // IMU angles
    Serial.printf("PITCH:%.2fdeg  ROLL:%.2fdeg  YAW_RATE:%.2fdeg/s\n",
      dbg.pitch*180/PI, dbg.roll*180/PI, dbg.yaw_rate*180/PI);
    Serial.printf("PITCH_RATE:%.2frad/s  PITCH_OFFSET:%.2fdeg\n",
      dbg.pitch_rate, dbg.pitch_offset);

    // Wheel data
    Serial.printf("wL:%.2f  wR:%.2f rad/s  |  xL:%.1f  xR:%.1f rad\n",
      dbg.wL, dbg.wR, dbg.xL, dbg.xR);
    Serial.printf("DIST_ZERO:%.2f rad\n", dbg.distance_zeropoint);

    // Control contributions
    Serial.printf("ANGLE:%.2f  GYRO:%.2f  DIST:%.2f  SPD:%.2f\n",
      dbg.angle_contribution, dbg.gyro_contribution,
      dbg.distance_contribution, dbg.speed_contribution);
    Serial.printf("LQR_RAW:%.2f  LQR_COMP:%.2f\n",
      dbg.lqr_u_raw, dbg.lqr_u_compensated);

    // Motor outputs
    Serial.printf("MOTOR_L:%.2fV  MOTOR_R:%.2fV\n",
      dbg.left_motor, dbg.right_motor);

    // State
    Serial.printf("STATE:%s  FAULT:0x%02X  LIFTED:%d\n",
      dbg.running?"RUN":"STOP", (unsigned)dbg.fault_flags, dbg.wheel_lifted);

    return;
  }

  if (strcmp(cmd, "save") == 0) {
    Serial.println("Use web UI to save params, or send via WebSocket");
    Serial.println("WS: {\"type\":\"save_params\"}");
    return;
  }

  if (strcmp(cmd, "reset") == 0) {
    g_ctx->balance.reset();
    Serial.println("Controller reset");
    return;
  }

  // Parse key=value or key?
  char* eq = strchr((char*)cmd, '=');
  char* q = strchr((char*)cmd, '?');

  char key[32] = {0};
  float value = 0;
  bool isQuery = false;

  // Handle key? query
  if (q && (!eq || q < eq)) {
    // key?
    int len = q - cmd;
    if (len > 0 && len < 31) {
      memcpy(key, cmd, len);
      key[len] = '\0';
    }
    isQuery = true;
  }
  // Handle key=value
  else if (eq) {
    int len = eq - cmd;
    if (len > 0 && len < 31) {
      memcpy(key, cmd, len);
      key[len] = '\0';
    }
    value = atof(eq + 1);
  }
  else {
    Serial.println("Unknown command. Type 'help' for available commands.");
    return;
  }

  // Set or get parameter
  BalanceController::Params p;
  g_ctx->balance.getParams(p);

  bool found = false;

  if (strcmp(key, "angle_kp") == 0) { p.angle_kp = isQuery ? p.angle_kp : value; found = true; }
  else if (strcmp(key, "gyro_kp") == 0) { p.gyro_kp = isQuery ? p.gyro_kp : value; found = true; }
  else if (strcmp(key, "distance_kp") == 0) { p.distance_kp = isQuery ? p.distance_kp : value; found = true; }
  else if (strcmp(key, "speed_kp") == 0) { p.speed_kp = isQuery ? p.speed_kp : value; found = true; }
  else if (strcmp(key, "yaw_angle_kp") == 0) { p.yaw_angle_kp = isQuery ? p.yaw_angle_kp : value; found = true; }
  else if (strcmp(key, "yaw_gyro_kp") == 0) { p.yaw_gyro_kp = isQuery ? p.yaw_gyro_kp : value; found = true; }
  else if (strcmp(key, "lqr_u_kp") == 0) { p.lqr_u_kp = isQuery ? p.lqr_u_kp : value; found = true; }
  else if (strcmp(key, "lqr_u_ki") == 0) { p.lqr_u_ki = isQuery ? p.lqr_u_ki : value; found = true; }
  else if (strcmp(key, "max_tilt_deg") == 0) { p.max_tilt_deg = isQuery ? p.max_tilt_deg : value; found = true; }
  else if (strcmp(key, "pitch_offset") == 0) { p.pitch_offset = isQuery ? p.pitch_offset : value; found = true; }
  else if (strcmp(key, "pid_limit") == 0) { p.pid_limit = isQuery ? p.pid_limit : value; found = true; }

  if (found) {
    if (!isQuery) {
      g_ctx->balance.setParams(p);
      Serial.printf("Set %s = %.4f\n", key, value);
    } else {
      // Query: print the actual value that was found
      float val = 0;
      if (strcmp(key, "angle_kp") == 0) val = p.angle_kp;
      else if (strcmp(key, "gyro_kp") == 0) val = p.gyro_kp;
      else if (strcmp(key, "distance_kp") == 0) val = p.distance_kp;
      else if (strcmp(key, "speed_kp") == 0) val = p.speed_kp;
      else if (strcmp(key, "yaw_angle_kp") == 0) val = p.yaw_angle_kp;
      else if (strcmp(key, "yaw_gyro_kp") == 0) val = p.yaw_gyro_kp;
      else if (strcmp(key, "lqr_u_kp") == 0) val = p.lqr_u_kp;
      else if (strcmp(key, "lqr_u_ki") == 0) val = p.lqr_u_ki;
      else if (strcmp(key, "max_tilt_deg") == 0) val = p.max_tilt_deg;
      else if (strcmp(key, "pitch_offset") == 0) val = p.pitch_offset;
      else if (strcmp(key, "pid_limit") == 0) val = p.pid_limit;
      Serial.printf("%s=%.4f\n", key, val);
    }
  } else {
    Serial.printf("Unknown param: %s (key len=%d)\n", key, strlen(key));
  }
}
