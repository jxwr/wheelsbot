#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <ArduinoOTA.h>
#include <cstring>

#include "pins.h"
#include "app_context.h"
#include "wifi_debug.h"
#include "serial_debug.h"

using namespace wheelsbot::hardware;
using namespace wheelsbot::control;

// Global context pointer for commander callbacks
static AppContext* bal_ctx = nullptr;

// Forward declarations for serial commands
static void setParam(const char* name, float value);
static void cmdShow();
static void handleSerialCommands();

// ============================================================
// Helper functions
// ============================================================
static inline float clamp(float x, float lo, float hi) {
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

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

// serialDebugTask: 100Hz - Process serial commands and stream data
static void serialDebugTask(void* arg) {
  auto* ctx = static_cast<AppContext*>(arg);
  static wheelsbot::debug::SerialDebug serial_debug;

  serial_debug.init(ctx);

  const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz
  TickType_t last = xTaskGetTickCount();

  while (true) {
    serial_debug.processCommands();
    serial_debug.sendDataIfReady();
    vTaskDelayUntil(&last, period);
  }
}

// balanceTask: 200Hz cascade control — Velocity → Angle + Yaw mixing
static void balanceTask(void* arg) {
  auto* ctx = static_cast<AppContext*>(arg);
  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t last = xTaskGetTickCount();

  ctx->balance.reset();
  uint32_t lastUs = micros();

  // Parameters are loaded from flash or defaults in balance_controller.h
  // No hardcoded tuning here - keep it clean!

  while (true) {
    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    if (!(dt > 0.f && dt < 0.1f)) dt = 0.005f;
    lastUs = nowUs;

    // Get sensor data
    // Note: wL/wR negated to match physical direction (positive = forward)
    float wL = ctx->wheel_state.valid ? -ctx->wheel_state.wL : 0.0f;
    float wR = ctx->wheel_state.valid ? -ctx->wheel_state.wR : 0.0f;
    float wheel_vel = (wL + wR) * 0.5f;

    // Prepare balance input
    BalanceInput bin;
    bin.pitch = ctx->imu_state.valid
        ? (ctx->imu_state.pitch_deg * 3.14159f / 180.0f) : 0.0f;
    bin.pitch_rate = ctx->imu_state.valid ? ctx->imu_state.gy : 0.0f;
    bin.yaw_rate = ctx->imu_state.valid ? ctx->imu_state.gz : 0.0f;
    bin.wheel_velocity = wheel_vel;
    bin.wheel_position = (ctx->wheel_state.xL + ctx->wheel_state.xR) * 0.5f;

    // Determine targets based on mode
    if (ctx->remote_mode) {
      bin.target_velocity = ctx->target_linear_vel;
      bin.target_yaw_rate = ctx->target_yaw_rate;
    } else {
      bin.target_velocity = 0.0f;
      bin.target_yaw_rate = 0.0f;
    }

    bin.dt = dt;
    bin.enabled = ctx->cmd_state.balance_enable;
    bin.sensors_valid = ctx->imu_state.valid && ctx->wheel_state.valid;

    // Execute cascade control
    BalanceOutput bout;
    ctx->balance.step(bin, bout);

    // Update shared state for telemetry
    ctx->bal_state.left_target = bout.left_motor;
    ctx->bal_state.right_target = bout.right_motor;
    ctx->bal_state.ok = bout.valid;

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

    // Control mode switching (edge-triggered to avoid redundant setting)
    static bool last_balance_mode = false;
    bool balance_mode = ctx->cmd_state.balance_enable && ctx->bal_state.ok;

    if (balance_mode != last_balance_mode) {
      if (balance_mode) {
        ctx->lmotor.controller = MotionControlType::torque;
        ctx->rmotor.controller = MotionControlType::torque;
        ctx->lmotor.torque_controller = TorqueControlType::voltage;
        ctx->rmotor.torque_controller = TorqueControlType::voltage;
      }
      last_balance_mode = balance_mode;
    }

    // Choose target based on mode
    float ltarget, rtarget;
    if (balance_mode) {
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
    ctx.balance.setParams(params);
    Serial.println("Loaded balance params from flash");
  } else {
    Serial.println("Using default balance params (flash unavailable)");
  }

  // Start tasks — core assignment:
  // Core 0: imuTask(prio 5), balanceTask(prio 4), serialDebugTask(prio 2), wifiDebugTask(prio 1), ledTask(prio 0)
  // Core 1: focTask(prio 5) — sole owner of sensor I2C reads
  xTaskCreatePinnedToCore(ledTask,         "LED", 2048, nullptr, 0, nullptr, 0);
  xTaskCreatePinnedToCore(imuTask,         "IMU", 8192, &ctx,    5, nullptr, 0);
  xTaskCreatePinnedToCore(balanceTask,     "BAL", 8192, &ctx,    4, nullptr, 0);
  xTaskCreatePinnedToCore(serialDebugTask, "SDB", 4096, &ctx,    2, nullptr, 0);
  xTaskCreatePinnedToCore(wifiDebugTask,   "WST", 8192, &ctx,    1, nullptr, 0);
  xTaskCreatePinnedToCore(focTask,         "FOC", 8192, &ctx,    5, nullptr, 1);

  Serial.println("Tasks started");
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

  // Serial commands now handled by serialDebugTask (CSV-based AI-friendly protocol)
  // Old simple command handler removed - use new SerialDebug interface

  vTaskDelay(pdMS_TO_TICKS(100));
}

// ============================================================
// Serial command implementation
// ============================================================

static void setParam(const char* name, float value) {
  BalanceController::Params p;
  bal_ctx->balance.getParams(p);

  if (strcmp(name, "velocity_kp") == 0) p.velocity_kp = value;
  else if (strcmp(name, "velocity_ki") == 0) p.velocity_ki = value;
  else if (strcmp(name, "velocity_kd") == 0) p.velocity_kd = value;
  else if (strcmp(name, "velocity_max_tilt") == 0) p.velocity_max_tilt = value;
  else if (strcmp(name, "angle_kp") == 0) p.angle_kp = value;
  else if (strcmp(name, "angle_ki") == 0) p.angle_ki = value;
  else if (strcmp(name, "angle_gyro_kd") == 0) p.angle_gyro_kd = value;
  else if (strcmp(name, "angle_d_alpha") == 0) p.angle_d_alpha = value;
  else if (strcmp(name, "angle_max_out") == 0) p.angle_max_out = value;
  else if (strcmp(name, "yaw_kd") == 0) p.yaw_kd = value;
  else if (strcmp(name, "max_tilt") == 0) p.max_tilt = value;
  else if (strcmp(name, "ramp_time") == 0) p.ramp_time = value;
  else if (strcmp(name, "pitch_offset") == 0) p.pitch_offset = value;
  else if (strcmp(name, "pitch_cmd_rate_limit") == 0) p.pitch_cmd_rate_limit = value;
  else if (strcmp(name, "sensor_timeout") == 0) p.sensor_timeout = value;
  else {
    Serial.printf("Unknown param: %s\n", name);
    return;
  }

  bal_ctx->balance.setParams(p);
  Serial.printf("Set %s = %.4f (NOT saved yet, use 'S' to save)\n", name, value);
}

static void cmdShow() {
  BalanceController::Params p;
  bal_ctx->balance.getParams(p);
  Serial.printf("=== Balance Controller Params ===\n");
  Serial.printf("Velocity: Kp=%.4f, Ki=%.4f, Kd=%.4f, max_tilt=%.3f\n",
                p.velocity_kp, p.velocity_ki, p.velocity_kd, p.velocity_max_tilt);
  Serial.printf("Angle:    Kp=%.4f, Ki=%.4f, gyro_Kd=%.4f, d_alpha=%.3f, max_out=%.2f\n",
                p.angle_kp, p.angle_ki, p.angle_gyro_kd, p.angle_d_alpha, p.angle_max_out);
  Serial.printf("Yaw:      Kd=%.4f\n", p.yaw_kd);
  Serial.printf("Safety:   max_tilt=%.3f, ramp_time=%.3f, pitch_offset=%.4f\n",
                p.max_tilt, p.ramp_time, p.pitch_offset);
  Serial.printf("Advanced: pitch_rate_limit=%.3f, sensor_timeout=%.3f\n",
                p.pitch_cmd_rate_limit, p.sensor_timeout);
  Serial.printf("Decimation: %d\n", p.velocity_decimation);
}

static void handleSerialCommands() {
  static char cmd[64];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        cmd[idx] = '\0';

        // Parse command
        char name[32];
        float value;
        if (sscanf(cmd, "P %s %f", name, &value) == 2) {
          // P <param> <value> - Set parameter
          setParam(name, value);
        }
        else if (cmd[0] == 'S' || cmd[0] == 's') {
          // S - Save to flash
          BalanceController::Params p;
          bal_ctx->balance.getParams(p);
          if (saveBalanceParams(p)) {
            Serial.println("✓ Params saved to flash");
          } else {
            Serial.println("✗ Save failed");
          }
        }
        else if (cmd[0] == 'L' || cmd[0] == 'l') {
          // L - Load from flash
          BalanceController::Params p;
          if (loadBalanceParams(p)) {
            bal_ctx->balance.setParams(p);
            Serial.println("✓ Params loaded from flash");
          } else {
            Serial.println("✗ No saved params found");
          }
        }
        else if (cmd[0] == 'D' || cmd[0] == 'd') {
          // D - Display current params
          cmdShow();
        }
        else if (cmd[0] == 'R' || cmd[0] == 'r') {
          // R - Reset to defaults
          bal_ctx->balance.reset();
          Serial.println("✓ Params reset to defaults (NOT saved, use 'S' to save)");
        }
        else {
          Serial.println("Commands:");
          Serial.println("  P <param> <value>  - Set parameter (e.g., P angle_kp 3.0)");
          Serial.println("  S                  - Save to flash");
          Serial.println("  L                  - Load from flash");
          Serial.println("  D                  - Display current params");
          Serial.println("  R                  - Reset to defaults");
          Serial.println("Params: velocity_kp/ki/kd, angle_kp/ki/gyro_kd/d_alpha,");
          Serial.println("        yaw_kd, max_tilt, ramp_time, pitch_offset,");
          Serial.println("        pitch_cmd_rate_limit, sensor_timeout");
        }

        idx = 0;
      }
    } else if (idx < sizeof(cmd) - 1) {
      cmd[idx++] = c;
    }
  }
}
