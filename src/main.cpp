#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

#include "pins.h"
#include "app_context.h"
#include "wifi_debug.h"

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

// balanceTask: 200Hz cascade control — Velocity → Angle
static void balanceTask(void* arg) {
  auto* ctx = static_cast<AppContext*>(arg);
  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t last = xTaskGetTickCount();

  ctx->cascade.reset();
  uint32_t lastUs = micros();

  // Initial tuning: inner loop only for safety
  ctx->cascade.angleLoop().setGains(10.0f, 0.0f, 0.4f);
  ctx->cascade.velocityLoop().setGains(0.01f, 0.0f, 0.0f);  // Minimal P-only for stability

  while (true) {
    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    if (!(dt > 0.f && dt < 0.1f)) dt = 0.005f;
    lastUs = nowUs;

    // Prepare cascade input
    CascadeInput cin;
    cin.velocity_reference = 0.0f;
    cin.velocity_measurement = ctx->wheel_state.valid
        ? (ctx->wheel_state.wL + ctx->wheel_state.wR) * 0.5f : 0.0f;
    cin.pitch_measurement = ctx->imu_state.valid
        ? (ctx->imu_state.pitch_deg * 3.14159f / 180.0f) : 0.0f;
    cin.pitch_rate = ctx->imu_state.valid ? ctx->imu_state.gy : 0.0f;
    cin.dt = dt;
    cin.timestamp_ms = millis();
    cin.enabled = ctx->cmd_state.balance_enable;
    cin.sensors_valid = ctx->imu_state.valid && ctx->wheel_state.valid;

    CascadeOutput cout;
    ctx->cascade.step(cin, cout);

    if (cout.valid && ctx->cascade.isRunning()) {
      ctx->bal_state.left_target  = cout.left_motor;
      ctx->bal_state.right_target = cout.right_motor;
      ctx->bal_state.ok = true;
    } else {
      ctx->bal_state.left_target  = 0.0f;
      ctx->bal_state.right_target = 0.0f;
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

  // Load cascade controller parameters from flash (if exists)
  CascadeController::Params params;
  if (loadCascadeParams(params)) {
    ctx.cascade.setParams(params);
    Serial.println("Loaded cascade params from flash");
  } else {
    Serial.println("Using default cascade params (flash unavailable)");
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

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
