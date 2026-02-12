#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

#include "pins.h"
#include "balance_core.h"
#include "shared_state.h"
#include "wifi_debug.h"

// Hardware Abstraction Layer
#include "hardware/hardware_manager.h"
#include "hardware/imu_interface.h"
#include "hardware/encoder_interface.h"

using namespace wheelsbot::hardware;

// ============================================================
// Second I2C bus
// ============================================================
TwoWire Wire2 = TwoWire(1);

// ============================================================
// SimpleFOC objects
// ============================================================
MagneticSensorI2C lsensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C rsensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor lmotor(POLE_PAIRS);
BLDCMotor rmotor(POLE_PAIRS);
BLDCDriver3PWM ldriver(PIN_PWM_U, PIN_PWM_V, PIN_PWM_W, PIN_EN);
BLDCDriver3PWM rdriver(PIN_PWM_U2, PIN_PWM_V2, PIN_PWM_W2, PIN_EN2);

// ============================================================
// Global shared state instances
// ============================================================
ImuShared      g_imu;
BalanceShared  g_bal;
WheelShared    g_wheel;
CommandShared  g_cmd;
bc_ctx_t       g_bc;

// ============================================================
// Tasks
// ============================================================

void ledTask(void* arg) {
  pinMode(LED_PIN, OUTPUT);
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// imuTask: 200Hz read via HAL
void imuTask(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t last = xTaskGetTickCount();

  ImuSensor* imu = HardwareManager::instance().imu();
  if (!imu) {
    Serial.println("IMU Task: no IMU available");
    vTaskDelete(nullptr);
    return;
  }

  while (true) {
    ImuData data;
    bool ok = imu->read(data);

    if (ok && data.valid) {
      g_imu.ax = data.ax;
      g_imu.ay = data.ay;
      g_imu.az = data.az;
      g_imu.gx = data.gx;
      g_imu.gy = data.gy;
      g_imu.gz = data.gz;
      g_imu.valid = true;

      g_imu.pitch_deg = data.pitch;
      g_imu.roll_deg  = data.roll;
      g_imu.yaw_deg   = data.yaw;
    } else {
      g_imu.valid = false;
    }

    vTaskDelayUntil(&last, period);
  }
}

// balanceTask: 200Hz bc_step — uses own micros() for dt (bug fix)
void balanceTask(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(5);
  TickType_t last = xTaskGetTickCount();

  bc_reset(&g_bc);
  uint32_t lastUs = micros();

  while (true) {
    // own dt calculation (bug fix: was using g_dt from imuTask)
    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    if (!(dt > 0.f && dt < 0.1f)) dt = 0.005f;
    lastUs = nowUs;

    bc_input_t in;
    memset(&in, 0, sizeof(in));

    in.time.dt = dt;
    in.cmd.enable = g_cmd.balance_enable;
    in.cmd.v_fwd = 0.0f;
    in.cmd.w_yaw = 0.0f;
    in.cmd.pitch_inject_enable = false;
    in.cmd.pitch_inject_deg = 0.0f;

    in.imu.valid = g_imu.valid;
    in.imu.ax = g_imu.ax; in.imu.ay = g_imu.ay; in.imu.az = g_imu.az;
    in.imu.gx = g_imu.gx; in.imu.gy = g_imu.gy; in.imu.gz = g_imu.gz;

    in.wheel.valid = g_wheel.valid;
    in.wheel.wL = g_wheel.wL;
    in.wheel.wR = g_wheel.wR;

    bc_output_t out;
    bc_step(&g_bc, &in, &out);

    if (out.ok && g_bc.d.state == BC_STATE_RUNNING && g_cmd.balance_enable) {
      g_bal.left_target  = out.left;
      g_bal.right_target = out.right;
      g_bal.ok = true;
    } else {
      g_bal.left_target  = 0.0f;
      g_bal.right_target = 0.0f;
      g_bal.ok = false;
    }

    vTaskDelayUntil(&last, period);
  }
}

// focTask: 1kHz FOC loop
void focTask(void* arg) {
  // ==== left motor ====
  lsensor.init();
  lmotor.linkSensor(&lsensor);

  ldriver.voltage_power_supply = SUPPLY_VOLTAGE;
  ldriver.init();
  lmotor.linkDriver(&ldriver);

  lmotor.controller = MotionControlType::torque;
  lmotor.torque_controller = TorqueControlType::voltage;
  lmotor.voltage_limit  = VOLTAGE_LIMIT_INIT;
  lmotor.velocity_limit = 100.0f;

  lmotor.init();
  if (!lmotor.initFOC()) {
    Serial.println("Left FOC init failed!");
    vTaskDelete(nullptr);
    return;
  }

  // ==== right motor ====
  rsensor.init(&Wire2);
  rmotor.linkSensor(&rsensor);

  rdriver.voltage_power_supply = SUPPLY_VOLTAGE;
  rdriver.init();
  rmotor.linkDriver(&rdriver);

  rmotor.controller = MotionControlType::torque;
  rmotor.torque_controller = TorqueControlType::voltage;
  rmotor.voltage_limit  = VOLTAGE_LIMIT_INIT;
  rmotor.velocity_limit = 100.0f;

  rmotor.init();
  if (!rmotor.initFOC()) {
    Serial.println("Right FOC init failed!");
    vTaskDelete(nullptr);
    return;
  }

  Serial.println("FOC ready");

  const TickType_t period = pdMS_TO_TICKS(1);
  TickType_t last = xTaskGetTickCount();
  uint32_t lastPrintMs = 0;

  bool last_motor_enable = false;

  while (true) {
    // enable/disable on edge only
    bool me = g_cmd.motor_enable;
    if (me != last_motor_enable) {
      last_motor_enable = me;
      if (me) {
        ldriver.enable(); lmotor.enable();
        rdriver.enable(); rmotor.enable();
      } else {
        lmotor.disable(); ldriver.disable();
        rmotor.disable(); rdriver.disable();
      }
    }

    // apply mode changes
    if (g_cmd.req_apply_mode) {
      if (g_cmd.req_motion_mode == 0) {
        lmotor.controller = MotionControlType::torque;
        rmotor.controller = MotionControlType::torque;
      } else if (g_cmd.req_motion_mode == 1) {
        lmotor.controller = MotionControlType::velocity;
        rmotor.controller = MotionControlType::velocity;
      } else {
        lmotor.controller = MotionControlType::angle;
        rmotor.controller = MotionControlType::angle;
      }

      if (g_cmd.req_torque_mode == 0) {
        lmotor.torque_controller = TorqueControlType::voltage;
        rmotor.torque_controller = TorqueControlType::voltage;
      } else if (g_cmd.req_torque_mode == 2) {
        lmotor.torque_controller = TorqueControlType::foc_current;
        rmotor.torque_controller = TorqueControlType::foc_current;
      }

      g_cmd.req_apply_mode = false;
    }

    // apply limits
    if (g_cmd.req_apply_limits) {
      lmotor.voltage_limit = g_cmd.req_tlimit;
      rmotor.voltage_limit = g_cmd.req_tlimit;

      if (g_cmd.req_vlimit > 0) {
        lmotor.velocity_limit = g_cmd.req_vlimit;
        rmotor.velocity_limit = g_cmd.req_vlimit;
      }

      g_cmd.req_apply_limits = false;
    }

    // apply PID
    if (g_cmd.req_apply_pid) {
      lmotor.PID_velocity.P = g_cmd.req_pid_p;
      lmotor.PID_velocity.I = g_cmd.req_pid_i;
      lmotor.PID_velocity.D = g_cmd.req_pid_d;

      rmotor.PID_velocity.P = g_cmd.req_pid_p;
      rmotor.PID_velocity.I = g_cmd.req_pid_i;
      rmotor.PID_velocity.D = g_cmd.req_pid_d;

      g_cmd.req_apply_pid = false;
    }

    // FOC loop
    lmotor.loopFOC();
    rmotor.loopFOC();

    g_wheel.wL =  lsensor.getVelocity();
    g_wheel.wR = -rsensor.getVelocity();  // 与电机取反一致
    g_wheel.xL =  lsensor.getAngle();
    g_wheel.xR = -rsensor.getAngle();
    g_wheel.valid = true;

    // choose target
    float ltarget = 0.0f;
    float rtarget = 0.0f;

    if (g_cmd.balance_enable && g_bal.ok) {
      lmotor.controller = MotionControlType::torque;
      rmotor.controller = MotionControlType::torque;
      lmotor.torque_controller = TorqueControlType::voltage;
      rmotor.torque_controller = TorqueControlType::voltage;

      ltarget = g_bal.left_target;
      rtarget = g_bal.right_target;
    } else {
      ltarget = g_cmd.manual_target;
      rtarget = g_cmd.manual_target;
    }

    lmotor.move(ltarget);
    rmotor.move(rtarget);

    // 1Hz debug print
    uint32_t ms = millis();
    if (ms - lastPrintMs >= 1000) {
      lastPrintMs = ms;
      Serial.printf("MOT,ctrl=%d,ltgt=%.3f,lvel=%.3f,rtgt=%.3f,rvel=%.3f\n",
        (int)lmotor.controller, ltarget, lsensor.getVelocity(),
        rtarget, rsensor.getVelocity());
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
void setup() {
  Serial.begin(115200);
  delay(500);
  SimpleFOCDebug::enable(&Serial);

  // I2C buses
  Wire.begin((int)PIN_I2C_SDA, (int)PIN_I2C_SCL);
  Wire.setClock(400000);

  Wire2.begin((int)PIN_I2C_SDA2, (int)PIN_I2C_SCL2);
  Wire2.setClock(400000);

  scanBus(Wire, "Wire(3,9)");
  scanBus(Wire2, "Wire2(2,1)");

  // Initialize hardware via HAL
  // Left encoder on Wire (shared with IMU), Right encoder on Wire2
  bool hw_ok = HardwareManager::instance().init(Wire, Wire, Wire2);
  if (!hw_ok) {
    Serial.println("Warning: Some hardware failed to initialize");
  }

  // init balance core
  bc_init(&g_bc);

  // WiFi + WebSocket debug
  wifi_debug_init();

  // Start tasks — core assignment per plan:
  // Core 0: imuTask(prio 5), balanceTask(prio 4), wifiDebugTask(prio 1), ledTask(prio 0)
  // Core 1: focTask(prio 5)
  xTaskCreatePinnedToCore(ledTask,       "LED", 2048, nullptr, 0, nullptr, 0);
  xTaskCreatePinnedToCore(imuTask,       "IMU", 8192, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(balanceTask,   "BAL", 8192, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(wifiDebugTask, "WST", 8192, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(focTask,       "FOC", 8192, nullptr, 5, nullptr, 1);

  Serial.println("Tasks started");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
