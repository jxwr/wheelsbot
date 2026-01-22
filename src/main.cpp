#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

// ====== Pin config for ESP32-S3-DevKitC-1 ======
static constexpr gpio_num_t PIN_PWM_U = GPIO_NUM_4;
static constexpr gpio_num_t PIN_PWM_V = GPIO_NUM_5;
static constexpr gpio_num_t PIN_PWM_W = GPIO_NUM_6;
static constexpr gpio_num_t PIN_EN    = GPIO_NUM_7;

static constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_8;
static constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_9;

static constexpr gpio_num_t PIN_I2C_IMU_SDA = GPIO_NUM_2;
static constexpr gpio_num_t PIN_I2C_IMU_SCL = GPIO_NUM_1;

static constexpr int   LED_PIN = 18;

// ====== Motor config ======
static constexpr int   POLE_PAIRS      = 7;
static constexpr float SUPPLY_VOLTAGE  = 12.0f;
static constexpr float VOLTAGE_LIMIT   = 3.0f;   // 先小一点，确认方向/闭环稳定再加


// ====== MPU config ======
static const uint8_t MPU_ADDR = 0x68;   // AD0=0

TwoWire WireIMU = TwoWire(1);

struct ImuRaw {
  int16_t ax, ay, az;
  int16_t temp;
  int16_t gx, gy, gz;
  uint32_t t_us;
};

bool mpuWrite(uint8_t reg, uint8_t val) {
  WireIMU.beginTransmission(MPU_ADDR);
  WireIMU.write(reg);
  WireIMU.write(val);
  return WireIMU.endTransmission() == 0;
}

bool mpuRead14(ImuRaw &out) {
  WireIMU.beginTransmission(MPU_ADDR);
  WireIMU.write(0x3B);                      // ACCEL_XOUT_H
  if (WireIMU.endTransmission(false) != 0) return false;

  int n = WireIMU.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (n != 14) return false;

  auto rd16 = [&]() -> int16_t {
    int16_t hi = WireIMU.read();
    int16_t lo = WireIMU.read();
    return (int16_t)((hi << 8) | lo);
  };

  out.ax = rd16(); out.ay = rd16(); out.az = rd16();
  out.temp = rd16();
  out.gx = rd16(); out.gy = rd16(); out.gz = rd16();
  out.t_us = micros();
  return true;
}

struct ImuScaled {
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;
  float pitch_deg;
};

ImuScaled convertAndCompute(const ImuRaw& s) {
  ImuScaled o;

  // accel: raw -> g
  o.ax_g = s.ax / 16384.0f;
  o.ay_g = s.ay / 16384.0f;
  o.az_g = s.az / 16384.0f;

  // gyro: raw -> deg/s
  o.gx_dps = s.gx / 131.0f;
  o.gy_dps = s.gy / 131.0f;
  o.gz_dps = s.gz / 131.0f;

  // pitch from accel only
  o.pitch_deg = atan2f(
    -o.ax_g,
    sqrtf(o.ay_g * o.ay_g + o.az_g * o.az_g)
  ) * 180.0f / PI;

  return o;
}

// ====== Helper functions ======

void deadloop() {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.println("reach here!");
  }
}

// ====== SimpleFOC objects ======
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor(POLE_PAIRS);
BLDCDriver3PWM driver(PIN_PWM_U, PIN_PWM_V, PIN_PWM_W, PIN_EN);
Commander command = Commander(Serial);

// 控制目标（可以被别的 task 改，比如 HTTP task）
volatile float target_velocity = 5.0f; // rad/s

// ---------------- LED task ----------------
void ledTask(void* arg) {
  pinMode(LED_PIN, OUTPUT);
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(3000));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Serial.println("LED: blink!");
  }
}


// ---------------- FOC task ----------------
void focSensorTestTask(void* arg) {
  // I2C + sensor init
  Wire.begin((int)PIN_I2C_SDA, (int)PIN_I2C_SCL);
  Wire.setClock(400000);

  sensor.init();
  motor.linkSensor(&sensor);

  // driver init
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.init();
  motor.linkDriver(&driver);

  // controller setup
  motor.controller = MotionControlType::velocity;
  motor.voltage_limit = VOLTAGE_LIMIT;

  // motor init + FOC align
  motor.init();
  motor.initFOC();

  // 如果你需要 SimpleFOC 的监控（会影响实时性，先别开）
  // motor.useMonitoring(Serial);

  Serial.println("Driver + FOC ready");

  // 固定周期：1ms = 1kHz（起步够用；你也可以改成 2ms/0.5ms）
  const TickType_t period = pdMS_TO_TICKS(1);
  TickType_t last = xTaskGetTickCount();

  // 用于限频打印（例如 20Hz）
  uint32_t lastPrintMs = 0;

  while (true) {
    // FOC 核心循环
    motor.loopFOC();
    motor.move((float)target_velocity);

    // 限频打印：别在高频回路里疯狂 Serial.print
    uint32_t now = millis();
    if (now - lastPrintMs >= 50) { // 20Hz
      lastPrintMs = now;
      Serial.print("angle=");
      Serial.print(sensor.getAngle());
      Serial.print("\tvel=");
      Serial.println(sensor.getVelocity());
    }

    // 稳定周期，不漂移
    vTaskDelayUntil(&last, period);
  }
}

void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void onMode(char* cmd) {
  int m = atoi(cmd);
  if (m == 0) motor.controller = MotionControlType::torque;
  if (m == 1) motor.controller = MotionControlType::velocity;
  if (m == 2) motor.controller = MotionControlType::angle;
}

void focDriverOpenLoopTask(void* arg) {
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;

    // driver init
  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

   // enable driver
  driver.enable();

  motor.linkDriver(&driver);
  motor.voltage_limit = 10;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // init motor hardware
  motor.init();

  motor.controller = MotionControlType::velocity_openloop;
  motor.target = 20; // one rotation per second
  motor.enable();  

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");
  command.add('M', doMotor, "Motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
  
  while (true) {
    // open loop velocity movement
    motor.move();
    vTaskDelay(pdMS_TO_TICKS(1));  // 让出 CPU

    // user communication
    command.run();
  }
}

void focDriverCloseLoopTask(void* arg) {
  driver.voltage_power_supply = 12;

    // driver init
  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

   // enable driver
  driver.enable();

  Wire.begin((int)PIN_I2C_SDA, (int)PIN_I2C_SCL);
  Wire.setClock(400000);
  sensor.init();
  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);

  // aligning voltage

  motor.voltage_sensor_align = 5;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // controller configuration 
  // default parameters in defaults.h

  // controller configuration based on the control type 
  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.02;

  // angle P controller -  default P=20
  motor.P_angle.P = 20;

  //  maximal velocity of the position control
  // default 20
  motor.velocity_limit = 10;

  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL; // set monitoring of d and q currents

  // init motor hardware
  motor.init();

  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }

  // set the target velocity [rad/s]
  // motor.target = 2; // Volts 

  // add target command T
  command.add('M', doMotor, "motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));
  _delay(1000);
  

  uint32_t lastPrintMs = 0;
  while (true) {
    motor.loopFOC();

    // open loop velocity movement
    motor.move();
    //vTaskDelay(pdMS_TO_TICKS(1));  // 让出 CPU

    // motor.monitor();

    uint32_t now = millis();
    if (now - lastPrintMs >= 5000) { // 20Hz
      lastPrintMs = now;
      Serial.print("angle=");
      Serial.print(sensor.getAngle());
      Serial.print("\tvel=");
      Serial.println(sensor.getVelocity());

      Serial.print("\tPID[P,I,D]=");
      Serial.print(motor.PID_velocity.P, 6);
      Serial.print(", ");
      Serial.print(motor.PID_velocity.I, 6);
      Serial.print(", ");
      Serial.println(motor.PID_velocity.D, 6);
    }

    // user communication
    command.run();
  }
}

void mpu6050TestTask(void* arg) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5); // 200Hz
  uint32_t lastPrintMs = 0;

  // ---- complementary filter state (deg) ----
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;

  // tuning
  const float alpha = 0.98f;  // gyro weight; 0.95~0.995 都可试

  // time
  uint32_t lastUs = micros();

  for (;;) {
    ImuRaw s;
    if (mpuRead14(s)) {
      // dt
      uint32_t nowUs = s.t_us;
      float dt = (nowUs - lastUs) * 1e-6f;
      if (dt <= 0 || dt > 0.1f) dt = 0.005f; // 防止异常 dt
      lastUs = nowUs;

      // scale
      const float ax = s.ax / 16384.0f;
      const float ay = s.ay / 16384.0f;
      const float az = s.az / 16384.0f;

      const float gx = s.gx / 131.0f; // deg/s
      const float gy = s.gy / 131.0f;
      const float gz = s.gz / 131.0f;

      // accel angles (deg)
      // roll: around X axis? 常用定义：roll = atan2(ay, az)
      float rollAcc  = atan2f(ay, az) * 180.0f / PI;
      // pitch: atan2(-ax, sqrt(ay^2+az^2))
      float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

      // gyro integrate
      // 注意：这里假设 gx->roll rate, gy->pitch rate, gz->yaw rate（与安装方向相关）
      float rollGyro  = roll  + gx * dt;
      float pitchGyro = pitch + gy * dt;
      float yawGyro   = yaw   + gz * dt;

      // complementary fuse
      roll  = alpha * rollGyro  + (1.0f - alpha) * rollAcc;
      pitch = alpha * pitchGyro + (1.0f - alpha) * pitchAcc;
      yaw   = yawGyro; // 没有磁力计就先纯积分（会漂移）

      // print 50Hz
      uint32_t nowMs = millis();
      if (nowMs - lastPrintMs >= 20) {
        lastPrintMs = nowMs;

        // 统一输出一行，便于网页解析
        // 格式：IMU,pitch,roll,yaw
        Serial.printf("IMU,%.2f,%.2f,%.2f\n", pitch, roll, yaw);
      }
    }

    vTaskDelayUntil(&last, period);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  SimpleFOCDebug::enable(&Serial);

  // LED 任务：低优先级即可
  xTaskCreatePinnedToCore(
    ledTask,
    "LED",
    2048,
    nullptr,
    1,
    nullptr,
    0
  );

  // FOC 任务：更高优先级，尽量不被别的任务打断
  if (false) {
    xTaskCreatePinnedToCore(
      focSensorTestTask,
      "FOC_Sensor",
      8192,
      nullptr,
      3,
      nullptr,
      1
    );
  }

  if (false) {
    xTaskCreatePinnedToCore(
      focDriverOpenLoopTask,
      "FOC_Driver",
      8192,
      nullptr,
      3,
      nullptr,
      1
    );
  }
  
  if (true) {
    xTaskCreatePinnedToCore(
      focDriverCloseLoopTask,
      "FOC_Driver",
      8192,
      nullptr,
      3,
      nullptr,
      1
    );
  }

  if (true) {
    WireIMU.begin(PIN_I2C_IMU_SDA, PIN_I2C_IMU_SCL);
    WireIMU.setClock(400000);
      // 唤醒 MPU6050
    mpuWrite(0x6B, 0x00);
    xTaskCreatePinnedToCore(
      mpu6050TestTask,
      "MPU6050",
      8192,
      nullptr,
      3,
      nullptr,
      0
    );
  }

  Serial.println("Tasks started");
}

void loop() {
  // loop 留空/低频 idle，别干扰 FOC
  vTaskDelay(pdMS_TO_TICKS(1000));
}
