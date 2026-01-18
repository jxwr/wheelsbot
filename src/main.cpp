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

static constexpr int   LED_PIN = 18;

// ====== Motor config ======
static constexpr int   POLE_PAIRS      = 7;
static constexpr float SUPPLY_VOLTAGE  = 12.0f;
static constexpr float VOLTAGE_LIMIT   = 3.0f;   // 先小一点，确认方向/闭环稳定再加

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

    Serial.println("LED: blink!");
  }
}


// ---------------- FOC task ----------------
void focSensorTestTask(void* arg) {
  // I2C + sensor init
  Wire.begin((int)PIN_I2C_SDA, (int)PIN_I2C_SCL);
  Wire.setClock(100000);

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

void focDriverTestTask(void* arg) {
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 6;

    // driver init
  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

   // enable driver
  driver.enable();

  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 10;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

  // set the target velocity [rad/s]
  motor.target = 20; // one rotation per second

  motor.enable();  

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  Serial.println("Driver ready!");

  _delay(5000);
  
  while (true) {
    // open loop velocity movement
    motor.move();
    vTaskDelay(pdMS_TO_TICKS(1));  // 让出 CPU

    // user communication
    command.run();
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
  /*
  xTaskCreatePinnedToCore(
    focSensorTestTask,
    "FOC_Sensor",
    8192,
    nullptr,
    3,
    nullptr,
    1
  );
  */

  xTaskCreatePinnedToCore(
     focDriverTestTask,
     "FOC_Driver",
     8192,
     nullptr,
     3,
     nullptr,
     1
  );

  Serial.println("Tasks started");
}

void loop() {
  // loop 留空/低频 idle，别干扰 FOC
  vTaskDelay(pdMS_TO_TICKS(1000));
}
