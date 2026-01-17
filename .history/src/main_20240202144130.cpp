#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <iostream>
#include <sstream>

// SDA 21
// SCL 22
// magnetic sensor instance - I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(14, 27, 26, 13);
// angle set point variable
float target_angle = 0;
float target_angle0 = 0;//电机0的值
float target_angle1 = 0;//电机1的值
bool bHassend ;

// attractor angle variable
float attract_angle = 0;

// distance between attraction points
float attractor_distance = 10*_PI/180.0; // dimp each 45 degrees

float findAttractor(float current_angle){
  return round(current_angle/attractor_distance)*attractor_distance;
}

// instantiate the commander
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {
  // initialise magnetic sensor hardware
  Wire.setClock(400000);
  sensor.init();
//新增一个磁编码器
  Wire1.setClock(400000);
  Wire1.begin(19,23,(uint32_t)400000);
  sensor1.init(&Wire1);

  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // power supply voltage [V]
  driver.voltage_power_supply = 12;  
  driver.init();  
  // link the motor and the driver
  motor.linkDriver(&driver);  
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;  
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;  
  // velocity PI controller parameters
  motor.PID_velocity.P = 10;
  motor.PID_velocity.I = 0.005;  
//  motor.PID_velocity.D = 0.01;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6;  
  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;  
  // angle P controller
  motor.P_angle.P = 30;  //位置PID的P值
  // maximal velocity of the position control
  motor.velocity_limit = 12;
  motor.PID_velocity.output_ramp = 1200;//调整这个值可以影响电机的加速和减速性能。较高的值会使电机加速和减速更快，但可能导致振动或电流峰值。
  motor.LPF_velocity.Tf = 0.01f;//这可以滤除电机的噪声和高频振动，从而使速度控制更加稳定。  
//新增一个电机
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.controller = MotionControlType::velocity;
  motor1.PID_velocity.P = 0.05;
  motor1.PID_velocity.I = 0.001;
//  motor1.PID_velocity.D = 0.0011;
  motor1.voltage_limit = 6;
  motor1.LPF_velocity.Tf = 0.01f;
  motor1.P_angle.P = 30;
//  motor1.P_angle.I = 0.08;
  motor1.velocity_limit = 50;
  motor1.PID_velocity.output_ramp = 1200;//调整这个值可以影响电机的加速和减速性能。较高的值会使电机加速和减速更快，但可能导致振动或电流峰值。
  motor1.LPF_velocity.Tf = 0.01f;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

//新增电机初始化
  motor1.useMonitoring(Serial);
  motor1.init();
  motor1.initFOC();

  Serial.println(F("电机准备就绪。"));
  Serial.println(F("通过串口发送命令, 角度Axx, 速度Vxx, 扭矩Txx:"));
  _delay(1000);
}
//==============串口接收==============
int commaPosition;
char comma;//用于读取逗号
String serialReceiveUserCommand() {  
  // a string to hold incoming data
  static String received_chars;  
  String command = "";
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {      
      // execute the user command
      command = received_chars;
      std::string str = command.c_str();//将String类转换为string类数据
      std::istringstream ss(str);  //定义字符串流    
      commaPosition = command.indexOf('\n');//检测字符串中的换行符，返回换行符的位置
      if(commaPosition != -1)//如果有换行存在就向下执行
      {
          ss>>target_angle0>>comma>>target_angle1;//字符串流输出到电机0和电机1
          target_angle = command.substring(0,commaPosition).toDouble();  //电机角度，双浮点数据类型
          bHassend = false ;//每发送一次位置，发一次标记
//          Serial.println(target_angle);
//          float p_now = sensor.getAngle();
//          Serial.print(p_now);
          Serial.print(target_angle0);
          Serial.print(comma);
          Serial.println(target_angle1);
      }
      // reset the command buffer 
      received_chars = "";
    }
  }
  return command;
}

//判断是否到达指定的位置
float ACCEPTABLE_ERROR = 0.1;// ACCEPTABLE_ERROR是可接受的误差范围
bool isMotorAtPosition(float targetPosition) {
    float positionError = abs(targetPosition - target_angle);
    return positionError <= ACCEPTABLE_ERROR;
}

static String received_chars;  
void loop() {
  motor.loopFOC();
  float torque = motor.PID_velocity(attract_angle - motor.shaft_angle);
  motor.move(torque);
  
  // calculate the attractor
  attract_angle = findAttractor(motor.shaft_angle);

  motor.monitor();

//   // main FOC algorithm function
//     motor.loopFOC();//给上劲
//     motor1.loopFOC();//给上劲
//     motor.move(target_angle0);//电机0实时控制到目标角度
//     motor1.move(target_angle1);//电机1实时控制到目标角度
//     serialReceiveUserCommand();//通过串口获取位置信息

    //实现双电机跟踪，motor主动，motor1跟随，motor设置为扭矩模式
    // float master_angle = sensor.getAngle();
    // motor1.move(master_angle);
    // if(isMotorAtPosition(p_now) && !(bHassend))
    // {        
    //     bHassend = true ;
    // }

//读取不同的命令，实现位置模式、速度模式、扭矩模式的切换
    // String command = "";
    // while (Serial.available()) 
    // {
    //     char inChar = (char)Serial.read();
    //     received_chars += inChar;
    //     if (inChar == '\n') 
    //     {      
    //         command = received_chars;
    //         commaPosition = command.indexOf('\n');//检测字符串中的换行符
    //         if(commaPosition != -1)//如果有换行存在就向下执行
    //         {
    //             char ch = command[0];
    //             if (ch == 'A' )
    //             {
    //                 motor.controller = MotionControlType::angle;
    //                 target_angle = command.substring(1,commaPosition).toDouble();  //电机角度                    
    //                 Serial.println(target_angle);
    //             } 
    //             else if(ch == 'V')
    //             {
    //                 motor.controller = MotionControlType::velocity;
    //                 target_angle = command.substring(1,commaPosition).toDouble();  //电机速度
    //                 if(abs(target_angle)>300)//限制最大弧度转速不超过每秒300弧度
    //                     target_angle=300;                    
    //                 Serial.println(target_angle);
    //             }
    //             else if(ch == 'T')
    //             {
    //                 motor.controller = MotionControlType::torque;
    //                 motor.velocity_limit= 20;
    //                 target_angle = command.substring(1,commaPosition).toDouble();  //电机扭矩
    //                 if(abs(target_angle)>12)//限制最大
    //                     target_angle=12;                    
    //                 Serial.println(target_angle);
    //             }
    //         received_chars = "";
    //         }
    //     }
    // }
}

