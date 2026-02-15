#include "serial_debug.h"
#include "wifi_debug.h"
#include <Arduino.h>
#include <cstring>
#include <cstdlib>

namespace wheelsbot {
namespace debug {

// ============================================================
// Constructor
// ============================================================

SerialDebug::SerialDebug() {}

// ============================================================
// Initialization
// ============================================================

void SerialDebug::init(AppContext* ctx) {
  ctx_ = ctx;
  Serial.println("# Serial Debug Ready (CSV mode)");
  Serial.println("# Commands: STATUS, SENSORS, PARAMS, GET, SET, STREAM, ENABLE, DISABLE, SAVE, LOAD");
}

// ============================================================
// Main Processing
// ============================================================

void SerialDebug::processCommands() {
  if (!ctx_) return;

  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      handleCommand(line.c_str());
    }
  }
}

void SerialDebug::sendDataIfReady() {
  if (!ctx_ || !stream_enabled_) return;

  uint32_t now = millis();
  if (now - last_stream_ms_ >= stream_interval_ms_) {
    sendData();
    last_stream_ms_ = now;
  }
}

// ============================================================
// Command Parsing and Dispatch
// ============================================================

void SerialDebug::handleCommand(const char* cmd) {
  char buf[128];
  strncpy(buf, cmd, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  // Parse command and arguments
  char* token = strtok(buf, " ");
  if (!token) return;

  if (strcmp(token, "STATUS") == 0) {
    cmdStatus();
  } else if (strcmp(token, "SENSORS") == 0) {
    cmdSensors();
  } else if (strcmp(token, "PARAMS") == 0) {
    cmdParams();
  } else if (strcmp(token, "GET") == 0) {
    char* param = strtok(nullptr, " ");
    if (param) cmdGet(param);
    else sendError("Missing parameter name");
  } else if (strcmp(token, "SET") == 0) {
    char* param = strtok(nullptr, " ");
    char* value_str = strtok(nullptr, " ");
    if (param && value_str) {
      float value;
      if (parseFloat(value_str, value)) {
        cmdSet(param, value);
      } else {
        sendError("Invalid value");
      }
    } else {
      sendError("Missing parameter or value");
    }
  } else if (strcmp(token, "STREAM") == 0) {
    char* hz_str = strtok(nullptr, " ");
    if (hz_str) {
      uint32_t hz = atoi(hz_str);
      cmdStream(hz);
    } else {
      sendError("Missing frequency");
    }
  } else if (strcmp(token, "ENABLE") == 0) {
    cmdEnable();
  } else if (strcmp(token, "DISABLE") == 0) {
    cmdDisable();
  } else if (strcmp(token, "SAVE") == 0) {
    cmdSave();
  } else if (strcmp(token, "LOAD") == 0) {
    cmdLoad();
  } else if (strcmp(token, "MOTOR") == 0) {
    char* limit_str = strtok(nullptr, " ");
    if (limit_str) {
      float limit;
      if (parseFloat(limit_str, limit)) {
        cmdMotor(limit);
      } else {
        sendError("Invalid voltage limit");
      }
    } else {
      sendError("Missing voltage limit");
    }
  } else {
    sendError("Unknown command");
  }
}

// ============================================================
// Command Implementations
// ============================================================

void SerialDebug::cmdStatus() {
  // CSV format: state,faults,enabled,voltage,loop_hz
  Serial.println("STATUS,state,faults,enabled,voltage,loop_hz");

  int state = ctx_->bal_state.ok ? 1 : 0;
  uint32_t faults = ctx_->balance.getFaultFlags();
  int enabled = ctx_->cmd_state.balance_enable ? 1 : 0;
  float voltage = SUPPLY_VOLTAGE;  // TODO: Add battery monitoring

  // Calculate loop frequency from balance controller stats
  control::FrequencyStats freq_stats;
  ctx_->balance.getFrequencyStats(freq_stats);
  float loop_hz = freq_stats.angle_hz;

  Serial.printf("%d,%u,%d,%.2f,%.1f\n",
    state, faults, enabled, voltage, loop_hz);
}

void SerialDebug::cmdSensors() {
  // CSV format: pitch,pitch_rate,yaw_rate,wheel_l,wheel_r,motor_l,motor_r
  Serial.println("SENSORS,pitch,pitch_rate,yaw_rate,wheel_l,wheel_r,motor_l,motor_r");

  float pitch_rad = ctx_->imu_state.pitch_deg * 3.14159f / 180.0f;
  Serial.printf("%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f\n",
    pitch_rad,
    ctx_->imu_state.gy,   // pitch_rate (rad/s)
    ctx_->imu_state.gz,   // yaw_rate (rad/s)
    ctx_->lmotor.shaft_velocity,
    ctx_->rmotor.shaft_velocity,
    ctx_->lmotor.voltage.q,
    ctx_->rmotor.voltage.q);
}

void SerialDebug::cmdParams() {
  // Get current parameters
  control::BalanceController::Params p;
  ctx_->balance.getParams(p);

  // CSV header
  Serial.println("PARAMS,velocity_kp,velocity_ki,velocity_kd,velocity_max_tilt,"
                 "angle_kp,angle_ki,angle_gyro_kd,angle_d_alpha,angle_max_out,"
                 "yaw_kd,max_tilt,ramp_time,pitch_offset,pitch_cmd_rate_limit,"
                 "sensor_timeout,velocity_decimation");

  // CSV values
  Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%.4f,%.4f,%.2f,%u\n",
    p.velocity_kp,
    p.velocity_ki,
    p.velocity_kd,
    p.velocity_max_tilt,
    p.angle_kp,
    p.angle_ki,
    p.angle_gyro_kd,
    p.angle_d_alpha,
    p.angle_max_out,
    p.yaw_kd,
    p.max_tilt,
    p.ramp_time,
    p.pitch_offset,
    p.pitch_cmd_rate_limit,
    p.sensor_timeout,
    p.velocity_decimation);
}

void SerialDebug::cmdGet(const char* param_name) {
  float value;
  if (getParamValue(param_name, value)) {
    Serial.printf("VALUE,%s,%.4f\n", param_name, value);
  } else {
    sendError("Unknown parameter");
  }
}

void SerialDebug::cmdSet(const char* param_name, float value) {
  if (setParamValue(param_name, value)) {
    sendOk();
  } else {
    sendError("Unknown parameter or invalid value");
  }
}

void SerialDebug::cmdStream(uint32_t hz) {
  if (hz > 200) {
    sendError("Max frequency is 200Hz");
    return;
  }

  stream_enabled_ = (hz > 0);
  stream_interval_ms_ = (hz > 0) ? (1000 / hz) : 0;
  sendOk();
}

void SerialDebug::cmdEnable() {
  ctx_->cmd_state.balance_enable = true;
  sendOk();
}

void SerialDebug::cmdDisable() {
  ctx_->cmd_state.balance_enable = false;
  sendOk();
}

void SerialDebug::cmdSave() {
  control::BalanceController::Params p;
  ctx_->balance.getParams(p);
  saveBalanceParams(p);
  sendOk();
}

void SerialDebug::cmdLoad() {
  control::BalanceController::Params p;
  if (loadBalanceParams(p)) {
    ctx_->balance.setParams(p);
    sendOk();
  } else {
    sendError("Load failed");
  }
}

void SerialDebug::cmdMotor(float voltage_limit) {
  if (voltage_limit < 0.5f || voltage_limit > 12.0f) {
    sendError("Voltage limit must be 0.5-12.0V");
    return;
  }

  ctx_->lmotor.voltage_limit = voltage_limit;
  ctx_->rmotor.voltage_limit = voltage_limit;

  Serial.printf("MOTOR,voltage_limit,%.2f\n", voltage_limit);
  sendOk();
}

// ============================================================
// Data Streaming
// ============================================================

void SerialDebug::sendData() {
  // DATA,<timestamp>,<pitch>,<pitch_rate>,<yaw_rate>,<wl>,<wr>,<ml>,<mr>,<voltage>,<state>,<faults>
  uint32_t ts = millis();
  int state = ctx_->bal_state.ok ? 1 : 0;
  uint32_t faults = ctx_->balance.getFaultFlags();

  float pitch_rad = ctx_->imu_state.pitch_deg * 3.14159f / 180.0f;
  Serial.printf("DATA,%u,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%u\n",
    ts,
    pitch_rad,
    ctx_->imu_state.gy,  // pitch_rate (rad/s)
    ctx_->imu_state.gz,  // yaw_rate (rad/s)
    ctx_->lmotor.shaft_velocity,
    ctx_->rmotor.shaft_velocity,
    ctx_->lmotor.voltage.q,
    ctx_->rmotor.voltage.q,
    SUPPLY_VOLTAGE,  // TODO: Add battery monitoring
    state,
    faults);
}

// ============================================================
// Output Helpers
// ============================================================

void SerialDebug::sendOk() {
  Serial.println("OK");
}

void SerialDebug::sendError(const char* msg) {
  Serial.printf("ERROR,%s\n", msg);
}

// ============================================================
// Parameter Mapping
// ============================================================

bool SerialDebug::getParamValue(const char* name, float& value) {
  control::BalanceController::Params p;
  ctx_->balance.getParams(p);

  if (strcmp(name, "velocity_kp") == 0) value = p.velocity_kp;
  else if (strcmp(name, "velocity_ki") == 0) value = p.velocity_ki;
  else if (strcmp(name, "velocity_kd") == 0) value = p.velocity_kd;
  else if (strcmp(name, "velocity_max_tilt") == 0) value = p.velocity_max_tilt;
  else if (strcmp(name, "angle_kp") == 0) value = p.angle_kp;
  else if (strcmp(name, "angle_ki") == 0) value = p.angle_ki;
  else if (strcmp(name, "angle_gyro_kd") == 0) value = p.angle_gyro_kd;
  else if (strcmp(name, "angle_d_alpha") == 0) value = p.angle_d_alpha;
  else if (strcmp(name, "angle_max_out") == 0) value = p.angle_max_out;
  else if (strcmp(name, "yaw_kd") == 0) value = p.yaw_kd;
  else if (strcmp(name, "max_tilt") == 0) value = p.max_tilt;
  else if (strcmp(name, "ramp_time") == 0) value = p.ramp_time;
  else if (strcmp(name, "pitch_offset") == 0) value = p.pitch_offset;
  else if (strcmp(name, "pitch_cmd_rate_limit") == 0) value = p.pitch_cmd_rate_limit;
  else if (strcmp(name, "sensor_timeout") == 0) value = p.sensor_timeout;
  else if (strcmp(name, "velocity_decimation") == 0) value = (float)p.velocity_decimation;
  else return false;

  return true;
}

bool SerialDebug::setParamValue(const char* name, float value) {
  control::BalanceController::Params p;
  ctx_->balance.getParams(p);

  bool found = true;
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
  else if (strcmp(name, "velocity_decimation") == 0) p.velocity_decimation = (uint32_t)value;
  else found = false;

  if (found) {
    ctx_->balance.setParams(p);
    return true;
  }
  return false;
}

// ============================================================
// String Helpers
// ============================================================

void SerialDebug::trim(char* str) {
  char* end;
  while (*str == ' ' || *str == '\t' || *str == '\n' || *str == '\r') str++;
  if (*str == 0) return;
  end = str + strlen(str) - 1;
  while (end > str && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) end--;
  end[1] = '\0';
}

bool SerialDebug::parseFloat(const char* str, float& out) {
  char* endptr;
  out = strtof(str, &endptr);
  return endptr != str && (*endptr == '\0' || *endptr == ' ');
}

}  // namespace debug
}  // namespace wheelsbot
