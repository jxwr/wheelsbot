#pragma once

#include "app_context.h"

namespace wheelsbot {
namespace debug {

// ============================================================
// Serial Debug Interface
// Provides CSV-based telemetry and parameter control
// Designed for AI agent automated tuning
// ============================================================

class SerialDebug {
 public:
  SerialDebug();

  // Initialize debug subsystem
  void init(AppContext* ctx);

  // Process incoming commands (call in loop or task)
  void processCommands();

  // Send data stream if enabled (call in high-freq loop)
  void sendDataIfReady();

  // Configuration
  void setStreamRate(uint32_t hz);  // 0 = disabled
  void enableStream(bool enable);

 private:
  AppContext* ctx_ = nullptr;

  // Stream state
  bool stream_enabled_ = false;
  uint32_t stream_interval_ms_ = 20;  // 50Hz default
  uint32_t last_stream_ms_ = 0;

  // Command handlers
  void handleCommand(const char* cmd);
  void cmdStatus();
  void cmdSensors();
  void cmdParams();
  void cmdGet(const char* param_name);
  void cmdSet(const char* param_name, float value);
  void cmdStream(uint32_t hz);
  void cmdEnable();
  void cmdDisable();
  void cmdSave();
  void cmdLoad();
  void cmdMotor(float voltage_limit);

  // Output helpers
  void sendData();
  void sendOk();
  void sendError(const char* msg);

  // Parameter mapping
  bool getParamValue(const char* name, float& value);
  bool setParamValue(const char* name, float value);

  // String parsing helpers
  static void trim(char* str);
  static bool parseFloat(const char* str, float& out);
};

}  // namespace debug
}  // namespace wheelsbot
