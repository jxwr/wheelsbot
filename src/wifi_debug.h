#pragma once

#include "control/balance_controller.h"

struct AppContext;  // forward declaration

void wifi_debug_init(AppContext& ctx);
void wifiDebugTask(void* arg);

// Parameter persistence helpers
bool loadBalanceParams(wheelsbot::control::BalanceController::Params& p);
bool saveBalanceParams(const wheelsbot::control::BalanceController::Params& p);
