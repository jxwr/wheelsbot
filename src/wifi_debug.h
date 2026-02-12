#pragma once

#include "control/cascade_controller.h"

struct AppContext;  // forward declaration

void wifi_debug_init(AppContext& ctx);
void wifiDebugTask(void* arg);

// Parameter persistence helpers
bool loadCascadeParams(wheelsbot::control::CascadeController::Params& p);
