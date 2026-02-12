#pragma once

#include "control/cascade_controller.h"

void wifi_debug_init();
void wifiDebugTask(void* arg);

// Parameter persistence helpers
bool loadCascadeParams(wheelsbot::control::CascadeController::Params& p);
