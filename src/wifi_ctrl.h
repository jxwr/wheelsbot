/*
 * wifi_ctrl.h - WiFi control and telemetry interface
 */
#ifndef WIFI_CTRL_H
#define WIFI_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

void wifi_ctrl_init(void);
void wifi_ctrl_task(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_CTRL_H */
