#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "imu_integrate.h"

typedef enum {
    NET_WS_DOWN = 0,
    NET_WS_CONNECTING,
    NET_WS_LINKED,
} net_ws_phase_t;

typedef struct {
    net_ws_phase_t phase;
    bool linked;       /* phase == LINKED (socket open) */
    bool robot_ok;     /* last status_result.robot */
    bool busy;         /* reset in progress on server */
    uint32_t send_fails;
    uint32_t reconnects;
} net_ws_status_t;

esp_err_t net_ws_start(const char *host, uint16_t port);
void net_ws_stop(void);
bool net_ws_connected(void);

/* True once a client exists — it retries on its own, so callers must not
 * tear it down and rebuild it just because the socket is currently down. */
bool net_ws_running(void);
net_ws_status_t net_ws_status(void);
net_ws_phase_t net_ws_phase(void);

/* Send controller_state at caller's rate. */
void net_ws_send_state(const imu_integrate_state_t *imu, bool engaged, float gain, uint32_t seq);
void net_ws_send_reset(void);
void net_ws_send_status(void);
