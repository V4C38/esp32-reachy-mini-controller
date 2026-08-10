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

typedef enum {
    NET_HOST_IDLE = 0,
    NET_HOST_ENGAGED,
    NET_HOST_RESETTING,
    NET_HOST_FAULT,
} net_host_mode_t;

typedef struct {
    net_ws_phase_t phase;
    bool linked;              /* phase == LINKED (socket open) */
    bool hello_ok;            /* host accepted protocol v2 hello */
    bool busy;                /* host mode == resetting */
    bool robot_ok;            /* last host_state.robot */
    net_host_mode_t host_mode;
    uint32_t send_fails;
    uint32_t reconnects;
    uint32_t boot_id;
    uint32_t next_op_id;
} net_ws_status_t;

esp_err_t net_ws_start(const char *host, uint16_t port);
void net_ws_stop(void);
bool net_ws_connected(void);

/* True once a client exists — it retries on its own, so callers must not
 * tear it down and rebuild it just because the socket is currently down. */
bool net_ws_running(void);
net_ws_status_t net_ws_status(void);
net_ws_phase_t net_ws_phase(void);

/* Send hello once after link if not yet exchanged. */
void net_ws_service(void);

/* Send protocol-v2 sample at caller's rate (fire-and-forget). */
void net_ws_send_sample(const imu_integrate_state_t *imu, bool engaged, float gain, uint32_t seq);

/* Reset with boot-scoped operation token. */
void net_ws_send_reset(void);
