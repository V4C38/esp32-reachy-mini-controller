#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "imu_integrate.h"

typedef enum {
    NET_HOST_IDLE = 0,
    NET_HOST_ENGAGED,
    NET_HOST_RESETTING,
    NET_HOST_FAULT,
} net_host_mode_t;

typedef struct {
    bool linked;              /* last host reply younger than 1 s */
    bool busy;                /* host mode == resetting */
    bool robot_ok;            /* last reply.robot */
    net_host_mode_t host_mode;
    uint32_t send_fails;
    uint32_t send_ok;
    uint32_t boot_id;
    uint32_t next_op_id;
} net_link_status_t;

esp_err_t net_link_open(const char *host, uint16_t port);
void net_link_close(void);
bool net_link_ready(void);
bool net_link_linked(void);
net_link_status_t net_link_status(void);

/* Drain inbound datagrams; send hello while unlinked. */
void net_link_service(void);

/* Fire-and-forget sample. A full TX queue drops the datagram. */
bool net_link_send_sample(const imu_integrate_state_t *imu, bool engaged, float gain, uint32_t seq);

/* Arm a reset token. Repeated on every sample until a terminal op_status. */
bool net_link_begin_reset(void);

/* True once when a terminal op_status (completed/failed) arrives. */
bool net_link_take_reset_outcome(bool *failed);
