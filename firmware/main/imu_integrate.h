#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q[4];          // wxyz — body→world
    float gyro_bias[3];
    float v[3];          // world-frame m/s
    float p[3];          // world-frame metres
    bool ready;
    bool still;

    /* Mahony gains */
    float kp_high;
    float kp_low;
    float ki;

    /* Still detection */
    float a_still;       /* |a_lin| threshold for hard still (m/s^2) */
    float w_still;       /* |w| threshold for hard still (rad/s) */
    float still_dwell;   /* seconds of hard_still before kp_high */
    float still_t;       /* accumulated still time */

    /* Velocity decay when genuinely still (time constant, seconds) */
    float v_decay_tau;

    /* Accel deadband at rest; grows with |w| */
    float a_dead;
    float a_dead_w_scale; /* extra deadband per (rad/s) of |w| */

    /* Gate translation integration while rotating */
    float w_gate;        /* |w| above which a_world is not integrated */

    float p_max;         /* metres; clamp with outward-velocity zeroing */
} imu_integrate_state_t;

void imu_integrate_init(imu_integrate_state_t *s);
void imu_integrate_reset_motion(imu_integrate_state_t *s);

/* calib_sample: accumulate gyro while still; returns true when ready */
bool imu_integrate_calib_sample(imu_integrate_state_t *s,
                                const float gyro[3],
                                const float accel[3],
                                float dt);

/* Step fusion + ZUPT. accel/gyro in m/s^2 and rad/s, already axis-remapped. */
void imu_integrate_step(imu_integrate_state_t *s,
                        const float gyro[3],
                        const float accel[3],
                        float dt);

#ifdef __cplusplus
}
#endif
