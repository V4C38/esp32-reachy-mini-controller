#include "imu_integrate.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define G_EARTH 9.80665f

static float vnorm3(const float a[3])
{
    return sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

static void q_normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-9f) {
        q[0] = 1.f; q[1] = q[2] = q[3] = 0.f;
        return;
    }
    q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
}

/* Rotate vector by quaternion (wxyz). */
static void q_rotate(const float q[4], const float v[3], float out[3])
{
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    float uvx = qy * v[2] - qz * v[1];
    float uvy = qz * v[0] - qx * v[2];
    float uvz = qx * v[1] - qy * v[0];
    float uuvx = qy * uvz - qz * uvy;
    float uuvy = qz * uvx - qx * uvz;
    float uuvz = qx * uvy - qy * uvx;
    out[0] = v[0] + 2.f * (qw * uvx + uuvx);
    out[1] = v[1] + 2.f * (qw * uvy + uuvy);
    out[2] = v[2] + 2.f * (qw * uvz + uuvz);
}

void imu_integrate_init(imu_integrate_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->q[0] = 1.f;
    s->kp_high = 2.0f;
    s->kp_low = 0.05f;
    s->ki = 0.0f;
    s->a_still = 0.20f;
    s->w_still = 0.08f;
    s->still_dwell = 0.10f;
    s->still_t = 0.f;
}

/* Internal calib accumulators — single instance (host sim + firmware). */
static float s_bias_sum[3];
static float s_bias_sumsq[3];
static float s_calib_t;
static const float CALIB_SEC = 1.0f;
static const float CALIB_VAR_MAX = 0.0025f; /* (rad/s)^2 */

bool imu_integrate_calib_sample(imu_integrate_state_t *s,
                                const float gyro[3],
                                const float accel[3],
                                float dt)
{
    (void)accel;
    if (s->ready) return true;
    if (s_calib_t <= 0.f) {
        s_bias_sum[0] = s_bias_sum[1] = s_bias_sum[2] = 0.f;
        s_bias_sumsq[0] = s_bias_sumsq[1] = s_bias_sumsq[2] = 0.f;
    }
    for (int i = 0; i < 3; i++) {
        s_bias_sum[i] += gyro[i];
        s_bias_sumsq[i] += gyro[i] * gyro[i];
    }
    s_calib_t += dt;
    if (s_calib_t < CALIB_SEC) return false;

    int n = (int)(s_calib_t / (dt > 1e-6f ? dt : 0.004f));
    if (n < 10) n = 10;
    float var_ok = 1.f;
    for (int i = 0; i < 3; i++) {
        float mean = s_bias_sum[i] / (float)n;
        float var = s_bias_sumsq[i] / (float)n - mean * mean;
        if (var > CALIB_VAR_MAX) var_ok = 0.f;
        s->gyro_bias[i] = mean;
    }
    s_calib_t = 0.f;
    if (!var_ok) {
        return false;
    }
    s->ready = true;
    return true;
}

void imu_integrate_step(imu_integrate_state_t *s,
                        const float gyro_in[3],
                        const float accel[3],
                        float dt)
{
    if (dt <= 0.f || dt > 0.05f) dt = 0.004f;

    float gyro[3] = {
        gyro_in[0] - s->gyro_bias[0],
        gyro_in[1] - s->gyro_bias[1],
        gyro_in[2] - s->gyro_bias[2],
    };

    float an = vnorm3(accel);
    float wn = vnorm3(gyro);

    /* Preliminary a_lin from *previous* attitude — used for still detection. */
    float g_world[3] = {0.f, 0.f, G_EARTH};
    float q_conj0[4] = {s->q[0], -s->q[1], -s->q[2], -s->q[3]};
    float g_body0[3];
    q_rotate(q_conj0, g_world, g_body0);
    float a_lin0[3] = {
        accel[0] - g_body0[0],
        accel[1] - g_body0[1],
        accel[2] - g_body0[2],
    };
    float alin_n = vnorm3(a_lin0);

    bool hard_still = (alin_n < s->a_still) && (wn < s->w_still);
    if (hard_still) {
        s->still_t += dt;
    } else {
        s->still_t = 0.f;
    }
    s->still = hard_still && (s->still_t >= s->still_dwell);

    float kp = s->still ? s->kp_high : s->kp_low;

    float q0 = s->q[0], q1 = s->q[1], q2 = s->q[2], q3 = s->q[3];
    float vx = 2.f * (q1 * q3 - q0 * q2);
    float vy = 2.f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float ax = accel[0], ay = accel[1], az = accel[2];
    if (an > 1e-3f) {
        ax /= an; ay /= an; az /= an;
    }
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    float gx = gyro[0] + kp * ex;
    float gy = gyro[1] + kp * ey;
    float gz = gyro[2] + kp * ez;

    float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float dq1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float dq2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float dq3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
    s->q[0] = q0 + dq0 * dt;
    s->q[1] = q1 + dq1 * dt;
    s->q[2] = q2 + dq2 * dt;
    s->q[3] = q3 + dq3 * dt;
    q_normalize(s->q);
}
