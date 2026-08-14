/*
 * Host acceptance harness for imu_integrate.c (Mahony attitude only).
 *
 * Generates physically consistent IMU samples (attitude and accelerometer
 * computed at the same instant) and asserts:
 *   - calibration becomes ready
 *   - stationary: attitude stays near identity, still flag latches
 *   - rotation tracking: recovered angle stays in a useful band of truth
 */
#include "imu_integrate.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define G 9.80665f
#define DT 0.004f

static int g_fails;

static void fail(const char *msg)
{
    fprintf(stderr, "FAIL: %s\n", msg);
    g_fails++;
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

/* Rotate v by q (wxyz). */
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

/* Integrate body-rate gyro into a body→world quaternion (same convention as fusion). */
static void q_integrate_gyro(float q[4], const float gyro[3], float dt)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];
    float dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float dq1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float dq2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float dq3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
    q[0] = q0 + dq0 * dt;
    q[1] = q1 + dq1 * dt;
    q[2] = q2 + dq2 * dt;
    q[3] = q3 + dq3 * dt;
    q_normalize(q);
}

static float q_angle(const float q[4])
{
    float w = fabsf(q[0]);
    if (w > 1.f) w = 1.f;
    return 2.f * acosf(w);
}

static void calib_and_settle(imu_integrate_state_t *s)
{
    imu_integrate_init(s);
    float gyro0[3] = {0.001f, -0.002f, 0.0005f};
    float accel0[3] = {0.f, 0.f, G};
    for (int i = 0; i < 300; i++) {
        imu_integrate_calib_sample(s, gyro0, accel0, DT);
    }
    if (!s->ready) {
        fail("calib did not become ready");
        return;
    }
    for (int i = 0; i < (int)(2.0f / DT); i++) {
        imu_integrate_step(s, gyro0, accel0, DT);
    }
}

/*
 * Rotation-only about a body axis. Attitude and gravity-in-body are kept
 * consistent at every sample so Mahony sees a physically valid accel.
 */
static void run_rotation(imu_integrate_state_t *s, int axis, float angle_rad, float T)
{
    float q_truth[4] = {1.f, 0.f, 0.f, 0.f};
    float w = angle_rad / T;
    int n = (int)(T / DT);
    for (int i = 0; i < n; i++) {
        float gyro[3] = {0.f, 0.f, 0.f};
        gyro[axis] = w;
        q_integrate_gyro(q_truth, gyro, DT);
        float g_world[3] = {0.f, 0.f, G};
        float q_conj[4] = {q_truth[0], -q_truth[1], -q_truth[2], -q_truth[3]};
        float accel[3];
        q_rotate(q_conj, g_world, accel);
        float gyro_meas[3] = {
            gyro[0] + s->gyro_bias[0],
            gyro[1] + s->gyro_bias[1],
            gyro[2] + s->gyro_bias[2],
        };
        imu_integrate_step(s, gyro_meas, accel, DT);
    }
    float g_world[3] = {0.f, 0.f, G};
    float q_conj[4] = {q_truth[0], -q_truth[1], -q_truth[2], -q_truth[3]};
    float accel[3];
    q_rotate(q_conj, g_world, accel);
    float gyro_still[3] = {
        s->gyro_bias[0],
        s->gyro_bias[1],
        s->gyro_bias[2],
    };
    for (int i = 0; i < (int)(1.0f / DT); i++) {
        imu_integrate_step(s, gyro_still, accel, DT);
    }
}

static int self_test(void)
{
    g_fails = 0;
    imu_integrate_state_t s;

    /* --- Stationary attitude --- */
    calib_and_settle(&s);
    float gyro0[3] = {s.gyro_bias[0], s.gyro_bias[1], s.gyro_bias[2]};
    float accel0[3] = {0.f, 0.f, G};
    for (int i = 0; i < (int)(30.0f / DT); i++) {
        imu_integrate_step(&s, gyro0, accel0, DT);
    }
    float tilt = q_angle(s.q);
    printf("stationary tilt = %.3f deg  still=%d\n", tilt * 180.f / (float)M_PI, s.still ? 1 : 0);
    if (tilt > 2.f * (float)M_PI / 180.f) fail("stationary tilt > 2 deg");
    if (!s.still) fail("stationary still flag not set");

    /* --- Rotation tracking --- */
    const float angles_deg[] = {20.f, 30.f, 45.f};
    const char *axis_name[] = {"X", "Y", "Z"};
    for (int axis = 0; axis < 3; axis++) {
        for (size_t k = 0; k < sizeof(angles_deg) / sizeof(angles_deg[0]); k++) {
            float ang = angles_deg[k] * (float)M_PI / 180.f;
            float T = 0.50f;
            calib_and_settle(&s);
            run_rotation(&s, axis, ang, T);
            float recovered = q_angle(s.q);
            float frac = recovered / ang;
            printf("rot axis=%s %.0fdeg → recovered=%.1fdeg (%.0f%%)\n",
                   axis_name[axis], angles_deg[k], recovered * 180.f / (float)M_PI, 100.f * frac);
            if (frac < 0.70f) {
                char buf[128];
                snprintf(buf, sizeof(buf),
                         "recovered only %.0f%% of %.0fdeg about axis %s",
                         100.f * frac, angles_deg[k], axis_name[axis]);
                fail(buf);
            }
            if (frac > 1.30f) {
                char buf[128];
                snprintf(buf, sizeof(buf),
                         "overshot to %.0f%% of %.0fdeg about axis %s",
                         100.f * frac, angles_deg[k], axis_name[axis]);
                fail(buf);
            }
        }
    }

    if (g_fails) {
        fprintf(stderr, "SELF-TEST FAILED (%d)\n", g_fails);
        return 1;
    }
    printf("SELF-TEST OK\n");
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "--self-test") == 0) {
        return self_test();
    }
    fprintf(stderr, "Usage: %s --self-test\n", argv[0]);
    return 2;
}
