/*
 * Host acceptance harness for imu_integrate.c.
 *
 * Generates physically consistent IMU samples (attitude and accelerometer
 * computed at the same instant) and asserts:
 *   - per-direction translation: correct sign, recovered magnitude band
 *   - rotation-only: phantom translation stays below a useful-signal fraction
 *   - stationary: no creep
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

/* Min-jerk acceleration for displacement d over duration T at time t. */
static float minjerk_accel(float d, float T, float t)
{
    if (t < 0.f || t > T) return 0.f;
    float s = t / T;
    return d / (T * T) * (60.f * s - 180.f * s * s + 120.f * s * s * s);
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
    /* Settle attitude under gravity. */
    for (int i = 0; i < (int)(2.0f / DT); i++) {
        imu_integrate_step(s, gyro0, accel0, DT);
    }
    imu_integrate_reset_motion(s);
}

/* Feed a pure world-axis translation with upright attitude (gravity along +Z world). */
static void run_translation(imu_integrate_state_t *s, int axis, float d, float T)
{
    float gyro0[3] = {0.f, 0.f, 0.f};
    int n = (int)(T / DT);
    for (int i = 0; i < n; i++) {
        float a_lin = minjerk_accel(d, T, i * DT);
        float accel[3] = {0.f, 0.f, G};
        accel[axis] += a_lin;
        /* Gyro includes residual bias so the estimator sees realistic noise. */
        float gyro[3] = {
            s->gyro_bias[0],
            s->gyro_bias[1],
            s->gyro_bias[2],
        };
        imu_integrate_step(s, gyro, accel, DT);
        (void)gyro0;
    }
    /* Settle still. */
    float accel0[3] = {0.f, 0.f, G};
    float gyro_still[3] = {
        s->gyro_bias[0],
        s->gyro_bias[1],
        s->gyro_bias[2],
    };
    for (int i = 0; i < (int)(1.0f / DT); i++) {
        imu_integrate_step(s, gyro_still, accel0, DT);
    }
}

/*
 * Rotation-only about a body axis. Attitude and gravity-in-body are kept
 * consistent at every sample so the only residual a_lin is filter lag.
 */
static void run_rotation(imu_integrate_state_t *s, int axis, float angle_rad, float T)
{
    float q_truth[4] = {1.f, 0.f, 0.f, 0.f};
    float w = angle_rad / T;
    int n = (int)(T / DT);
    for (int i = 0; i < n; i++) {
        float gyro[3] = {0.f, 0.f, 0.f};
        gyro[axis] = w;
        /* Advance truth first, then sample accel at the new attitude. */
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
    /* Hold final attitude still. */
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

    /* --- Stationary drift --- */
    calib_and_settle(&s);
    float gyro0[3] = {s.gyro_bias[0], s.gyro_bias[1], s.gyro_bias[2]};
    float accel0[3] = {0.f, 0.f, G};
    for (int i = 0; i < (int)(30.0f / DT); i++) {
        imu_integrate_step(&s, gyro0, accel0, DT);
    }
    float drift = sqrtf(s.p[0]*s.p[0] + s.p[1]*s.p[1] + s.p[2]*s.p[2]);
    printf("stationary drift = %.5f m\n", drift);
    if (drift > 0.002f) fail("stationary drift > 2 mm");

    /* --- Per-direction translation --- */
    /* Useful-signal targets: recover at least 40% of truth, correct sign,
     * and stay below 150% (no runaway). */
    const float distances[] = {0.030f, 0.050f, 0.080f};
    const float durations[] = {0.40f, 0.60f, 0.80f};
    const char *axis_name[] = {"+X", "+Y", "+Z"};
    float best_mag = 0.f;
    float worst_frac = 1.f;

    for (int axis = 0; axis < 3; axis++) {
        for (int sign = -1; sign <= 1; sign += 2) {
            for (size_t k = 0; k < sizeof(distances) / sizeof(distances[0]); k++) {
                float d = sign * distances[k];
                float T = durations[k];
                calib_and_settle(&s);
                run_translation(&s, axis, d, T);
                float p_axis = s.p[axis];
                float mag = fabsf(p_axis);
                float truth = fabsf(d);
                float frac = mag / truth;
                printf("trans %s%s d=%+.3f T=%.2f → p=[%+.4f %+.4f %+.4f] "
                       "axis=%+.4f (%.0f%%)\n",
                       sign > 0 ? "+" : "-", axis_name[axis] + 1,
                       d, T, s.p[0], s.p[1], s.p[2], p_axis, 100.f * frac);

                if ((p_axis > 0.f) != (d > 0.f) && mag > 0.002f) {
                    char buf[128];
                    snprintf(buf, sizeof(buf),
                             "wrong sign on axis %d (got %+.4f, want sign of %+.3f)",
                             axis, p_axis, d);
                    fail(buf);
                }
                if (frac < 0.40f) {
                    char buf[128];
                    snprintf(buf, sizeof(buf),
                             "recovered only %.0f%% of %.0f mm on axis %d",
                             100.f * frac, 1000.f * truth, axis);
                    fail(buf);
                }
                if (frac > 1.50f) {
                    char buf[128];
                    snprintf(buf, sizeof(buf),
                             "overshot to %.0f%% of %.0f mm on axis %d",
                             100.f * frac, 1000.f * truth, axis);
                    fail(buf);
                }
                if (mag > best_mag) best_mag = mag;
                if (frac < worst_frac) worst_frac = frac;
            }
        }
    }

    /* --- Rotation-only phantom --- */
    /* Phantom must stay well below the useful translation signal. */
    const float angles_deg[] = {20.f, 30.f, 45.f};
    float worst_phantom = 0.f;
    for (int axis = 0; axis < 3; axis++) {
        for (size_t k = 0; k < sizeof(angles_deg) / sizeof(angles_deg[0]); k++) {
            float ang = angles_deg[k] * (float)M_PI / 180.f;
            float T = 0.50f;
            calib_and_settle(&s);
            run_rotation(&s, axis, ang, T);
            float mag = sqrtf(s.p[0]*s.p[0] + s.p[1]*s.p[1] + s.p[2]*s.p[2]);
            printf("rot-only axis=%d %.0fdeg → phantom=%.4f m  p=[%+.4f %+.4f %+.4f]\n",
                   axis, angles_deg[k], mag, s.p[0], s.p[1], s.p[2]);
            if (mag > worst_phantom) worst_phantom = mag;
            /* Absolute cap: 8 mm; also must be < 25% of best recovered translation. */
            if (mag > 0.008f) {
                char buf[128];
                snprintf(buf, sizeof(buf),
                         "phantom %.1f mm > 8 mm for %.0fdeg about axis %d",
                         1000.f * mag, angles_deg[k], axis);
                fail(buf);
            }
        }
    }

    printf("\n--- summary ---\n");
    printf("best recovered translation = %.1f mm\n", 1000.f * best_mag);
    printf("worst recovery fraction    = %.0f%%\n", 100.f * worst_frac);
    printf("worst rotation phantom     = %.1f mm\n", 1000.f * worst_phantom);
    if (best_mag > 1e-6f && worst_phantom > 0.25f * best_mag) {
        char buf[128];
        snprintf(buf, sizeof(buf),
                 "phantom (%.1f mm) exceeds 25%% of best translation (%.1f mm)",
                 1000.f * worst_phantom, 1000.f * best_mag);
        fail(buf);
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
