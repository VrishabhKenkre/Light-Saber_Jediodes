/**
 * @file    blade_swing.c
 * @brief   Blade swing classification using MPU6050 DMP data.
 *
 * Coordinate convention (sensor frame):
 *   - z axis: along blade, hilt toward +Z, tip toward -Z
 *   - y axis: points toward user (button direction +Y)
 *   - x axis: lateral, +X to the right of the button from user POV
 */

#include "jediode_blade_swing.h"
#include <math.h>
#include <string.h>

typedef struct {
    float qw, qx, qy, qz;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint32_t last_update_ms;  // optional: timestamp from your systick
} MPU6050_Data;

/* Global DMP data coming from mpu6050.c */
extern MPU6050_Data g_mpu6050;

/* LSB per g for accel at ±2g (change if you use another FSR) */
#ifndef MPU6050_ACCEL_LSB_PER_G
#define MPU6050_ACCEL_LSB_PER_G  16384.0f
#endif

/* ========= Tunable parameters ======================================= */

/* EMA smoothing factor for linear acceleration */
#define BLADE_SWING_DEFAULT_ALPHA   0.3f

/* Minimum total linear-acceleration magnitude [g] to consider a swing */
#define BLADE_SWING_MIN_MAG_G       0.4f

/* Axis component threshold [g] to decide direction */
#define BLADE_SWING_AXIS_THRESH_G   0.5f

/* Small epsilon for normalizations */
#define BLADE_SWING_EPS             1e-3f

/* ========= Internal state =========================================== */

typedef struct {
    /* Home pose */
    float q_home[4];        /* [qw, qx, qy, qz] */
    float g_home[3];        /* gravity direction in sensor frame at home */
    float up_axis[3];       /* derived from -g_home (unit vector) */
    uint8_t has_home;

    /* Latest linear acceleration (in g, sensor frame) */
    float acc_lin[3];       /* [ax, ay, az] */
    float acc_lin_filt[3];  /* EMA filtered */

    /* Smoothing factor for EMA */
    float alpha;
} BladeSwingState;

static BladeSwingState s_state;

/* ========= Small helpers ============================================ */

/* Convert quaternion -> gravity direction in sensor frame.
 * This is the same formula used in many MPU6050 DMP examples.
 */
static void quat_to_gravity(float qw, float qx, float qy, float qz,
                            float *gx, float *gy, float *gz)
{
    /* Gravity vector in sensor coordinates (unit length, ideally) */
    *gx = 2.0f * (qx * qz - qw * qy);
    *gy = 2.0f * (qw * qx + qy * qz);
    *gz = qw * qw - qx * qx - qy * qy + qz * qz;
}

/* Normalize a 3D vector; if too small, return (0,0,1). */
static void normalize3(const float v_in[3], float v_out[3])
{
    float n2 = v_in[0]*v_in[0] + v_in[1]*v_in[1] + v_in[2]*v_in[2];
    if (n2 < BLADE_SWING_EPS * BLADE_SWING_EPS) {
        v_out[0] = 0.0f;
        v_out[1] = 0.0f;
        v_out[2] = 1.0f;
        return;
    }
    float invn = 1.0f / sqrtf(n2);
    v_out[0] = v_in[0] * invn;
    v_out[1] = v_in[1] * invn;
    v_out[2] = v_in[2] * invn;
}

/* Dot product */
static float dot3(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/* ========= Public API =============================================== */

void blade_swing_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_state.alpha = BLADE_SWING_DEFAULT_ALPHA;
}

/* Capture current quaternion + gravity as home pose */
void blade_swing_set_home_from_current(void)
{
    /* 1) Copy current quaternion */
    s_state.q_home[0] = g_mpu6050.qw;
    s_state.q_home[1] = g_mpu6050.qx;
    s_state.q_home[2] = g_mpu6050.qy;
    s_state.q_home[3] = g_mpu6050.qz;

    /* 2) Compute gravity direction in sensor frame */
    float gx, gy, gz;
    quat_to_gravity(g_mpu6050.qw,
                    g_mpu6050.qx,
                    g_mpu6050.qy,
                    g_mpu6050.qz,
                    &gx, &gy, &gz);

    s_state.g_home[0] = gx;
    s_state.g_home[1] = gy;
    s_state.g_home[2] = gz;

    /* 3) Define "up" axis as opposite of gravity at home pose */
    float up_tmp[3] = { -gx, -gy, -gz };
    normalize3(up_tmp, s_state.up_axis);

    s_state.has_home = 1;
}

/* Set EMA smoothing factor */
void blade_swing_set_smoothing(float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    s_state.alpha = alpha;
}

/* Update from latest DMP data */
void blade_swing_update(void)
{
    /* 1) Convert raw accel counts -> g's */
    float ax_g = (float)g_mpu6050.ax / MPU6050_ACCEL_LSB_PER_G;
    float ay_g = (float)g_mpu6050.ay / MPU6050_ACCEL_LSB_PER_G;
    float az_g = (float)g_mpu6050.az / MPU6050_ACCEL_LSB_PER_G;

    /* 2) Gravity from current quaternion */
    float gx, gy, gz;
    quat_to_gravity(g_mpu6050.qw,
                    g_mpu6050.qx,
                    g_mpu6050.qy,
                    g_mpu6050.qz,
                    &gx, &gy, &gz);

    /* 3) Linear acceleration in g's: a_lin = a_meas - g_vec */
    float acc_lin_now[3];
    acc_lin_now[0] = ax_g - gx;
    acc_lin_now[1] = ay_g - gy;
    acc_lin_now[2] = az_g - gz;

    /* 4) Save raw (optional) */
    s_state.acc_lin[0] = acc_lin_now[0];
    s_state.acc_lin[1] = acc_lin_now[1];
    s_state.acc_lin[2] = acc_lin_now[2];

    /* 5) EMA smoothing */
    float a = s_state.alpha;

    s_state.acc_lin_filt[0] = a * acc_lin_now[0] +
                              (1.0f - a) * s_state.acc_lin_filt[0];
    s_state.acc_lin_filt[1] = a * acc_lin_now[1] +
                              (1.0f - a) * s_state.acc_lin_filt[1];
    s_state.acc_lin_filt[2] = a * acc_lin_now[2] +
                              (1.0f - a) * s_state.acc_lin_filt[2];
}

/* Get filtered linear accel in sensor frame (g's) */
void blade_swing_get_filtered_accel(float *ax, float *ay, float *az)
{
    if (ax) *ax = s_state.acc_lin_filt[0];
    if (ay) *ay = s_state.acc_lin_filt[1];
    if (az) *az = s_state.acc_lin_filt[2];
}

/* Classify swing direction relative to home pose */
BladeSwingDirection blade_swing_classify(void)
{
    if (!s_state.has_home)
        return BLADE_SWING_NONE;

    float ax = s_state.acc_lin_filt[0];
    float ay = s_state.acc_lin_filt[1];
    float az = s_state.acc_lin_filt[2];

    /* Overall magnitude */
    float mag2 = ax*ax + ay*ay + az*az;
    if (mag2 < (BLADE_SWING_MIN_MAG_G * BLADE_SWING_MIN_MAG_G))
        return BLADE_SWING_NONE;

    float mag = sqrtf(mag2);

    /* Component along "up" axis (derived from home gravity) */
    float a_up = dot3(s_state.acc_lin_filt, s_state.up_axis);

    /* Left-right component:
     * by definition +X = right, -X = left in sensor frame.
     * We assume home pose defines which side is "right" from POV.
     */
    float a_lr = ax;

    /* Decide whether swing is mostly vertical or mostly horizontal */
    float abs_up = fabsf(a_up);
    float abs_lr = fabsf(a_lr);

    if (abs_up >= abs_lr)
    {
        /* Vertical classification: use a_up sign */
        if (abs_up < BLADE_SWING_AXIS_THRESH_G)
            return BLADE_SWING_NONE;

        if (a_up > 0.0f)
            return BLADE_SWING_UP;
        else
            return BLADE_SWING_DOWN;
    }
    else
    {
        /* Horizontal classification: use a_lr sign */
        if (abs_lr < BLADE_SWING_AXIS_THRESH_G)
            return BLADE_SWING_NONE;

        if (a_lr > 0.0f)
            return BLADE_SWING_RIGHT;
        else
            return BLADE_SWING_LEFT;
    }
}