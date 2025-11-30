#include "swing_detector.h"
#include <math.h>

/* TUNING CONSTANTS
/* How many recent samples we keep (ring buffer).
    At 100 Hz, 256 samples ≈ 2.56 seconds of history.
 */
#define SWING_BUFFER_SIZE          256

/* Thresholds (tune with real data) */
#define SWING_START_THRESHOLD_DPS  50.0f   /* “this looks like a swing” */
#define SWING_MIN_PEAK_DPS         50.0f   /* minimum peak speed in window */
#define SWING_MIN_ANGLE_DEG         40.0f   /* minimum total rotation on axis */

/* INTERNAL TYPES & STATE */

typedef struct {
    uint32_t ts_ms;
    float    gx_dps;
    float    gy_dps;
    float    gz_dps;
    uint8_t  valid;
} GyroSample_t;

static GyroSample_t g_buf[SWING_BUFFER_SIZE];
static uint16_t     g_head      = 0;  /* next write index */
static uint16_t     g_count     = 0;  /* number of valid samples */

static float s_int_gx_deg = 0.0f;
static float s_int_gy_deg = 0.0f;
static float s_int_gz_deg = 0.0f;
static float s_max_omega  = 0.0f;
static uint8_t s_seenBigOmega = 0;

void SwingDetector_Init(void)
{
    for (uint16_t i = 0; i < SWING_BUFFER_SIZE; i++) {
        g_buf[i].valid = 0;
    }
    g_head  = 0;
    g_count = 0;
}

void SwingDetector_FeedSample(float gx_dps,
                              float gy_dps,
                              float gz_dps,
                              uint32_t timestamp_ms)
{
    GyroSample_t *s = &g_buf[g_head];

    s->ts_ms  = timestamp_ms;
    s->gx_dps = gx_dps;
    s->gy_dps = gy_dps;
    s->gz_dps = gz_dps;
    s->valid  = 1;

    g_head = (uint16_t)((g_head + 1U) % SWING_BUFFER_SIZE);

    if (g_count < SWING_BUFFER_SIZE) {
        g_count++;
    }
}

SwingResult SwingDetector_AnalyzeWindow(uint32_t t_start_ms,
                                          uint32_t t_end_ms)
{
    SwingResult res;
    res.swing_detected = 0;
    res.direction      = SWING_NONE;

    if (t_end_ms <= t_start_ms) {
        return res;  /* invalid window */
    }
    if (g_count == 0) {
        return res;  /* no data */
    }

    /* Start from oldest sample in ring buffer */
    uint16_t oldest_index = (uint16_t)((g_head + SWING_BUFFER_SIZE - g_count) %
                                       SWING_BUFFER_SIZE);

    float   int_gx_deg    = 0.0f;
    float   int_gy_deg    = 0.0f;
    float   int_gz_deg    = 0.0f;
    float   max_omega     = 0.0f;
    uint8_t seenBigOmega  = 0;

    uint8_t  have_prev    = 0;
    uint32_t prev_ts      = 0;
    float    prev_gx      = 0.0f;
    float    prev_gy      = 0.0f;
    float    prev_gz      = 0.0f;

    uint16_t idx = oldest_index;

    for (uint16_t n = 0; n < g_count; n++) {

        GyroSample_t *s = &g_buf[idx];

        if (s->valid) {
            uint32_t ts = s->ts_ms;

            /* Only look at samples inside [t_start, t_end] */
            if (ts >= t_start_ms && ts <= t_end_ms) {

                float gx = s->gx_dps;
                float gy = s->gy_dps;
                float gz = s->gz_dps;

                if (!have_prev) {
                    prev_ts = ts;
                    prev_gx = gx;
                    prev_gy = gy;
                    prev_gz = gz;
                    have_prev = 1;
                } else {
                    uint32_t dt_ms = ts - prev_ts;
                    prev_ts = ts;

                    float dt_s = (float)dt_ms * 0.001f;
                    if (dt_s > 0.0f) {
                        /* simple integration using current sample */
                        int_gx_deg += gx * dt_s;
                        int_gy_deg += gy * dt_s;
                        int_gz_deg += gz * dt_s;
                    }
                }

                /* Magnitude of angular velocity at this sample */
                float omega = sqrtf(gx*gx + gy*gy + gz*gz);

                if (omega > max_omega) {
                    max_omega = omega;
                }
                if (omega > SWING_START_THRESHOLD_DPS) {
                    seenBigOmega = 1;
                }
            }
        }

        idx = (uint16_t)((idx + 1U) % SWING_BUFFER_SIZE);
    }

    /* No samples inside window → no swing */
    if (!have_prev) {
        return res;
    }

    /* Reject if we never saw strong motion */
    if (!seenBigOmega || max_omega < SWING_MIN_PEAK_DPS) {
        return res;
    }

    /* Total rotation on each axis */
    float ax = fabsf(int_gx_deg);
    float ay = fabsf(int_gy_deg);
    float az = fabsf(int_gz_deg);

    float maxAngle = ax;
    if (ay > maxAngle) maxAngle = ay;
    if (az > maxAngle) maxAngle = az;

    if (maxAngle < SWING_MIN_ANGLE_DEG) {
        /* moved, but not enough to count as a real swing */
        return res;
    }

    /* Decide dominant axis and map to your 4 directions.
     * Axes assumption: X forward, Y left, Z up.
     * If Z dominates (twist), we treat as no valid swing.
     */
    if (ax >= ay && ax >= az) {
        /* Dominant rotation around X axis → horizontal swing */
        res.swing_detected = 1;
        res.direction = (int_gx_deg > 0.0f)
                        ? SWING_RIGHT_TO_LEFT   /* or flip if your sign convention is opposite */
                        : SWING_LEFT_TO_RIGHT;
    }
    else if (ay >= ax && ay >= az) {
        /* Dominant rotation around Y axis → vertical swing */
        res.swing_detected = 1;
        res.direction = (int_gy_deg > 0.0f)
                        ? SWING_UP
                        : SWING_DOWN;
    }
    else {
        /* Dominant rotation around Z axis = twist/stab: */
        return res;
    }

    return res;
}

void SwingDetector_BeginWindow(void)
{
    s_int_gx_deg   = 0.0f;
    s_int_gy_deg   = 0.0f;
    s_int_gz_deg   = 0.0f;
    s_max_omega    = 0.0f;
    s_seenBigOmega = 0;
}

/* Feed one sample into the current window.
 * gx_dps, gy_dps, gz_dps: gyro in deg/s
 * ax_lin, ay_lin, az_lin: linear accel (currently unused)
 * dt_s:                time since last sample in seconds
 */
void SwingDetector_Update(float gx_dps, float gy_dps, float gz_dps,
                          float ax_lin, float ay_lin, float az_lin,
                          float dt_s)
{
    (void)ax_lin;  /* not used yet */
    (void)ay_lin;
    (void)az_lin;

    if (dt_s <= 0.0f) {
        return;
    }

    /* Integrate rotation in degrees */
    s_int_gx_deg += gx_dps * dt_s;
    s_int_gy_deg += gy_dps * dt_s;
    s_int_gz_deg += gz_dps * dt_s;

    /* Magnitude of angular velocity at this sample */
    float omega = sqrtf(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);

    if (omega > s_max_omega) {
        s_max_omega = omega;
    }
    if (omega > SWING_START_THRESHOLD_DPS) {
        s_seenBigOmega = 1;
    }
}

/* Decide if we saw a swing in the last window.
 * Returns a SwingResult with:
 *   swing_detected = 0 → no swing
 *   swing_detected = 1 → direction is valid
 */
SwingResult SwingDetector_EndWindow(void)
{
    SwingResult res;
    res.swing_detected = 0;
    res.direction      = SWING_NONE;

    /* If we never saw strong motion, reject */
    if (!s_seenBigOmega || s_max_omega < SWING_MIN_PEAK_DPS) {
        return res;
    }

    /* Total absolute rotation on each axis */
    float ax = (s_int_gx_deg >= 0.0f) ? s_int_gx_deg : -s_int_gx_deg;
    float ay = (s_int_gy_deg >= 0.0f) ? s_int_gy_deg : -s_int_gy_deg;
    float az = (s_int_gz_deg >= 0.0f) ? s_int_gz_deg : -s_int_gz_deg;

    /* We care about:
     *   - Z rotation (yaw)  → LEFT / RIGHT swings
     *   - Y rotation (pitch)→ UP / DOWN swings
     *   - X rotation (roll) → ignore (twist of blade)
     */

    float maxAngle = ay;
    if (az > maxAngle) maxAngle = az;

    if (maxAngle < SWING_MIN_ANGLE_DEG) {
        /* moved, but not enough to count as a real swing */
        return res;
    }

    /* Decide dominant axis (Y = vertical swing, Z = horizontal swing) */
    if (az >= ay) {
        /* Dominant rotation around Z axis → LEFT/RIGHT swing */
        res.swing_detected = 1;

        /* RIGHT-HAND RULE:
         *   +Z rotation = CCW when looking from above.
         * For now we map:
         *   +Z → LEFT_TO_RIGHT
         *   -Z → RIGHT_TO_LEFT
         * If this feels reversed, just swap the two lines.
         */
        if (s_int_gz_deg > 0.0f) {
            res.direction = SWING_LEFT_TO_RIGHT;
        } else {
            res.direction = SWING_RIGHT_TO_LEFT;
        }
    }
    else {
        /* Dominant rotation around Y axis → UP/DOWN swing */
        res.swing_detected = 1;

        /* With X forward, Y left, Z up:
         *   +Y rotation is nose-up or nose-down depending on view;
         * tune by feel:
         */
        if (s_int_gy_deg > 0.0f) {
            res.direction = SWING_UP;
        } else {
            res.direction = SWING_DOWN;
        }
    }

    return res;
}
