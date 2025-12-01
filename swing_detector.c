#include "swing_detector.h"
#include <math.h>

#define SWING_MIN_PEAK_DPS      80.0f   /* minimum peak angular speed */
#define SWING_MIN_ANGLE_DEG     25.0f   /* minimum total rotation angle */

/* If |omega| ever exceeds this, we consider it a real "swing-ish" motion */
#define SWING_START_THRESHOLD_DPS  30.0f


/* Integrated angles (deg) over the window */
static float s_int_gx = 0.0f;
static float s_int_gy = 0.0f;
static float s_int_gz = 0.0f;

/* Max angular speed in the window */
static float s_max_omega = 0.0f;

/* Did we ever exceed SWING_START_THRESHOLD_DPS in this window? */
static uint8_t s_seenBigOmega = 0;

/* Is a window currently active? */
static uint8_t s_windowActive = 0;

//Swing ring Buffer

#define SWING_EVENT_BUF_SIZE  16

static SwingEvent eventBuf[SWING_EVENT_BUF_SIZE];
static uint8_t    eventHead  = 0;
static uint8_t    eventCount = 0;

void SwingEventBuffer_Init(void)
{
    eventHead  = 0;
    eventCount = 0;
}

void SwingEventBuffer_Push(uint32_t t_ms, SwingDirection dir)
{
    uint8_t idx = (uint8_t)((eventHead + eventCount) % SWING_EVENT_BUF_SIZE);

    eventBuf[idx].t_ms = t_ms;
    eventBuf[idx].dir  = dir;

    if (eventCount < SWING_EVENT_BUF_SIZE) {
        eventCount++;
    } else {
        /* buffer full → drop oldest, move head forward */
        eventHead = (uint8_t)((eventHead + 1U) % SWING_EVENT_BUF_SIZE);
    }
}

uint8_t SwingEventBuffer_Pop(SwingEvent *out)
{
    if (eventCount == 0) {
        return 0;
    }

    *out = eventBuf[eventHead];
    eventHead = (uint8_t)((eventHead + 1U) % SWING_EVENT_BUF_SIZE);
    eventCount--;

    return 1;
}


void SwingDetector_Init(void)
{
    /* Reset window state */
    s_int_gx       = 0.0f;
    s_int_gy       = 0.0f;
    s_int_gz       = 0.0f;
    s_max_omega    = 0.0f;
    s_seenBigOmega = 0;
    s_windowActive = 0;

    /* Also clear the swing event buffer */
    SwingEventBuffer_Init();
}

/* Start a new integration window */
void SwingDetector_BeginWindow(void)
{
    s_int_gx       = 0.0f;
    s_int_gy       = 0.0f;
    s_int_gz       = 0.0f;
    s_max_omega    = 0.0f;
    s_seenBigOmega = 0;
    s_windowActive = 1;
}

/* Feed one sample into the current window */
void SwingDetector_Update(float gx_dps, float gy_dps, float gz_dps,
                          float ax_lin, float ay_lin, float az_lin,
                          float dt_s)
{
    (void)ax_lin;
    (void)ay_lin;
    (void)az_lin;

    if (!s_windowActive) {
        return;
    }

    if (dt_s <= 0.0f) {
        return;
    }

    /* Integrate gyro angles (deg) over the window */
    s_int_gx += gx_dps * dt_s;
    s_int_gy += gy_dps * dt_s;
    s_int_gz += gz_dps * dt_s;

    /* Magnitude of angular velocity */
    float omega = sqrtf(gx_dps * gx_dps +
                        gy_dps * gy_dps +
                        gz_dps * gz_dps);

    if (omega > s_max_omega) {
        s_max_omega = omega;
    }
    if (omega > SWING_START_THRESHOLD_DPS) {
        s_seenBigOmega = 1;
    }
}

/* End window, decide if a swing happened and which direction */
SwingResult SwingDetector_EndWindow(void)
{
    SwingResult res;
    res.swing_detected = 0;
    res.direction      = SWING_NONE;

    if (!s_windowActive) {
        return res;
    }

    s_windowActive = 0;

    /* Reject if we never saw strong motion */
    if (!s_seenBigOmega || s_max_omega < SWING_MIN_PEAK_DPS) {
        return res;
    }

    /* Total rotation on each axis (absolute angle in deg) */
    float ax = fabsf(s_int_gx);
    float ay = fabsf(s_int_gy);
    float az = fabsf(s_int_gz);

    float maxAngle = ax;
    char  axis     = 'x';

    if (ay > maxAngle) {
        maxAngle = ay;
        axis = 'y';
    }
    if (az > maxAngle) {
        maxAngle = az;
        axis = 'z';
    }

    /* Not enough total rotation → no swing */
    if (maxAngle < SWING_MIN_ANGLE_DEG) {
        return res;
    }

    /* Map axis + sign to direction.
     *
     * I assume:
     *   X axis: LEFT <-> RIGHT
     *   Y axis: UP <-> DOWN
     */

    switch (axis) {
    /* Now: X axis = UP / DOWN */
    case 'x':
        if (s_int_gx > 0.0f) {
            res.direction = SWING_UP;
        } else {
            res.direction = SWING_DOWN;
        }
        res.swing_detected = 1;
        break;

    /* Now: Y axis = LEFT / RIGHT */
    case 'y':
        if (s_int_gy > 0.0f) {
            res.direction = SWING_LEFT_TO_RIGHT;
        } else {
            res.direction = SWING_RIGHT_TO_LEFT;
        }
        res.swing_detected = 1;
        break;

    case 'z':
    default:
        res.swing_detected = 0;
        res.direction      = SWING_NONE;
        break;
    }
    return res;
}