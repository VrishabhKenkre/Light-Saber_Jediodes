#ifndef SWING_DETECTOR_H
#define SWING_DETECTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Directions for your game */
typedef enum {
    SWING_NONE = 0,
    SWING_LEFT_TO_RIGHT,
    SWING_RIGHT_TO_LEFT,
    SWING_UP,
    SWING_DOWN
} SwingDirection;

/* Result for one analysis window */
typedef struct {
    uint8_t        swing_detected;   /* 0 = no, 1 = yes */
    SwingDirection direction;
} SwingResult;

/* Logged swing event (for state machine later) */
typedef struct {
    uint32_t       t_ms;   /* timestamp from get_Ticks() */
    SwingDirection dir;
} SwingEvent;

/* --------- Swing detector API --------- */

/* Initialize internal state + event buffer */
void SwingDetector_Init(void);

/* Start a new analysis window (reset integrals, flags, etc.) */
void SwingDetector_BeginWindow(void);

/* Feed one sample into the current window.
 * gx_dps, gy_dps, gz_dps = gyro in deg/s
 * ax_lin, ay_lin, az_lin = linear accel (currently unused, pass 0)
 * dt_s = time step in seconds since previous sample
 */
void SwingDetector_Update(float gx_dps, float gy_dps, float gz_dps,
                          float ax_lin, float ay_lin, float az_lin,
                          float dt_s);

/* End the window, classify swing & direction */
SwingResult SwingDetector_EndWindow(void);

/* --------- Event ring buffer API --------- */

/* Clear logged swing events */
void SwingEventBuffer_Init(void);

/* Push a swing event (called from main when swing_detected=1) */
void SwingEventBuffer_Push(uint32_t t_ms, SwingDirection dir);

/* Pop oldest event, returns 1 if one was returned, 0 if none */
uint8_t SwingEventBuffer_Pop(SwingEvent *out);

#ifdef __cplusplus
}
#endif

#endif /* SWING_DETECTOR_H */



