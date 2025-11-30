#ifndef SWING_DETECTOR_H
#define SWING_DETECTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SWING_NONE = 0,
    SWING_LEFT_TO_RIGHT,
    SWING_RIGHT_TO_LEFT,
    SWING_UP,
    SWING_DOWN
} SwingDirection;

typedef struct {
    uint8_t         swing_detected;   // 0 = no swing in this window, 1 = swing
    SwingDirection direction;       // only valid if swing_detected == 1
} SwingResult;

/* Call once at startup */
void SwingDetector_Init(void);

void SwingDetector_FeedSample(float gx_dps,
                              float gy_dps,
                              float gz_dps,
                              uint32_t timestamp_ms);

/*
 * Analyze samples between [t_start_ms, t_end_ms] and decide if a swing happened.
 * Assumes axes: X = forward, Y = left, Z = up.
 */
SwingResult SwingDetector_AnalyzeWindow(uint32_t t_start_ms,
                                          uint32_t t_end_ms);

void SwingDetector_BeginWindow(void);

void SwingDetector_Update(float gx_dps, float gy_dps, float gz_dps,
                          float ax_lin, float ay_lin, float az_lin,
                          float dt_s);

SwingResult SwingDetector_EndWindow(void);


#ifdef __cplusplus
}
#endif
#endif /* SWING_DETECTOR_H */

