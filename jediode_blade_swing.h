#ifndef __BLADE_SWING_H_
#define __BLADE_SWING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Direction of detected swing */
typedef enum {
    BLADE_SWING_NONE = 0,
    BLADE_SWING_UP,
    BLADE_SWING_DOWN,
    BLADE_SWING_LEFT,
    BLADE_SWING_RIGHT
} BladeSwingDirection;

/**
 * @brief Initialize internal state (filters, home pose flag, etc.).
 *        Call once at startup.
 */
void blade_swing_init(void);

/**
 * @brief Capture the current DMP pose as the "home" reference.
 *        Uses g_mpu6050_dmp quaternion + gravity to define:
 *        - home quaternion
 *        - "up" axis derived from gravity
 */
void blade_swing_set_home_from_current(void);

/**
 * @brief Set exponential smoothing factor for linear acceleration.
 *        alpha in [0,1]: 0 = infinitely slow, 1 = no smoothing.
 */
void blade_swing_set_smoothing(float alpha);

/**
 * @brief Update internal linear-acceleration estimate from global
 *        g_mpu6050_dmp.
 *
 *        Call this exactly once whenever you have parsed a new
 *        DMP packet into g_mpu6050_dmp (e.g., after mpu6050_dmp_service()).
 */
void blade_swing_update(void);

/**
 * @brief Classify the direction of the latest swing
 *        based on filtered linear acceleration and home pose.
 *
 * @return BLADE_SWING_NONE if:
 *         - no home pose set, or
 *         - motion below threshold,
 *         else one of UP/DOWN/LEFT/RIGHT.
 */
BladeSwingDirection blade_swing_classify(void);

/**
 * @brief Get the filtered linear acceleration in sensor frame (in g's).
 *        Pointers may be NULL if you only care about some axes.
 */
void blade_swing_get_filtered_accel(float *ax, float *ay, float *az);

#ifdef __cplusplus
}
#endif

#endif /* __BLADE_SWING_H_ */
