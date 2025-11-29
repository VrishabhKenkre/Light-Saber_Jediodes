#ifndef JEDIODE_DMP_TYPES_H
#define JEDIODE_DMP_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float qw, qx, qy, qz;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint32_t last_update_ms;
} MPU6050_Data;

#ifdef __cplusplus
}
#endif
#endif