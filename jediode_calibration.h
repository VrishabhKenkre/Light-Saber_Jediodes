#ifndef JEDIODE_CALIBRATION_H
#define JEDIODE_CALIBRATION_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t is_gyro_calibrated;
    uint8_t is_accel_calibrated;

    float gyro_bias[3];
    float accel_bias[3];

    uint16_t gyro_samples;
    uint16_t accel_samples;

    float gx_sum, gy_sum, gz_sum;
    float ax_sum, ay_sum, az_sum;
} IMU_CalibrationState;

void imu_calibration_init(void);
void imu_calibration_process(int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz);
uint8_t imu_calibration_is_done(void);
void imu_apply_calibration(float *gx, float *gy, float *gz,
                           float *ax, float *ay, float *az);


#ifdef __cplusplus
}
#endif
#endif