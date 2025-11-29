#include <string.h>
#include <stdint.h>
#include "jediode_calibration.h"

static IMU_CalibrationState cal;

void imu_calibration_init(void)
{
    memset(&cal, 0, sizeof(cal));
}

void imu_calibration_process(int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz)
{
    if (!cal.is_gyro_calibrated) {
        cal.gx_sum += gx;
        cal.gy_sum += gy;
        cal.gz_sum += gz;
        cal.gyro_samples++;

        if (cal.gyro_samples > 500) {
            cal.gyro_bias[0] = cal.gx_sum / cal.gyro_samples;
            cal.gyro_bias[1] = cal.gy_sum / cal.gyro_samples;
            cal.gyro_bias[2] = cal.gz_sum / cal.gyro_samples;
            cal.is_gyro_calibrated = 1;
        }
    }

    if (!cal.is_accel_calibrated) {
        cal.ax_sum += ax;
        cal.ay_sum += ay;
        cal.az_sum += az;
        cal.accel_samples++;

        if (cal.accel_samples > 500) {
            cal.accel_bias[0] = cal.ax_sum / cal.accel_samples;
            cal.accel_bias[1] = cal.ay_sum / cal.accel_samples;
            cal.accel_bias[2] = (cal.az_sum / cal.accel_samples) - 16384.0f;
            cal.is_accel_calibrated = 1;
        }
    }
}

uint8_t imu_calibration_is_done(void)
{
    return cal.is_accel_calibrated && cal.is_gyro_calibrated;
}

void imu_apply_calibration(float *gx, float *gy, float *gz,
                           float *ax, float *ay, float *az)
{
    *gx -= cal.gyro_bias[0];
    *gy -= cal.gyro_bias[1];
    *gz -= cal.gyro_bias[2];

    *ax -= cal.accel_bias[0];
    *ay -= cal.accel_bias[1];
    *az -= cal.accel_bias[2];
}