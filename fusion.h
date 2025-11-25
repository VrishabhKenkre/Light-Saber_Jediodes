#ifndef FUSION_H_
#define FUSION_H_

#include <stdint.h>
#include "mpu6050.h"   // for ScaledData_Def

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float roll;   // degrees
    float pitch;  // degrees
    float yaw;    // degrees (gyro-integrated only, no accel correction)
} Attitude_Def;

/**
 * @brief Initialize attitude structure.
 */
static inline void Attitude_Init(Attitude_Def *att,
                                 float roll_deg,
                                 float pitch_deg,
                                 float yaw_deg)
{
    if (att == 0) return;
    att->roll  = roll_deg;
    att->pitch = pitch_deg;
    att->yaw   = yaw_deg;
}

/**
 * @brief Complementary filter update: fuse accel + gyro into roll/pitch (deg).
 *
 * @param acc   Acceleration in g (ScaledData_Def from MPU6050_Get_Accel_Scale)
 * @param gyro  Gyro rates in deg/s (ScaledData_Def from MPU6050_Get_Gyro_Scale)
 * @param dt    Time step in seconds
 * @param att   In/out attitude (roll,pitch,yaw) in degrees
 */
void Attitude_Update(const ScaledData_Def *acc,
                     const ScaledData_Def *gyro,
                     float dt,
                     Attitude_Def *att);

/**
 * @brief Remove gravity from accelerometer measurement using attitude estimate.
 *
 * @param acc_meas  Measured accel in g (includes gravity)
 * @param att       Attitude (roll,pitch in deg)
 * @param acc_lin   Output: linear accel in g (gravity-compensated)
 */
void Remove_Gravity(const ScaledData_Def *acc_meas,
                    const Attitude_Def *att,
                    ScaledData_Def *acc_lin);


#ifdef __cplusplus
}
#endif

#endif /* FUSION_H_ */
