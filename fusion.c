#include "fusion.h"
#include <math.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD2DEG (57.2957795130823208768f)
#define DEG2RAD (1.0f / RAD2DEG)

/*
 * NOTE ON AXIS CONVENTION:
 * This sensor axes are:
 *   X: forward
 *   Y: left
 *   Z: up
 *
 * And that when the board is flat on a table:
 *   acc.x ≈ 0, acc.y ≈ 0, acc.z ≈ +1g.
 *
 */

void Attitude_Update(const ScaledData_Def *acc,
                     const ScaledData_Def *gyro,
                     float dt,
                     Attitude_Def *att)
{
    if (acc == 0 || gyro == 0 || att == 0) {
        return;
    }

    if (dt <= 0.0f) {
        dt = 0.001f; // fallback ~1 ms
    }

    // 1) Gyro integration (deg/s * s = deg)
    //    Adjust mapping if your gyro axes differ.
    float roll_gyro  = att->roll  + gyro->x * dt;
    float pitch_gyro = att->pitch + gyro->y * dt;
    float yaw_gyro   = att->yaw   + gyro->z * dt;  // pure integration, no correction

    // 2) Accel-based roll/pitch (from gravity)
    //    atan2f returns radians; convert to degrees.
    //    Standard IMU formula:
    //      roll  = atan2( Ay, Az )
    //      pitch = atan2( -Ax, sqrt(Ay^2 + Az^2) )
    float acc_roll  = atan2f(acc->y, acc->z) * RAD2DEG;
    float acc_pitch = atan2f(-acc->x,
                             sqrtf(acc->y * acc->y + acc->z * acc->z)) * RAD2DEG;

    // 3) Complementary filter: trust gyro short-term, accel long-term
    //    alpha ~ 0.98 is typical for ~100–200 Hz update.
    const float alpha = 0.98f;

    att->roll  = alpha * roll_gyro  + (1.0f - alpha) * acc_roll;
    att->pitch = alpha * pitch_gyro + (1.0f - alpha) * acc_pitch;

    // Yaw has no accelerometer reference, so leave it gyro-integrated.
    att->yaw = yaw_gyro;
}

void Remove_Gravity(const ScaledData_Def *acc_meas,
                    const Attitude_Def *att,
                    ScaledData_Def *acc_lin)
{
    if (acc_meas == 0 || att == 0 || acc_lin == 0) {
        return;
    }

    float phi   = att->roll  * DEG2RAD;   // roll
    float theta = att->pitch * DEG2RAD;   // pitch

    // Gravity vector in body frame (assuming yaw ≈ 0 for simplicity).
    // g_world = [0, 0, 1g]
    // R = R_y(pitch) * R_x(roll)
    // g_body = R^T * g_world
    float sin_phi   = sinf(phi);
    float cos_phi   = cosf(phi);
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    float g_bx = -sin_theta;
    float g_by =  sin_phi * cos_theta;
    float g_bz =  cos_phi * cos_theta;

    // measured = linear + gravity  =>  linear = measured - gravity
    acc_lin->x = acc_meas->x - g_bx;
    acc_lin->y = acc_meas->y - g_by;
    acc_lin->z = acc_meas->z - g_bz;
}
