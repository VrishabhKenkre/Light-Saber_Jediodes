#include "main.h"
#include "debug_mort.h"
#include "hardware_stm_gpio.h"
#include "hardware_stm_timer3.h"
#include "hardware_stm_interruptcontroller.h"

#include "i2c.h"
#include "stdio.h"
#include "mpu6050.h"

extern "C" {
#include "uart.h"
#include "Time_Out.h"
#include "fusion.h"
}

//hellos

int main(void)
{
    USART2_Init();          // actually sets up USART3 -> ST-LINK VCP
    Ticks_Init(180000000);  // core clock ~180 MHz (for Nucleo F446)

    my_i2c_init();


    MPU_ConfigTypeDef mpu_cfg;

    mpu_cfg.ClockSource      = Internal_8MHz;     // or whichever you want
    mpu_cfg.Gyro_Full_Scale  = FS_SEL_250;
    mpu_cfg.Accel_Full_Scale = AFS_SEL_2g;
    mpu_cfg.CONFIG_DLPF      = DLPF_94A_98G_Hz;
    mpu_cfg.Sleep_Mode_Bit   = 0;

    MPU6050_Config(&mpu_cfg);


    UART_Write_String("Boot OK\r\n");

    Attitude_Def att;
    Attitude_Init(&att, 0.0f, 0.0f, 0.0f);

    uint32_t last_ticks = get_Ticks();
    char buf[128];

    while (1) {
        uint32_t now = get_Ticks();
        float dt = (now - last_ticks) / 1000.0f; // ms -> s
        last_ticks = now;
        if (dt <= 0.0f) dt = 0.001f;

        ScaledData_Def acc, gyro, acc_lin;

        MPU6050_Get_Accel_Scale(&acc);   // also fills GyroRW
        MPU6050_Get_Gyro_Scale(&gyro);   // uses GyroRW

        Attitude_Update(&acc, &gyro, dt, &att);
        Remove_Gravity(&acc, &att, &acc_lin);

        sprintf(buf,
        "R=%.2f P=%.2f | Ax_lin=%.3f Ay_lin=%.3f Az_lin=%.3f\r\n",
        att.roll, att.pitch,
        acc_lin.x, acc_lin.y, acc_lin.z);
        UART_Write_String(buf);

        delay(50);
    }
}