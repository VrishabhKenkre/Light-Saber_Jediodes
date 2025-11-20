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
}



// #define rate 300
// uint32_t previous;


// int main(void)
// {
// 	Ticks_Init(16000000);

// 	USART2_Init();

// 	my_i2c_init();

// 	MPU_ConfigTypeDef myConfig;

// 	myConfig.Accel_Full_Scale = AFS_SEL_4g;
// 	myConfig.ClockSource = Internal_8MHz;
// 	myConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
// 	myConfig.Sleep_Mode_Bit = 0;
// 	myConfig.Gyro_Full_Scale = FS_SEL_500;

// 	MPU6050_Config(&myConfig);

// 	ScaledData_Def meAccel;

// 	while(1)
// 	{
// 		MPU6050_Get_Accel_Scale(&meAccel);
// 		if(get_Ticks()-previous >rate)
// 		{
// 		    printf("Accel: X = %.2f Y = %.2f Z = %.2f\r\n", meAccel.x, meAccel.y, meAccel.z);
// 		    printf("==============================\r\n");
// 			previous=get_Ticks();
// 		}
// 	}

// }

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

    // Read WHO_AM_I register (0x75, should be 0x68)
    uint8_t who = 0;
    i2c_readByte(MPU_ADDR, WHO_AM_I_REG, &who);

    UART_Write_String("WHO_AM_I = 0x");
    char buf[8];
    sprintf(buf, "%02X\r\n", who);
    UART_Write_String(buf);

    while (1) {
    ScaledData_Def accel;
    MPU6050_Get_Accel_Scale(&accel);

    char buf[64];
    sprintf(buf, "Ax=%.3f Ay=%.3f Az=%.3f\r\n",
            accel.x, accel.y, accel.z);
    UART_Write_String(buf);

    delay(200);
    }
}