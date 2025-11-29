#include "main.h"
#include "debug_mort.h"
#include "hardware_stm_gpio.h"
#include "hardware_stm_timer3.h"
#include "hardware_stm_interruptcontroller.h"

#include "i2c.h"
#include "stdio.h"
#include "mpu6050.h"

#include "uart.h"
#include "Time_Out.h"
#include "fusion.h"



//hellos

int main(void)
{
    USART2_Init();
    Ticks_Init(180000000);

    //my_i2c_init();           // Your old I2C init
    i2c1_init();
    MPU6050_DMA_Init();      // Our DMA extension

    // Configure IMU using your existing driver
    MPU_ConfigTypeDef cfg = {
        .ClockSource = Internal_8MHz,
        .Gyro_Full_Scale = FS_SEL_250,
        .Accel_Full_Scale = AFS_SEL_2g,
        .CONFIG_DLPF = DLPF_94A_98G_Hz,
        .Sleep_Mode_Bit = 0
    };
    MPU6050_Config(&cfg);

    // Register DMA callback
    i2c1_dma_setCallback(MPU6050_DMA_DoneHandler);

    UART_Write_String("MPU6050 DMA READY\n");

    MPU6050_Frame_t frame;

    while (1)
    {
        // Trigger a DMA read periodically
        MPU6050_DMA_RequestRead();
        // If new data arrived
        if (MPU6050_DMA_GetLatest(&frame))
        {
            char buf[128];
            sprintf(buf,
                "ax=%d ay=%d az=%d | gx=%d gy=%d gz=%d | t=%u\n",
                frame.ax, frame.ay, frame.az,
                frame.gx, frame.gy, frame.gz,
                frame.timestamp
            );
            UART_Write_String(buf);
        }

        delay(5); // Trigger rate ~200 Hz (or use TIMx)
    }
}