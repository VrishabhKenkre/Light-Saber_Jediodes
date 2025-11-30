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
#include "swing_detector.h"   

//hellos

/* raw -> deg/s for ±250 dps full-scale */
#define GYRO_SCALE_250DPS   (250.0f / 32768.0f)
/* length of one detection window in ms */
#define WINDOW_MS           400U

int main(void)
{
    USART2_Init();
    Ticks_Init(180000000);

    //my_i2c_init();           // Your old I2C init
    i2c1_init();
    MPU6050_DMA_Init();      // Our DMA extension

    // Configure IMU using your existing driver
    MPU_ConfigTypeDef cfg = {
        .ClockSource      = Internal_8MHz,
        .Gyro_Full_Scale  = FS_SEL_250,
        .Accel_Full_Scale = AFS_SEL_2g,
        .CONFIG_DLPF      = DLPF_94A_98G_Hz,
        .Sleep_Mode_Bit   = 0
    };
    MPU6050_Config(&cfg);

    // Register DMA callback
    i2c1_dma_setCallback(MPU6050_DMA_DoneHandler);

    // Init swing detector
    SwingDetector_Init();
    SwingDetector_BeginWindow();

    UART_Write_String("MPU6050 DMA + Swing detector READY\r\n");

    MPU6050_Frame_t frame;

    uint32_t lastTs        = 0;
    uint8_t  haveLast      = 0;
    uint32_t windowStartMs = get_Ticks();

    while (1)
    {
        // Trigger a DMA read periodically
        MPU6050_DMA_RequestRead();

        // If new data arrived
        if (MPU6050_DMA_GetLatest(&frame))
        {
            uint32_t ts   = frame.timestamp;  // set in MPU6050_DMA_DoneHandler()
            float    dt_s = 0.0f;

            if (haveLast) {
                uint32_t dt_ms = ts - lastTs;   // SysTick is 1 ms
                dt_s = (float)dt_ms * 0.001f;
            } else {
                haveLast = 1;
            }
            lastTs = ts;

            // Convert raw gyro to deg/s
            float gx_dps = (float)frame.gx * GYRO_SCALE_250DPS;
            float gy_dps = (float)frame.gy * GYRO_SCALE_250DPS;
            float gz_dps = (float)frame.gz * GYRO_SCALE_250DPS;

            // For now we’re not using linear acceleration in the swing detector
            float ax_lin = 0.0f;
            float ay_lin = 0.0f;
            float az_lin = 0.0f;

            // Feed sample into swing window (only if we have a valid dt)
            if (dt_s > 0.0f) {
                SwingDetector_Update(gx_dps, gy_dps, gz_dps,
                                     ax_lin, ay_lin, az_lin,
                                     dt_s);
            }

            // Every WINDOW_MS, classify the last window as swing / no swing
            if ((ts - windowStartMs) >= WINDOW_MS)
            {
                SwingResult sr = SwingDetector_EndWindow();

                char buf[128];

                if (!sr.swing_detected) {
                    sprintf(buf,
                            "[%lu ms] NO SWING\r\n",
                            (unsigned long)ts);
                } else {
                    const char *dirStr = "UNKNOWN";
                    switch (sr.direction) {
                        case SWING_LEFT_TO_RIGHT:  dirStr = "LEFT_TO_RIGHT";  break;
                        case SWING_RIGHT_TO_LEFT:  dirStr = "RIGHT_TO_LEFT";  break;
                        case SWING_UP:             dirStr = "UP";             break;
                        case SWING_DOWN:           dirStr = "DOWN";           break;
                        default:                   dirStr = "UNKNOWN";        break;
                    }

                    sprintf(buf,
                            "[%lu ms] SWING: %s\r\n",
                            (unsigned long)ts, dirStr);
                }

                UART_Write_String(buf);

                // Start a new window
                SwingDetector_BeginWindow();
                windowStartMs = ts;
            }
        }

        delay(5); // Trigger rate ~200 Hz (or use TIMx)
    }
}
