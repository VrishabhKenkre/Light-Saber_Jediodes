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
 

int main(void)
{
    USART2_Init();
    Ticks_Init(180000000);    // SysTick at 1 ms tick

    i2c1_init();              // your bare-metal I2C init
    MPU6050_DMA_Init();       // DMA extension

    // Configure IMU using your existing driver
    MPU_ConfigTypeDef cfg = {
        .ClockSource      = Internal_8MHz,
        .Gyro_Full_Scale  = FS_SEL_250,
        .Accel_Full_Scale = AFS_SEL_2g,
        .CONFIG_DLPF      = DLPF_94A_98G_Hz,
        .Sleep_Mode_Bit   = 0
    };
    MPU6050_Config(&cfg);

    // Register DMA callback that unpacks samples
    i2c1_dma_setCallback(MPU6050_DMA_DoneHandler);

    // Init swing detector (also clears its internal event buffer)
    SwingDetector_Init();

    UART_Write_String("MPU6050 DMA + Swing detector READY\r\n");

    MPU6050_Frame_t frame;

    const uint32_t WINDOW_MS = 300;      // ~0.3 s swing analysis window
    uint8_t  windowActive    = 0;
    uint32_t windowStartTs   = 0;
    uint32_t lastSampleTs    = 0;

    while (1)
    {
        // Kick a DMA read (non-blocking)
        MPU6050_DMA_RequestRead();

        // If a new frame is ready, process it
        if (MPU6050_DMA_GetLatest(&frame))
        {
            uint32_t now = frame.timestamp;  // from get_Ticks() in DMA handler

            // Compute dt in seconds between this and previous sample
            float dt_s = 0.0f;
            if (lastSampleTs != 0U) {
                uint32_t dt_ms = now - lastSampleTs;
                dt_s = (float)dt_ms * 0.001f;
            }
            lastSampleTs = now;

            // Convert raw gyro to deg/s (FS = ±250 dps)
            float gx_dps = (float)frame.gx * 250.0f / 32768.0f;
            float gy_dps = (float)frame.gy * 250.0f / 32768.0f;
            float gz_dps = (float)frame.gz * 250.0f / 32768.0f;

            // For now we don’t use linear accel in swing detection
            float ax_lin = 0.0f;
            float ay_lin = 0.0f;
            float az_lin = 0.0f;

            // --- Window management for swing detection ---

            // If no active window, start one at this timestamp
            if (!windowActive) {
                SwingDetector_BeginWindow();
                windowStartTs = now;
                windowActive  = 1;
            }

            // Feed this sample into the current window
            if (dt_s > 0.0f) {
                SwingDetector_Update(gx_dps, gy_dps, gz_dps,
                                     ax_lin, ay_lin, az_lin,
                                     dt_s);
            }

            // If window has reached its duration, classify it
            if (windowActive && (now - windowStartTs) >= WINDOW_MS) {
                SwingResult sr = SwingDetector_EndWindow();
                windowActive = 0;   // next sample will start a new window

                if (sr.swing_detected) {
                    // 1) Log swing into the event buffer (inside swing_detector.c)
                    SwingEventBuffer_Push(now, sr.direction);

                    // 2) Also print it so you can see detection on the terminal
                    char buf[80];
                    const char *dirStr = "NONE";
                    switch (sr.direction) {
                        case SWING_LEFT_TO_RIGHT:  dirStr = "LEFT_TO_RIGHT";  break;
                        case SWING_RIGHT_TO_LEFT:  dirStr = "RIGHT_TO_LEFT";  break;
                        case SWING_UP:             dirStr = "UP";             break;
                        case SWING_DOWN:           dirStr = "DOWN";           break;
                        default:                   dirStr = "NONE";           break;
                    }
                    sprintf(buf, "[%u ms] SWING: %s\r\n", (unsigned)now, dirStr);
                    UART_Write_String(buf);
                }

                // If no swing_detected → no event pushed, buffer stays unchanged
            }

            // Later, your state machine will do something like:
            // SwingEvent ev;
            // if (SwingEventBuffer_Pop(&ev)) {
            //     // use ev.t_ms and ev.dir
            // }
        }

        // Control how often you trigger DMA requests (approx. sample rate)
        delay(5);   // ~200 Hz trigger rate, adjust if needed
    }
}



