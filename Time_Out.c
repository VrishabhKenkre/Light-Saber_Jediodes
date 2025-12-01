// Time_Out.c  (TIM10-based timebase)
#include "Time_Out.h"
#include "hardware_stm_timer10.h"

// Global millisecond tick counter
static volatile uint32_t current_ticks = 0;

/**
 * Ticks_Init
 *  - freq_hz is the TIM10 input clock frequency (Hz)
 *  - We configure TIM10 to generate an update interrupt every 1 ms.
 *
 *  tick_freq = freq_hz / (PSC + 1) / (ARR + 1)
 *  We choose: make timer run at 1 MHz, then ARR = 1000 - 1 → 1 ms
 */
void Ticks_Init(uint32_t freq_hz)
{
    // Make timer clock ~1 MHz
    uint16_t psc = (uint16_t)((freq_hz / 1000000U) - 1U);   // e.g. 180 MHz → psc = 179
    uint16_t arr = 1000U - 1U;                              // 1000 us = 1 ms

    // Configure TIM10 to generate update interrupts at 1 kHz
    init_timer10ToInterrupt(psc, arr);
}

/**
 * Called from TIM1_UP_TIM10_IRQHandler once per update event.
 * Just increments the free-running millisecond counter.
 */
void TimeOut_OnTick(void)
{
    current_ticks++;
}

uint32_t get_Ticks(void)
{
    return current_ticks;
}

void delay(uint32_t delay_ms)
{
    uint32_t start = get_Ticks();
    while ((get_Ticks() - start) < delay_ms) {
        // Busy-wait
    }
}
