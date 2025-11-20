// Time_Out.c
#include "Time_Out.h"
#include "stm32f4xx.h"

static volatile uint32_t current_ticks = 0;

void Ticks_Init(uint32_t freq)
{
    // Configure SysTick for 1ms tick, assuming freq in Hz (e.g. 16000000)
    SysTick->LOAD = (freq / 1000U) - 1U;
    SysTick->VAL  = 0;

    // Internal clock, tick interrupt enable, counter enable
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
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
        // spin
    }
}
