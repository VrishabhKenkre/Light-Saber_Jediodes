/**
  ******************************************************************************
  * @file    hardware_stm_timer10.c
  * @author  paussava
  * @version 1.0
  * @date    Nov-2025
  * @brief   Bare-metal TIM10 driver for STM32F446ZE
  ******************************************************************************
  */

#include "hardware_stm_timer10.h"
#include "hardware_stm_gpio.h"
#include "stm32f4xx_rcc_mort.h"
#include "stm32f4xx_mort2.h"
#include <stdint.h>
#include <stdio.h>
#include "Time_Out.h" 

/* ============================================================
 * TIM10 BASE ADDRESS (APB2)
 * ============================================================ */
#define TIM10_BASE_ADDRESS   ((uint32_t)0x40014400)

/* TIM10 Registers (Base + Offset) */
#define TIM10_CR1        (*(volatile uint32_t *)(TIM10_BASE_ADDRESS + 0x00))
#define TIM10_DIER       (*(volatile uint32_t *)(TIM10_BASE_ADDRESS + 0x0C))
#define TIM10_SR         (*(volatile uint32_t *)(TIM10_BASE_ADDRESS + 0x10))
#define TIM10_EGR        (*(volatile uint32_t *)(TIM10_BASE_ADDRESS + 0x14))
#define TIM10_CCMR1      (*(volatile uint32_t *)(TIM10_BASE_ADDRESS + 0x18))
#define TIM10_CCER       (*(volatile uint32_t *)(TIM10_BASE_ADDRESS + 0x20))
#define TIM10_CNT        (*(volatile uint32_t *)(TIM10_BASE_ADDRESS + 0x24))
#define TIM10_PSC        (*(volatile uint32_t *)(TIM10_BASE_ADDRESS + 0x28))
#define TIM10_ARR        (*(volatile uint32_t *)(TIM10_BASE_ADDRESS + 0x2C))
#define TIM10_CCR1       (*(volatile uint32_t *)(TIM10_BASE_ADDRESS + 0x34))

/* ============================================================
 * TIM10 Bit Definitions
 * ============================================================ */
#define TIM10_CR1_CEN        (1U << 0)
#define TIM10_CR1_ARPE       (1U << 7)

#define TIM10_DIER_UIE       (1U << 0)

#define TIM10_SR_UIF         (1U << 0)

#define TIM10_CCMR1_OC1PE    (1U << 3)
#define TIM10_CCMR1_OC1M_PWM1 (6U << 4)

#define TIM10_CCER_CC1E      (1U << 0)

/* ============================================================
 * NVIC BASE (same as your style)
 * ============================================================ */
#define SYSTEM_CONTROL_BASE_ADDRESS     ((uint32_t)0xE000E000)
#define NVIC_BASE_ADDRESS               (SYSTEM_CONTROL_BASE_ADDRESS + 0x100)

/* NVIC ISER Registers */
#define NVIC_ISER0     (*(volatile uint32_t *)(NVIC_BASE_ADDRESS + 0x00))
#define NVIC_ISER1     (*(volatile uint32_t *)(NVIC_BASE_ADDRESS + 0x04))
#define NVIC_ISER2     (*(volatile uint32_t *)(NVIC_BASE_ADDRESS + 0x08))

/* TIM1_UP_TIM10 = IRQ 25 → ISER0 bit 25 */
#define TIM10_INTERRUPT_BIT_ISER0   (1UL << 25)


/* ============================================================
 * TIM1 BASE ADDRESS for IRQ handler
 * ============================================================ */
#define TIM1_BASE_ADDRESS   ((uint32_t)0x40010000)

/* TIM1 Registers (Base + Offset) */
#define TIM1_SR        (*(volatile uint32_t *)(TIM1_BASE_ADDRESS + 0x10))

/* TIM1_SR Bit Definitions */
#define TIM1_SR_UIF    (1U << 0)   /* Update Interrupt Flag */

/* ============================================================
 * Utility
 * ============================================================ */
uint32_t tim10_arr(void)
{
    return TIM10_ARR;
}

void tim10_set_duty(uint32_t duty)
{
    uint32_t arr = TIM10_ARR;
    if (duty > arr) duty = arr;
    TIM10_CCR1 = duty;
}


void enable_NVIC_Timer10(void)
{
    NVIC_ISER0 = TIM10_INTERRUPT_BIT_ISER0;
}

void init_timer10ToInterrupt(uint16_t psc, uint16_t arr)
{
    /* Enable TIMER10 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
    enable_NVIC_Timer10();
    
    /* Disable counter */
    TIM10_CR1 &= ~TIM10_CR1_CEN;

    /* PSC & ARR */
    TIM10_PSC = psc;
    TIM10_ARR = arr;

    /* Clear pending flag */
    TIM10_SR &= ~TIM10_SR_UIF;

    /* Enable update interrupt */
    TIM10_DIER |= TIM10_DIER_UIE;

    /* Enable preload */
    TIM10_CR1 |= TIM10_CR1_ARPE;

    /* Start */
    TIM10_CR1 |= TIM10_CR1_CEN;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    /* TIM10 source */
    if (TIM10_SR & TIM10_SR_UIF)
    {
        // Clear update flag
        TIM10_SR &= ~TIM10_SR_UIF;

        // 1 ms tick hook – this replaces SysTick
        TimeOut_OnTick();
    }

}
