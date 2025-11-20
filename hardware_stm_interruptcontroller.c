#include "hardware_stm_interruptcontroller.h"
#include "stm32f4xx_rcc_mort.h"
#include "hardware_stm_gpio.h"
#include <stdint.h>

/* ---- NVIC base and ISER ---- */
#define SYSTEM_CONTROL_BASE_ADDRESS                     (0xE000E000U)
#define NVIC_BASE_ADDRESS                               (SYSTEM_CONTROL_BASE_ADDRESS + 0x100U)
#define NVIC_INTERRUPT_SET_ENABLE_REGISTER_0_31         (NVIC_BASE_ADDRESS + 0x00U)

#define NVIC_INTERRUPT_CLEAR_ENABLE_REGISTER_0_31       (NVIC_BASE_ADDRESS + 0x80)
#define NVIC_INTERRUPT_CLEAR_ENABLE_REGISTER_32_63      (NVIC_INTERRUPT_CLEAR_ENABLE_REGISTER_0_31 + 0x4)
#define NVIC_INTERRUPT_CLEAR_ENABLE_REGISTER_64_95      (NVIC_INTERRUPT_CLEAR_ENABLE_REGISTER_0_31 + 0x8)
#define NVIC_INTERRUPT_SET_PENDING_REGISTER_0_31        (NVIC_BASE_ADDRESS + 0x100)
#define NVIC_INTERRUPT_SET_PENDING_REGISTER_32_63       (NVIC_INTERRUPT_SET_PENDING_REGISTER_0_31 + 0x4)
#define NVIC_INTERRUPT_SET_PENDING_REGISTER_64_95       (NVIC_INTERRUPT_SET_PENDING_REGISTER_0_31 + 0x8)
#define NVIC_INTERRUPT_CLEAR_PENDING_REGISTER_0_31      (NVIC_BASE_ADDRESS + 0x180)
#define NVIC_INTERRUPT_CLEAR_PENDING_REGISTER_32_63     (NVIC_INTERRUPT_CLEAR_PENDING_REGISTER_0_31 + 0x4)
#define NVIC_INTERRUPT_CLEAR_PENDING_REGISTER_64_95     (NVIC_INTERRUPT_CLEAR_PENDING_REGISTER_0_31 + 0x8)

/* TIM3 global interrupt (IRQ 29) in ISER0 */
#define TIM3_INTERRUPT_BIT                              (0x20000000U)
#define EXTI9_5_INTERRUPT_BIT                           (0x800000)   // 1 << 23

/* ---- TIM3 base + registers ---- */
#define TIM3_BASE_ADDRESS                               ((uint32_t)0x40000400U)

/* 16-bit registers (we’ll use unsigned short* like slides) */
#define TIM3_CR1_REGISTER_1                             (TIM3_BASE_ADDRESS + 0x00U)
#define TIM3_INTERRUPT_ENABLE_REGISTER                  (TIM3_BASE_ADDRESS + 0x0CU)
#define TIM3_STATUS_REGISTER                            (TIM3_BASE_ADDRESS + 0x10U)
#define TIM3_CAPTURE_COMPARE_ENABLE_REGISTER            (TIM3_BASE_ADDRESS + 0x20U)
#define TIM3_CAPTURE_COMPARE_MODE_2_REGISTER            (TIM3_BASE_ADDRESS + 0x1CU)
#define TIM3_PRESCALER_REGISTER                         (TIM3_BASE_ADDRESS + 0x28U)
#define TIM3_AUTORELOAD_REGISTER                        (TIM3_BASE_ADDRESS + 0x2CU)
#define TIM3_COMPARE_3_REGISTER                         (TIM3_BASE_ADDRESS + 0x3CU)

/* ---- Bit masks ---- */
#define COUNTER_ENABLE_BIT                              ((uint16_t)0x0001)  /* CR1 CEN */

#define TIM_UIF                                         ((uint16_t)0x0001)  /* status: update flag */
#define TIM_CH3_CC3IF                                   ((uint16_t)0x0008)  /* status: CC3 flag    */

#define TIM_CH3_CC_INTERRUPT_ENABLE                     ((uint16_t)0x0008)  /* DIER: CC3IE bit */
#define TIM_UPDATE_INTERRUPT_ENABLE                     ((uint16_t)0x0001)  /* DIER: UIE bit   */

#define TIM3_CCER_CC3E                                  ((uint16_t)0x0100)  /* CCER: CC3E */

#define TIM_CCMR13_OCPE                                 ((uint16_t)0x0008)  /* CCMR2: OC3PE */


//Map PC6 to EXTI6 (SYSCFG_EXTICR2)
#define SYSCFG_BASE_ADDRESS                             ((uint32_t)(0x40013800))
#define SYSCFG_EXTERNAL_INTERRUPT_REGISTER_2            (SYSCFG_BASE_ADDRESS + 0x0C)
#define SYSCFG_EXTERNAL_INTERRUPT_6_BITS                ((uint32_t)0xF00)  // mask for EXTI6
#define SYSCFG_EXTERNAL_INTERRUPT_6_PORTC               ((uint32_t)0x200)  // 0010 << 8 for Port C

 /* External interrupt controller registers / bits from slides: */
#define EXTERNAL_INTERRUPT_CONTROLLER_BASE_ADDRESS      ((uint32_t)(0x40013C00))
#define EXTERNAL_INTERRUPT_CONTROLLER_MASK_REGISTER     (EXTERNAL_INTERRUPT_CONTROLLER_BASE_ADDRESS)
#define EXTERNAL_INTERRUPT_CONTROLLER_MASK_REGISTER_EXTI6 ((uint32_t)0x40)  // bit 6
#define EXTERNAL_INTERRUPT_CONTROLLER_RTSR              (EXTERNAL_INTERRUPT_CONTROLLER_BASE_ADDRESS+0x08)
#define EXTERNAL_INTERRUPT_CONTROLLER_RTSR_EXTI6        ((uint32_t)0x40)
#define EXTERNAL_INTERRUPT_CONTROLLER_FTSR              (EXTERNAL_INTERRUPT_CONTROLLER_BASE_ADDRESS+0x0C)
#define EXTERNAL_INTERRUPT_CONTROLLER_FTSR_EXTI6        ((uint32_t)0x40)
#define EXTERNAL_INTERRUPT_CONTROLLER_PENDING_REGISTER  (EXTERNAL_INTERRUPT_CONTROLLER_BASE_ADDRESS+0x14)
#define EXTERNAL_INTERRUPT_CONTROLLER_PENDING_EXTI6     ((uint32_t)0x40)   
    

/* Small helpers to clear flags */
static inline void TIM3_CLEAR_CC3IF(void)
{
    /* CCxIF clears by reading SR then reading CCRx */
    (void)TIM3->SR;
    (void)TIM3->CCR3;
}

static inline void TIM3_CLEAR_UIF(void)
{
    /* UIF clears by writing 0 to the bit in SR */
    TIM3->SR &= ~TIM_SR_UIF;
}

/* NVIC helper                                                                 */

void enableNVIC_Timer3(void)
{
    unsigned int *reg_pointer_32;
    reg_pointer_32 = (unsigned int *)NVIC_INTERRUPT_SET_ENABLE_REGISTER_0_31;
    *reg_pointer_32 |= TIM3_INTERRUPT_BIT;   // enable TIM3 IRQ in NVIC
}


/* TIM3 configuration for PB0                                                  */

void initTimer3ToInterrupt(void)
{
    unsigned short *reg_pointer_16;

    /* 1) Timer 3 APB clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 2) Call the NVIC function to enable the interrupt that would go to timer 3 */
    enableNVIC_Timer3();

    /* 3) Compute Prescale and Autorreload (for ~0.25 Hz)
     *
     *    Assume timer clock = 90 MHz.
     *    Let PSC = 9999 -> f_cnt = 90 MHz / (9999 + 1) = 9 kHz
     *    Let ARR = 36000 -> period = (36000 + 1) / 9000 ≈ 4 s → 0.25 Hz
     */
    uint16_t prescalervalue = 9999;
    uint16_t autoreloadvalue = 36000;

    /* 4) Clear any pending flags in the status register */
    reg_pointer_16 = (unsigned short *)TIM3_STATUS_REGISTER;
    *reg_pointer_16 = 0;    // write 0 to clear all flags

    /* 5) Set Prescale and Autorreload values in their corresponding registers */
    reg_pointer_16 = (unsigned short *)TIM3_PRESCALER_REGISTER;
    *reg_pointer_16 = prescalervalue;

    reg_pointer_16 = (unsigned short *)TIM3_AUTORELOAD_REGISTER;
    *reg_pointer_16 = autoreloadvalue;

    /* 6) Set the Compare Value on channel 3 of Timer 3 (e.g., 50% of period) */
    reg_pointer_16 = (unsigned short *)TIM3_COMPARE_3_REGISTER;
    *reg_pointer_16 = autoreloadvalue / 2;   // mid-period event

    /* 7) Enable Preload Register (Don’t HAVE to, but good practice) */
    reg_pointer_16 = (unsigned short *)TIM3_CAPTURE_COMPARE_MODE_2_REGISTER;
    *reg_pointer_16 = *reg_pointer_16 | TIM_CCMR13_OCPE;

    /* 8) Enable the TIM3 channel 3 counter and keep the default configuration for channel polarity */
    reg_pointer_16 = (unsigned short *)TIM3_CAPTURE_COMPARE_ENABLE_REGISTER;
    *reg_pointer_16 = *reg_pointer_16 | TIM3_CCER_CC3E;

    /* 9) Enable interrupt on capture compare channel 3 (and update for overflow) */
    reg_pointer_16 = (unsigned short *)TIM3_INTERRUPT_ENABLE_REGISTER;
    *reg_pointer_16 = (TIM_CH3_CC_INTERRUPT_ENABLE | TIM_UPDATE_INTERRUPT_ENABLE);

    /* 10) Enable timer 3 (start counter) */
    reg_pointer_16 = (unsigned short *)TIM3_CR1_REGISTER_1;
    *reg_pointer_16 = *reg_pointer_16 | COUNTER_ENABLE_BIT;
}

/* TIM3 interrupt handler  */

void TIM3_IRQHandler(void)
{
    unsigned short *sr_ptr;
    unsigned short *dier_ptr;
    unsigned short sr, dier;

    sr_ptr   = (unsigned short *)TIM3_STATUS_REGISTER;
    dier_ptr = (unsigned short *)TIM3_INTERRUPT_ENABLE_REGISTER;

    sr   = *sr_ptr;
    dier = *dier_ptr;

    /* 1) Check if Output Compare 3 triggered the interrupt */
    if ((sr & TIM_CH3_CC3IF) && (dier & TIM_CH3_CC_INTERRUPT_ENABLE))
    {
        /* Clear CC3 flag (write 0 to that bit, keep others) */
        *sr_ptr = (unsigned short)(sr & ~(TIM_CH3_CC3IF));

        /* Perform action: turn OFF LED (B0) */
        clearGPIOB0();
    }

    /* Re-read SR in case previous write changed it */
    sr = *sr_ptr;

    /* 2) Check if Overflow triggered the interrupt (UIF) */
    if ((sr & TIM_UIF) && (dier & TIM_UPDATE_INTERRUPT_ENABLE))
    {
        /* Clear UIF flag */
        *sr_ptr = (unsigned short)(sr & ~(TIM_UIF));

        /* Perform action: turn ON LED (B0) */
        setGPIOB0();
    }
}

void enableEXTI6OnPortC(void)
{
    uint32_t *reg_pointer_32;

    /* 1) Init GPIO C6 as input and B0 as LED output (slides do this too) */
    initGpioC6AsInput();
    initGpioB0AsOutput();

    /* 2) Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* 3) Map EXTI6 to Port C (SYSCFG_EXTICR2 bits [11:8]) */
    reg_pointer_32 = (uint32_t *)SYSCFG_EXTERNAL_INTERRUPT_REGISTER_2;

    /* clear EXTI6 bits */
    *reg_pointer_32 = *reg_pointer_32 & ~SYSCFG_EXTERNAL_INTERRUPT_6_BITS;

    /* set EXTI6 to Port C */
    *reg_pointer_32 = *reg_pointer_32 | SYSCFG_EXTERNAL_INTERRUPT_6_PORTC;

    /* 4) Unmask EXTI6 in EXTI_IMR (so line 6 can generate interrupts) */
    reg_pointer_32 = (uint32_t *)EXTERNAL_INTERRUPT_CONTROLLER_MASK_REGISTER;
    *reg_pointer_32 = *reg_pointer_32 | EXTERNAL_INTERRUPT_CONTROLLER_MASK_REGISTER_EXTI6;

    /* 5) Enable rising-edge trigger for EXTI6 in EXTI_RTSR */
    reg_pointer_32 = (uint32_t *)EXTERNAL_INTERRUPT_CONTROLLER_RTSR;
    *reg_pointer_32 = *reg_pointer_32 | EXTERNAL_INTERRUPT_CONTROLLER_RTSR_EXTI6;

    /* 6) Enable EXTI9_5 interrupt line in NVIC (IRQ for EXTI lines 5..9) */
    reg_pointer_32 = (uint32_t *)NVIC_INTERRUPT_SET_ENABLE_REGISTER_0_31;
    *reg_pointer_32 = EXTI9_5_INTERRUPT_BIT;
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t *reg_pointer_32;
    reg_pointer_32 = (uint32_t *)EXTERNAL_INTERRUPT_CONTROLLER_PENDING_REGISTER;

    if ((*reg_pointer_32 & EXTERNAL_INTERRUPT_CONTROLLER_PENDING_EXTI6) > 0) {
        *reg_pointer_32 = EXTERNAL_INTERRUPT_CONTROLLER_PENDING_EXTI6;
        toggleGPIOB0();
    }
}




