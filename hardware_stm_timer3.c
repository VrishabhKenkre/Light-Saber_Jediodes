#include "hardware_stm_timer3.h"
#include "hardware_stm_gpio.h"
#include "stm32f4xx_rcc_mort.h"
#include "stm32f4xx_mort2.h"
#include "hardware_stm_interruptcontroller.h"
#include "stm32f446xx.h"   // gives you TIM3 peripheral struct


// /* --- TIM3 register addresses (RM0390 TIM2-5 map) --- */
// #define TIM3_BASE        (TIM3_BASE_MORT)
// #define TIM3_CR1         (TIM3_BASE + 0x00)
// #define TIM3_CR2         (TIM3_BASE + 0x04)
// #define TIM3_SMCR        (TIM3_BASE + 0x08)
// #define TIM3_DIER        (TIM3_BASE + 0x0C)
// #define TIM3_SR          (TIM3_BASE + 0x10)
// #define TIM3_EGR         (TIM3_BASE + 0x14)
// #define TIM3_CCMR1       (TIM3_BASE + 0x18)
// #define TIM3_CCMR2       (TIM3_BASE + 0x1C)
// #define TIM3_CCER        (TIM3_BASE + 0x20)
// #define TIM3_CNT         (TIM3_BASE + 0x24)
// #define TIM3_PSC         (TIM3_BASE + 0x28)
// #define TIM3_ARR         (TIM3_BASE + 0x2C)
// #define TIM3_CCR1        (TIM3_BASE + 0x34)
// #define TIM3_CCR2        (TIM3_BASE + 0x38)
// #define TIM3_CCR3        (TIM3_BASE + 0x3C)
// #define TIM3_CCR4        (TIM3_BASE + 0x40)

/* CCMR (common to CCMR1/CCMR2) bitfields */
#define TIM_CCMR_CCxS_MASK       (3U  << 0)   /* CCxS[1:0] */
#define TIM_CCMR_OCxM_MASK       (7U  << 4)   /* OCxM[6:4] */
#define TIM_CCMR_OCxPE           (1U  << 3)   /* preload enable */
#define TIM_CCMR_ICxPSC_MASK     (3U  << 2)   /* ICxPSC[1:0] */
#define TIM_CCMR_ICxF_MASK       (0xFU<< 4)   /* ICxF[3:0] (shares [7:4]) */

/* Encodings */
#define TIM_CCMR_CCxS_OUTPUT     (0U  << 0)   /* CCxS = 00 (output) */
#define TIM_CCMR_CCxS_TI1        (1U  << 0)   /* CCxS = 01 (TI1) */
#define TIM_CCMR_OCxM_TOGGLE     (3U  << 4)   /* OCxM = 011 (toggle on match) */
#define TIM_CCMR_OCxM_PWM1       (6U  << 4)   /* OCxM = 110 (PWM mode 1) */

/* CCER helpers for channel x ∈ {1,2,3,4} */
#define TIM_CCER_CCxE(ch)        (1U << (4*((ch)-1)    ))  /* enable */
#define TIM_CCER_CCxP(ch)        (1U << (4*((ch)-1) + 1))  /* polarity */
#define TIM_CCER_CCxNP(ch)       (1U << (4*((ch)-1) + 3))  /* complementary polarity */

// /* CR1 / EGR bits */
// #define TIM_CR1_ARPE             (1U << 7)
// #define TIM_CR1_CEN              (1U << 0)
// #define TIM_EGR_UG               (1U << 0)

// /* Status/Interrupt bits*/
// #define TIM_SR_UIF        (1U << 0)  /* update (overflow) flag        */
// #define TIM_SR_CC3IF      (1U << 3)  /* ch3 compare flag              */
// #define TIM_DIER_UIE      (1U << 0)  /* update interrupt enable       */
// #define TIM_DIER_CC3IE    (1U << 3)  /* ch3 compare interrupt enable  */


/* ---- One-liners to clear flags (hide the weird sequences) ---- */
#define TIM3_CLEAR_CC3IF()  do { (void)TIM3->SR; (void)TIM3->CCR3; } while (0)
/* CCxIF clears by reading SR then reading CCRx */

#define TIM3_CLEAR_UIF()    do { TIM3->SR &= ~TIM_SR_UIF; } while (0)
/* UIF clears by writing 0 to SR.UIF */


void timer3_ch3_init_oc_toggle_1Hz(void)
{
    // 1) Enable APB1 clock for TIM3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 2) Compute time base for ~1 Hz toggle events.
       APB1 timer clock used in class = 90 MHz  (mbed config)  -> slide says APB1 timers = 90 MHz. */

    /* Let PSC = 8999 -> f_cnt = 90MHz/(8999+1) = 10 kHz
       Let ARR = 9999 -> event (CNT==CCR3) rate = f_cnt/(ARR+1) = 10k/10k = 1 Hz
       In OC Toggle mode, the pin toggles on each match -> LED flips once per second (~1 Hz). */
    TIM3->PSC  = 8999U;
    TIM3->ARR  = 9999U;

    /* 3) Position the compare for channel 3 somewhere in the period - I am taking half.. (any 0..ARR is fine). */
    TIM3->CCR3 = 5000U;

    // 4) Configure channel 3 as Output Compare, "toggle on match"
    // CC3S[1:0] = 00 (output mode)
    // OC3M[6:4] = 011 (toggle on match)
    TIM3->CCMR2 &= ~(TIM_CCMR_CCxS_MASK | TIM_CCMR_OCxM_MASK);   // clear CC3S and OC3M
    TIM3->CCMR2 |=  (TIM_CCMR_CCxS_OUTPUT | TIM_CCMR_OCxM_TOGGLE);   // CC3S=00, OC3M=0b011

    // 5) Enable channel 3 output on the pin
    TIM3->CCER |= TIM_CCER_CCxE(3);                   // CC3E=1

    /* 6) Force an update so PSC/ARR/CCR latch, then enable the counter */
    TIM3->EGR  = TIM_EGR_UG;                    // UG=1
    TIM3->CR1 |= TIM_CR1_CEN;                    // CEN=1 /* CEN=1 starts the counter */
}

void timer3_ch1_init_input_capture_rising_1MHz(void)
{
    /* 1) Enable TIM3 clock (APB1) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 2) 1 MHz counter tick: 90 MHz / (PSC+1) = 1 MHz → PSC=89 */
    TIM3->PSC = 89U;

    /* 3) Free-run to max (16-bit) */
    TIM3->ARR = 0xFFFFU;

    /* 4) Input-capture on CH1: CCMR1
          CC1S=01 (input mapped on TI1), IC1PSC=00, IC1F=0000 */
    TIM3->CCMR1 &= ~(TIM_CCMR_CCxS_MASK | TIM_CCMR_ICxPSC_MASK | TIM_CCMR_ICxF_MASK);
    TIM3->CCMR1 |=  (TIM_CCMR_CCxS_TI1);  /* CC1S=01 */

    /* 5) Rising edge: CC1P=0, CC1NP=0; enable capture: CC1E=1 */
    TIM3->CCER &= ~(TIM_CCER_CCxP(1) | TIM_CCER_CCxNP(1));
    TIM3->CCER |=  (TIM_CCER_CCxE(1));

    /* 6) Latch PSC/ARR, start counter */
    TIM3->EGR  = TIM_EGR_UG;     /* UG */
    TIM3->CR1 |= TIM_CR1_CEN;     /* CEN */
}


//Interrupts and PWMs
//Logic
/* Choose a friendly time base: f_tim ≈ 90 MHz (class clocking)
   Prescale to 10 kHz so ARR fits in 16-bit:
     PSC = 8999 → f_cnt = 90e6/(8999+1) = 10 kHz
   For 0.25 Hz PWM: period T=4 s → counts = f_cnt * T = 10k * 4 = 40,000 → ARR = 39,999.
   50% duty → CCR3 = 20,000.
*/


//helper
static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi){
    return v < lo ? lo : (v > hi ? hi : v);
}

void timer3_ch3_init_pwm_0p25Hz(uint8_t duty_percent)
{
    /* 1) Enable TIM3 clock (on APB1) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 2) Time base: 10 kHz counter, ARR=39999 → 0.25 Hz PWM period */
    TIM3->PSC = 8999U;           // f_cnt = 10 kHz
    TIM3->ARR = 39999U;          // period = (ARR+1)/f_cnt = 4 s

    /* 3) Channel 3 in PWM mode 1, preload enable */
    // CCMR2: OC3M bits [6:4] = 110 (PWM mode 1), OC3PE (bit 3) = 1, CC3S=00
    uint32_t ccmr2 = TIM3->CCMR2;
    ccmr2 &= ~(TIM_CCMR_CCxS_MASK | TIM_CCMR_OCxM_MASK);   // clear CC3S, OC3M
    ccmr2 |=  (TIM_CCMR_OCxM_PWM1 | TIM_CCMR_OCxPE | TIM_CCMR_CCxS_OUTPUT);  // OC3M=110 (PWM1), OC3PE=1, CC3S=00
    TIM3->CCMR2 = ccmr2;

    /* 4) Enable CH3 output, active-high */
    TIM3->CCER &= ~(TIM_CCER_CCxP(3) | TIM_CCER_CCxNP(3));
    TIM3->CCER |=  TIM_CCER_CCxE(3);           // CC3E = 1 (enable)  (enable channel output) :contentReference[oaicite:2]{index=2}

    /* 5) Set duty cycle (CCR3) from percentage */
    uint32_t arrp1 = TIM3->ARR + 1U;
    uint32_t ccr   = (arrp1 * clamp_u32(duty_percent,0,100)) / 100U;
    TIM3->CCR3 = ccr;

    /* 6) Enable auto-reload preload and generate update so all preload regs latch */
    TIM3->CR1  |= TIM_CR1_ARPE;      // ARPE = 1
    TIM3->EGR   = TIM_EGR_UG;        // UG (force update)

    /* 7) Start the counter */
    TIM3->CR1  |= TIM_CR1_CEN;      // CEN = 1 (counter enable)  :contentReference[oaicite:3]{index=3}
}

void timer3_ch3_set_duty_percent(uint8_t duty_percent)
{
    uint32_t arrp1 = TIM3->ARR + 1U;
    uint32_t ccr   = (arrp1 * clamp_u32(duty_percent,0,100)) / 100U;
    TIM3->CCR3 = ccr;
    TIM3->EGR  = TIM_EGR_UG;   // latch on next update
}

