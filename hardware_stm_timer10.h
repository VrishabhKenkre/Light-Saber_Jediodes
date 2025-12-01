/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HARDWARE_STM_TIMER10_H_
#define __HARDWARE_STM_TIMER10_H_


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdint.h"

/* Macros for Everyone--------------------------------------------------------*/
#define PIN_0   0
#define PIN_1   1
#define PIN_2   2
#define PIN_3   3
#define PIN_4   4
#define PIN_5   5
#define PIN_6   6
#define PIN_7   7
#define PIN_8   8
#define PIN_9   9
#define PIN_10  10

/*Function definitions---------------------------------------------------------*/
uint32_t tim10_arr(void);
void tim10_set_duty(uint32_t duty);
void timer10_pwm_init_PB8(uint16_t psc, uint16_t arr);
void timer10_pwm_init_PF6(uint16_t psc, uint16_t arr);
void init_timer10ToInterrupt(uint16_t psc, uint16_t arr);//****
void TIM1_UP_TIM10_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /*__TIMER10_H*/

