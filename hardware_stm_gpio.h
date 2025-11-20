/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HARDWARE_STM_GPIO_H_
#define __HARDWARE_STM_GPIO_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


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
void initGpioB0AsOutput( void );
void setGPIOB0( void );
void clearGPIOB0( void );
void toggleGPIOB0( void );
void initGpioC6AsInput( void );
uint32_t checkGPIOC6(void);

void clearGPIOB0(void);
uint32_t readGPIOC6(void);


void initGpioB0AsAF2_TIM3CH3(void);
void initGpioC6AsAF2_TIM3CH1(void);

void PC6_use_pullup(void);
void PC6_use_pulldown(void);


void initGpioB1AsOutput(void);
void setGPIOB1( void );
void clearGPIOB1(void);
void initGpioB2AsOutput(void);
void setGPIOB2(void);
void clearGPIOB2(void);

#ifdef __cplusplus
}
#endif

#endif /*__GPIO_H */
