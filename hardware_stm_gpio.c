/**
  ******************************************************************************
  * @file    hardware_stm_gpio.c 
  * @author  mortamar@andrew.cmu.edu
  * @version 1.0
  * @date    Septembr-2021
  * @brief   Controls STM32F446ze GPIO
  ******************************************************************************
  */

#include "hardware_stm_gpio.h"
#include "stm32f4xx_rcc_mort.h"
#include <cstdint>


//led 1 is connected to PB0. 
// GPIO B addresses: 0x4002 0400 - 0x4002 07FF
// GPIO C addresses: 0x4002 0800 - 0x4002 0BFF


/* MACRO definitions----------------------------------------------------------*/
//Port B addresses:
#define PORTB_BASE_ADDRESS ((uint32_t)0x40020400)        //The first address in memory corresponding to Port B (this is in the user manual!)
// I gave you the first one, now you fill in the rest, check in the user manual what is the offset from the base address for each register!
#define PORTB_MODER_REGISTER (PORTB_BASE_ADDRESS + 0x00) //replace the question mark with the correct offset!
#define PORTB_OTYPER_REGISTER (PORTB_BASE_ADDRESS + 0x04)
#define PORTB_OSPEEDR_REGISTER (PORTB_BASE_ADDRESS + 0x08)
#define PORTB_PUPDR_REGISTER (PORTB_BASE_ADDRESS + 0x0C)
#define PORTB_IDR_REGISTER (PORTB_BASE_ADDRESS + 0x10)
#define PORTB_ODR_REGISTER (PORTB_BASE_ADDRESS + 0x14)
//#define PORTB_BSRRL_REGISTER (PORTB_BASE_ADDRESS + 0x18)
#define PORTB_BSRR_REGISTER (PORTB_BASE_ADDRESS + 0x18)
//#define PORTB_BSRRH_REGISTER (PORTB_BASE_ADDRESS + 0x1A)
//#define PORTB_LCKR_REGISTER (PORTB_BASE_ADDRESS + 0x1C)
#define PORTB_AFR1_REGISTER (PORTB_BASE_ADDRESS + 0x20)
#define PORTB_AFR2_REGISTER (PORTB_BASE_ADDRESS + 0x24)
//#define PORTB_OSPEEDR_REGISTER (PORTB_BASE_ADDRESS + 0x08)

//Port C addresses:
#define PORTC_BASE_ADDRESS      ((uint32_t)0x40020800)
#define PORTC_MODER_REGISTER    (PORTC_BASE_ADDRESS + 0x00)
#define PORTC_OTYPER_REGISTER   (PORTC_BASE_ADDRESS + 0x04)
#define PORTC_OSPEEDR_REGISTER  (PORTC_BASE_ADDRESS + 0x08)
#define PORTC_PUPDR_REGISTER    (PORTC_BASE_ADDRESS + 0x0C)
#define PORTC_IDR_REGISTER      (PORTC_BASE_ADDRESS + 0x10)
#define PORTC_ODR_REGISTER      (PORTC_BASE_ADDRESS + 0x14)
#define PORTC_BSRR_REGISTER     (PORTC_BASE_ADDRESS + 0x18)
#define PORTC_AFRL_REGISTER     (PORTC_BASE_ADDRESS + 0x20)
#define PORTC_AFRH_REGISTER     (PORTC_BASE_ADDRESS + 0x24)

//Some Masks
#define _2BIT_FIELD(pin)        (0x3u << ((pin) * 2))
#define _1BIT_FIELD(pin)        (1u    << (pin))

//so we can write a specific 2-bit value (00..11) at a pin’s field:
#define _2BIT_VAL(pin, v) ((uint32_t)((v) & 0x3u) << ((pin) * 2))


#define GPIO_0_MODER                 0x0003      // bits 1:0
#define GPIO_0_MODER_OUT             0x0001
#define GPIO_0_MODER_AF              0x0002

/* OTYPER bit for PB0 */
#define GPIO_0_OTYPER                0x0001
#define GPIO_0_OTYPER_PP             0x0000

/* PUPDR bits for PB0 */
#define GPIO_0_PUPDR                 0x0003
#define GPIO_0_PUPDR_NOPULL          0x0000

/* ODR bit for PB0 */
#define GPIO_0_ODR_HIGH              0x0001


/* MODER bits for PC6 */
#define GPIO_6_MODER                 0x3000      // bits 13:12
#define GPIO_6_MODER_AF              0x2000
#define GPIO_6_MODER_OUT             0x1000
#define GPIO_6_MODER_IN              0x0000
#define GPIO_6_MODER_AN              0x3000

/* OTYPER bit for PC6 */
#define GPIO_6_OTYPER                0x40        // bit 6
#define GPIO_6_OTYPER_PP             0x00

/* OSPEEDR bits for PC6 */
#define GPIO_6_OSPEEDR               0x3000
#define GPIO_6_OSPEEDR_HIGH_SPEED    0x3000

/* PUPDR bits for PC6 */
#define GPIO_6_PUPDR                 0x3000
#define GPIO_6_PUPDR_NOPULL          0x0000
#define GPIO_6_PUPDER_PD             0x2000      // pulldown (10b)

//Macros for PB1 and PB2
#define GPIO_1_MODER        (3U << (1*2))
#define GPIO_1_MODER_OUT    (1U << (1*2))
#define GPIO_1_OTYPER       (1U << 1)
#define GPIO_1_OTYPER_PP    (0U << 1)
#define GPIO_1_PUPDR        (3U << (1*2))
#define GPIO_1_PUPDR_NOPULL (0U << (1*2))
#define GPIO_1_ODR_HIGH     (1U << 1)

#define GPIO_2_MODER        (3U << (2*2))
#define GPIO_2_MODER_OUT    (1U << (2*2))
#define GPIO_2_OTYPER       (1U << 2)
#define GPIO_2_OTYPER_PP    (0U << 2)
#define GPIO_2_PUPDR        (3U << (2*2))
#define GPIO_2_PUPDR_NOPULL (0U << (2*2))
#define GPIO_2_ODR_HIGH     (1U << 2)



/* function definitions----------------------------------------------------------*/

void initGpioC6AsInput(void)
{
    uint32_t *reg_pointer;

    /* GPIOC Peripheral clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* GPIOC Pin 6 as input: MODER[13:12] = 00 */
    reg_pointer = (uint32_t *)PORTC_MODER_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_6_MODER) | GPIO_6_MODER_IN;

    /* PUSH-PULL Pin: OTYPER[6] = 0 */
    reg_pointer = (uint32_t *)PORTC_OTYPER_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_6_OTYPER) | GPIO_6_OTYPER_PP;

    /* GPIOC pin 6 high speed */
    reg_pointer = (uint32_t *)PORTC_OSPEEDR_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_6_OSPEEDR) | GPIO_6_OSPEEDR_HIGH_SPEED;

    /* Configure pulled-down: PUPDR[13:12] = 10 (pulldown → idle LOW) */
    reg_pointer = (uint32_t *)PORTC_PUPDR_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_6_PUPDR) | GPIO_6_PUPDER_PD;
}

void initGpioB0AsOutput(void)
{
    uint32_t *reg_pointer;

    /* GPIOB Peripheral clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* GPIOB0 configured as output: MODER[1:0] = 01 */
    reg_pointer = (uint32_t *)PORTB_MODER_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_0_MODER) | GPIO_0_MODER_OUT;

    /* GPIOB0 configured as push-pull */
    reg_pointer = (uint32_t *)PORTB_OTYPER_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_0_OTYPER) | GPIO_0_OTYPER_PP;

    /* GPIOB0 no pull-up/pull-down */
    reg_pointer = (uint32_t *)PORTB_PUPDR_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_0_PUPDR) | GPIO_0_PUPDR_NOPULL;

    /* Start with LED ON or OFF – your choice. Slides usually drive high: */
    reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
    *reg_pointer |= GPIO_0_ODR_HIGH;     // LED starts ON
    // If you want OFF initially: *reg_pointer &= ~GPIO_0_ODR_HIGH;
}

void toggleGPIOB0(void)
{
    uint32_t *reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
    uint32_t value = *reg_pointer & GPIO_0_ODR_HIGH;

    if (value > 0)
    {
        *reg_pointer &= ~GPIO_0_ODR_HIGH;    // turn OFF
    }
    else
    {
        *reg_pointer |= GPIO_0_ODR_HIGH;     // turn ON
    }
}


void setGPIOB0( void )
{
    *(volatile uint32_t*)PORTB_BSRR_REGISTER = _1BIT_FIELD(0);     
}
void clearGPIOB0( void )
{
     *(volatile uint32_t*)PORTB_BSRR_REGISTER = (_1BIT_FIELD(0) << 16);    
}

uint32_t checkGPIOC6(void)
{
    /* Return the sampled logic level on PC6 (0 or 1) */
    uint32_t valueC6 = ((*(volatile uint32_t*)PORTC_IDR_REGISTER) >> 6) & 1u;
    return valueC6;   
}

void setGPIOB1( void )
{
    *(volatile uint32_t*)PORTB_BSRR_REGISTER = _1BIT_FIELD(1);     
}

void clearGPIOB1( void )
{
    *(volatile uint32_t*)PORTB_BSRR_REGISTER = (_1BIT_FIELD(1) << 16);    
}

void setGPIOB2( void )
{
    *(volatile uint32_t*)PORTB_BSRR_REGISTER = _1BIT_FIELD(2);     
}

void clearGPIOB2( void )
{
    *(volatile uint32_t*)PORTB_BSRR_REGISTER = (_1BIT_FIELD(2) << 16);    
}

void initGpioB1AsOutput(void)
{
    uint32_t *reg_pointer;

    /* GPIOB Peripheral clock enable (harmless if already enabled elsewhere) */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* GPIOB1 configured as output: MODER[3:2] = 01 */
    reg_pointer = (uint32_t *)PORTB_MODER_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_1_MODER) | GPIO_1_MODER_OUT;

    /* GPIOB1 configured as push-pull */
    reg_pointer = (uint32_t *)PORTB_OTYPER_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_1_OTYPER) | GPIO_1_OTYPER_PP;

    /* GPIOB1 no pull-up/pull-down */
    reg_pointer = (uint32_t *)PORTB_PUPDR_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_1_PUPDR) | GPIO_1_PUPDR_NOPULL;

    /* Start LED on PB1 in known state (here: OFF) */
    reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
    *reg_pointer &= ~GPIO_1_ODR_HIGH;    // LED starts OFF
}

void initGpioB2AsOutput(void)
{
    uint32_t *reg_pointer;

    /* GPIOB Peripheral clock enable (harmless if already enabled elsewhere) */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* GPIOB2 configured as output: MODER[5:4] = 01 */
    reg_pointer = (uint32_t *)PORTB_MODER_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_2_MODER) | GPIO_2_MODER_OUT;

    /* GPIOB2 configured as push-pull */
    reg_pointer = (uint32_t *)PORTB_OTYPER_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_2_OTYPER) | GPIO_2_OTYPER_PP;

    /* GPIOB2 no pull-up/pull-down */
    reg_pointer = (uint32_t *)PORTB_PUPDR_REGISTER;
    *reg_pointer = (*reg_pointer & ~GPIO_2_PUPDR) | GPIO_2_PUPDR_NOPULL;

    /* Start LED on PB2 OFF */
    reg_pointer = (uint32_t *)PORTB_ODR_REGISTER;
    *reg_pointer &= ~GPIO_2_ODR_HIGH;
}



/////TIMERSssssssss...
void initGpioB0AsAF2_TIM3CH3(void)
{
    /* Turn on GPIOB clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* PB0 -> Alternate Function (MODER[1:0] = 10) */
    *(volatile uint32_t*)PORTB_MODER_REGISTER  &= ~_2BIT_FIELD(0);
    *(volatile uint32_t*)PORTB_MODER_REGISTER  |=  _2BIT_VAL(0, 0x2u);

    /* Push-pull, no pull */
    *(volatile uint32_t*)PORTB_OTYPER_REGISTER &= ~_1BIT_FIELD(0);
    *(volatile uint32_t*)PORTB_PUPDR_REGISTER  &= ~_2BIT_FIELD(0);

    /*We want it Fasssssssst*/
    /* So PB0 : OSPEEDR[1:0] = 10 */
    *(volatile uint32_t*)PORTB_OSPEEDR_REGISTER &= ~_2BIT_FIELD(0);
    *(volatile uint32_t*)PORTB_OSPEEDR_REGISTER |= _2BIT_VAL(0, 0x2u);

    /* AFRL nibble for PB0 = AF2 (TIM3/4/5) */
    /* *4 because it has a 4 bit field */
    *(volatile uint32_t*)PORTB_AFR1_REGISTER   &= ~(0xFu << (0 * 4));
    *(volatile uint32_t*)PORTB_AFR1_REGISTER   |=  (0x2u << (0 * 4));
}

void initGpioC6AsAF2_TIM3CH1(void)
{
    /* Enable GPIOC clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* PC6 → Alternate Function (MODER[13:12] = 10) */
    *(volatile uint32_t*)PORTC_MODER_REGISTER &= ~_2BIT_FIELD(6);
    *(volatile uint32_t*)PORTC_MODER_REGISTER |=  _2BIT_VAL(6, 0x2u);

    /* Push-pull (harmless) and no pull (source will drive) */
    *(volatile uint32_t*)PORTC_OTYPER_REGISTER &= ~_1BIT_FIELD(6);
    *(volatile uint32_t*)PORTC_PUPDR_REGISTER  &= ~_2BIT_FIELD(6);

    /* AFRL nibble for PC6 = AF2 (TIM3 group) → bits [27:24] = 0x2 */
    *(volatile uint32_t*)PORTC_AFRL_REGISTER &= ~(0xFu << (6 * 4));
    *(volatile uint32_t*)PORTC_AFRL_REGISTER |=  (0x2u << (6 * 4));
}

//Helpers
void PC6_use_pullup(void) {
    *(volatile uint32_t*)PORTC_PUPDR_REGISTER &= ~_2BIT_FIELD(6);
    *(volatile uint32_t*)PORTC_PUPDR_REGISTER |=  _2BIT_VAL(6, 0x1u); // 01 = pull-up
}
void PC6_use_pulldown(void) {
    *(volatile uint32_t*)PORTC_PUPDR_REGISTER &= ~_2BIT_FIELD(6);
    *(volatile uint32_t*)PORTC_PUPDR_REGISTER |=  _2BIT_VAL(6, 0x2u); // 10 = pull-down
}