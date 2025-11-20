/**
  ******************************************************************************
  * @file    nucleo_led.c 
  * @author  mortamar@andrew.cmu.edu
  * @version 1.0
  * @date    Septembr-2021
  * @brief   Controls the LED's on the nucleo board
  ******************************************************************************
  */

#include "hardware_stm_gpio.h"
#include "nucleo_led.h"

/************************************
* Initializes LED1 on the nucleo Board which is connected to Port B Pin 0
*************************************/
void init_LED1(void )
{
    // Call something from hardware_stm_gpio
    initGpioB0AsOutput();
}
/************************************
* Toggles LED1 
*************************************/
void toggle_LED1( void )
{
    // Call something else from hardware_stm_gpio
    toggleGPIOB0();
}

void init_PC6_input(void)
{
    // hardware layer configures PC6 as input + pull (we set pull-down there)
    initGpioC6AsInput();
}

uint32_t read_PC6(void)
{
    return checkGPIOC6();
}

