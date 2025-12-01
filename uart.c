#include "stm32f4xx.h"
#include "uart.h"

void USART3_Init(void)
{
    /* 1) Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // GPIO D
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // UART3

    /* 2) PD8 = TX, PD9 = RX → AF7 */

    // MODER: alternate function
    GPIOD->MODER &= ~(3U << (8*2));
    GPIOD->MODER &= ~(3U << (9*2));
    GPIOD->MODER |=  (2U << (8*2));
    GPIOD->MODER |=  (2U << (9*2));

    // AFR[1] (pins 8..15)
    GPIOD->AFR[1] &= ~(0xF << (0));       // clear PD8 (index 0)
    GPIOD->AFR[1] &= ~(0xF << (4));       // clear PD9 (index 1)
    GPIOD->AFR[1] |=  (7U << (0));        // AF7 for PD8
    GPIOD->AFR[1] |=  (7U << (4));        // AF7 for PD9

    /* 3) Baud rate: 115200
       APB1 on STM32F446 = 45 MHz (default)
       USARTDIV = 45,000,000 / 115,200 = 390.6
       BRR = 390 = 0x186
    */
    USART3->BRR = 0x186;

    /* 4) Enable TX, RX, USART */
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    /* tiny delay after enabling UE (mandatory) */
    (void)USART3->SR;
}


static void usart_write_char(int ch)
{
    while (!(USART3->SR & USART_SR_TXE)) {;}
    USART3->DR = (uint16_t)ch;
}

void UART_Write_String(const char *p)
{
    while (*p != '\0')
    {
        usart_write_char(*p++);
    }
}
	