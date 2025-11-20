#include "stm32f4xx.h"
#include "uart.h"

void USART2_Init(void)   // name stays USART2_Init() so main.cpp doesn’t change
{
    // 1) Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;   // GPIOD clock
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // USART3 clock

    // 2) PD8 = TX (AF7), PD9 = RX (AF7)
    // Clear mode bits
    GPIOD->MODER &= ~(3U << (8 * 2));
    GPIOD->MODER &= ~(3U << (9 * 2));
    // Set to alternate function
    GPIOD->MODER |=  (2U << (8 * 2));
    GPIOD->MODER |=  (2U << (9 * 2));

    // Select AF7 for PD8/PD9
    GPIOD->AFR[1] &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
    GPIOD->AFR[1] |=  (7U  << ((8 - 8) * 4)) | (7U  << ((9 - 8) * 4));

    // 3) Baudrate: 115200 @ 16 MHz APB1
    USART3->BRR = 0x0187;

    // 4) Enable TX and USART
    USART3->CR1 = USART_CR1_TE | USART_CR1_UE;
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
	