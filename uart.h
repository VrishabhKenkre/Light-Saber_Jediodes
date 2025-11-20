// uart.h
#ifndef __uart__h
#define __uart__h

#include "stm32f4xx.h"
#include "stdio.h"

#ifdef __cplusplus
extern "C" {
#endif

void USART2_Init(void);
void UART_Write_String(const char *p);

#ifdef __cplusplus
}
#endif

#endif

