#include "i2c.h"
#include "uart.h"
#include "stdio.h"
#include "stm32f4xx_rcc_mort.h"
#include "stm32f4xx_mort2.h"
#include "Time_Out.h" 

char data[50];

void my_i2c_init(void){
RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN; //enable gpiob clock
RCC->APB1ENR|=RCC_APB1ENR_I2C1EN; //enable i2c1 clock
GPIOB->MODER|=0xA0000; //set pb8 and 9 to alternative function
GPIOB->AFR[1]|=0x44;
GPIOB->OTYPER|=GPIO_OTYPER_OT_8|GPIO_OTYPER_OT_9; //set pb8 and pb9 as open drain
I2C1->CR1=I2C_CR1_SWRST;//reset i2c
I2C1->CR1&=~I2C_CR1_SWRST;// release reset i2c	
I2C1->CR2|=16;//set clock source to 16MHz
I2C1->CCR=80;  //based on calculation
I2C1->TRISE=17; //output max rise 
I2C1->CR1|=I2C_CR1_PE; //enable I2C
}

uint8_t i2c_readByte(uint8_t saddr, uint8_t maddr, uint8_t *data)
{
    volatile int tmp;
    while (I2C1->SR2 & I2C_SR2_BUSY) {;}
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {;}
    I2C1->DR = saddr << 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {;}
    tmp = I2C1->SR2;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {;}
    I2C1->DR = maddr;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {;}
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {;}
    I2C1->DR = (saddr << 1) | 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {;}
    I2C1->CR1 &= ~I2C_CR1_ACK;
    tmp = I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE)) {;}
    *data = (uint8_t)I2C1->DR;
    return 0;
}


void i2c_bus_scan(void)
{        
					int a=0; //address variable
         for (uint8_t i=0;i<128;i++) //go through all 127 address
   {
            I2C1->CR1 |= I2C_CR1_START; //generate start 
            while(!(I2C1->SR1 & I2C_SR1_SB)); // wait to start to be generated
            I2C1->DR=(i<<1|0); // transmit the address
            while(!(I2C1->SR1)|!(I2C1->SR2)){}; //clear status register
            I2C1->CR1 |= I2C_CR1_STOP; //generate stop condition
            delay(100);//minium wait time is 40 uS, but for sure, leave it 100 uS
            a=(I2C1->SR1&I2C_SR1_ADDR); //read the status register address set
            if (a==2)//if the address is valid
         {
					 //print the address
					 printf(data,"Found I2C device at adress 0x%X (hexadecimal), or %d (decimal)\r\n",i,i);
           UART_Write_String(data);
         }
     }
}

void i2c_write_byte(uint8_t saddr, uint8_t maddr, uint8_t data)
{
    volatile int Temp;
    while (I2C1->SR2 & I2C_SR2_BUSY) {;}
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {;}
    I2C1->DR = saddr << 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {;}
    Temp = I2C1->SR2;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {;}
    I2C1->DR = maddr;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {;}
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) {;}
    I2C1->CR1 |= I2C_CR1_STOP;
}



void i2c_ReadMulti(uint8_t saddr, uint8_t maddr, int n, uint8_t *data)
{
    volatile int temp;
    while (I2C1->SR2 & I2C_SR2_BUSY) {;}
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {;}
    I2C1->DR = saddr << 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {;}
    temp = I2C1->SR2;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {;}
    I2C1->DR = maddr;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {;}
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {;}
    I2C1->DR = (saddr << 1) | 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {;}
    temp = I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_ACK;

    while (n > 0U) {
        if (n == 1U) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
            while (!(I2C1->SR1 & I2C_SR1_RXNE)) {;}
            *data++ = (uint8_t)I2C1->DR;
            break;
        } else {
            while (!(I2C1->SR1 & I2C_SR1_RXNE)) {;}
            *data++ = (uint8_t)I2C1->DR;
            n--;
        }
    }
}
