#include "i2c.h"
#include "debug_mort.h"
#include "stm32f4xx_rcc_mort.h"
#include "stm32f4xx_mort2.h"


/* ================================
 * I2C1 BASE + REGISTER OFFSETS
 * ================================ */
#define I2C1_BASE_ADDRESS      ((uint32_t)0x40005400)

#define I2C1_CR1               (*(volatile uint32_t *)(I2C1_BASE_ADDRESS + 0x00))
#define I2C1_CR2               (*(volatile uint32_t *)(I2C1_BASE_ADDRESS + 0x04))
#define I2C1_OAR1              (*(volatile uint32_t *)(I2C1_BASE_ADDRESS + 0x08))
#define I2C1_OAR2              (*(volatile uint32_t *)(I2C1_BASE_ADDRESS + 0x0C))
#define I2C1_DR                (*(volatile uint32_t *)(I2C1_BASE_ADDRESS + 0x10))
#define I2C1_SR1               (*(volatile uint32_t *)(I2C1_BASE_ADDRESS + 0x14))
#define I2C1_SR2               (*(volatile uint32_t *)(I2C1_BASE_ADDRESS + 0x18))
#define I2C1_CCR               (*(volatile uint32_t *)(I2C1_BASE_ADDRESS + 0x1C))
#define I2C1_TRISE             (*(volatile uint32_t *)(I2C1_BASE_ADDRESS + 0x20))

/* ================================
 * I2C1 BIT DEFINITIONS
 * ================================ */
#define I2C_CR1_PE             (1 << 0)
#define I2C_CR1_START          (1 << 8)
#define I2C_CR1_STOP           (1 << 9)
#define I2C_CR1_ACK            (1 << 10)
#define I2C_CR1_SWRST          (1 << 15)

#define I2C_SR1_SB             (1 << 0)
#define I2C_SR1_ADDR           (1 << 1)
#define I2C_SR1_BTF            (1 << 2)
#define I2C_SR1_RXNE           (1 << 6)
#define I2C_SR1_TXE            (1 << 7)

#define I2C_SR2_BUSY           (1 << 1)

#define I2C_CR2_DMAEN          (1 << 11)
#define I2C_CR2_LAST           (1 << 12)

/* DMA1 Stream0 for I2C1_RX ------------------------------------- */
#define DMA1_BASE_ADDR         0x40026000
#define DMA1_STREAM0           ((uint32_t)(DMA1_BASE_ADDR + 0x010))
#define DMA1_S0CR              (*(volatile uint32_t *)(DMA1_BASE_ADDR + 0x010))
#define DMA1_S0NDTR            (*(volatile uint32_t *)(DMA1_BASE_ADDR + 0x014))
#define DMA1_S0PAR             (*(volatile uint32_t *)(DMA1_BASE_ADDR + 0x018))
#define DMA1_S0M0AR            (*(volatile uint32_t *)(DMA1_BASE_ADDR + 0x01C))
#define DMA1_S0FCR             (*(volatile uint32_t *)(DMA1_BASE_ADDR + 0x024))

/* DMA interrupt flag registers */
#define DMA1_LISR              (*(volatile uint32_t *)(DMA1_BASE_ADDR + 0x000))
#define DMA1_LIFCR             (*(volatile uint32_t *)(DMA1_BASE_ADDR + 0x008))

#define DMA_FLAG_TCIF0         (1 << 5)

#define PORTB_BASE_ADDRESS      ((uint32_t)0x40020400)
#define PORTB_MODER             (*(volatile uint32_t *)(PORTB_BASE_ADDRESS + 0x00))
#define PORTB_OTYPER            (*(volatile uint32_t *)(PORTB_BASE_ADDRESS + 0x04))
#define PORTB_AFRL  (*(volatile uint32_t *)(PORTB_BASE_ADDRESS + 0x20))
#define PORTB_AFRH  (*(volatile uint32_t *)(PORTB_BASE_ADDRESS + 0x24))

#define SYSTEM_CONTROL_BASE_ADDRESS                     (0xE000E000)
#define NVIC_BASE_ADDRESS                               (SYSTEM_CONTROL_BASE_ADDRESS + 0x100)

/* NVIC Interrupt Set-Enable Registers */
#define NVIC_INTERRUPT_SET_ENABLE_REGISTER_0_31         (*(volatile uint32_t *)(NVIC_BASE_ADDRESS))

static volatile uint8_t i2c1_dma_busy = 0;
static void (*i2c1_dma_callback)(void) = 0;

/* ================================================================
 *                       BASIC I2C INITIALIZATION
 * ================================================================ */
void i2c1_init(void)
{
    /* Enable clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOBEN_MORT, ENABLE_MORT);
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_I2C1EN_MORT, ENABLE_MORT);
    
    /* PB8(SCL), PB9(SDA) AF4 */
    PORTB_MODER |= (0x2 << (8 * 2)) | (0x2 << (9 * 2));
    PORTB_AFRH |= (0x4 << ((8 - 8) * 4)) | (0x4 << ((9 - 8) * 4));
    PORTB_OTYPER |= (1 << 8) | (1 << 9);

    /* Reset I2C */
    I2C1_CR1 |= I2C_CR1_SWRST;
    I2C1_CR1 &= ~I2C_CR1_SWRST;

    /* Peripheral clock frequency = 16 MHz */
    I2C1_CR2 = 16;

    /* Standard mode 100kHz: CCR = 80, TRISE = 17 */
    I2C1_CCR = 160;
    I2C1_TRISE = 17;

    /* Enable peripheral */
    I2C1_CR1 |= I2C_CR1_PE;
}

/* ================================================================
 *                   BLOCKING SINGLE-BYTE READ
 * ================================================================ */
uint8_t i2c1_readByte(uint8_t dev, uint8_t reg)
{
    volatile uint32_t temp;

    while (I2C1_SR2 & I2C_SR2_BUSY);

    /* START + Address(Write) */
    I2C1_CR1 |= I2C_CR1_START;
    while (!(I2C1_SR1 & I2C_SR1_SB));

    I2C1_DR = dev << 1;
    while (!(I2C1_SR1 & I2C_SR1_ADDR));
    temp = I2C1_SR2;

    /* Write register */
    while (!(I2C1_SR1 & I2C_SR1_TXE));
    I2C1_DR = reg;

    while (!(I2C1_SR1 & I2C_SR1_TXE));
    /* Repeated START */
    I2C1_CR1 |= I2C_CR1_START;

    while (!(I2C1_SR1 & I2C_SR1_SB));
    I2C1_DR = (dev << 1) | 1;

    while (!(I2C1_SR1 & I2C_SR1_ADDR));
    I2C1_CR1 &= ~I2C_CR1_ACK;
    temp = I2C1_SR2;

    /* STOP + read */
    I2C1_CR1 |= I2C_CR1_STOP;
    while (!(I2C1_SR1 & I2C_SR1_RXNE));

    return (uint8_t)I2C1_DR;
}

void i2c1_writeByte(uint8_t dev, uint8_t reg, uint8_t data)
{
    volatile uint32_t temp;

    /* Wait until bus is free */
    while (I2C1_SR2 & I2C_SR2_BUSY);

    /* START condition */
    I2C1_CR1 |= I2C_CR1_START;
    while (!(I2C1_SR1 & I2C_SR1_SB));

    /* Send device address (write) */
    I2C1_DR = dev << 1;
    while (!(I2C1_SR1 & I2C_SR1_ADDR));
    temp = I2C1_SR2;    // clear ADDR

    /* Send register address */
    while (!(I2C1_SR1 & I2C_SR1_TXE));
    I2C1_DR = reg;

    /* Send data byte */
    while (!(I2C1_SR1 & I2C_SR1_TXE));
    I2C1_DR = data;

    /* Wait for transfer complete */
    while (!(I2C1_SR1 & I2C_SR1_BTF));

    /* STOP condition */
    I2C1_CR1 |= I2C_CR1_STOP;
}

/* ================================================================
 *                  BLOCKING MULTI-BYTE READ
 * ================================================================ */
void i2c1_readMulti(uint8_t dev, uint8_t reg, int n, uint8_t *buf)
{
    volatile uint32_t tmp;

    while (I2C1_SR2 & I2C_SR2_BUSY);

    /* Register write phase */
    I2C1_CR1 |= I2C_CR1_START;
    while (!(I2C1_SR1 & I2C_SR1_SB));
    I2C1_DR = dev << 1;

    while (!(I2C1_SR1 & I2C_SR1_ADDR));
    tmp = I2C1_SR2;

    while (!(I2C1_SR1 & I2C_SR1_TXE));
    I2C1_DR = reg;

    while (!(I2C1_SR1 & I2C_SR1_TXE));

    /* Restart for read */
    I2C1_CR1 |= I2C_CR1_START;
    while (!(I2C1_SR1 & I2C_SR1_SB));

    I2C1_DR = (dev << 1) | 1;
    while (!(I2C1_SR1 & I2C_SR1_ADDR));
    tmp = I2C1_SR2;

    I2C1_CR1 |= I2C_CR1_ACK;

    while (n > 0)
    {
        if (n == 1)
        {
            I2C1_CR1 &= ~I2C_CR1_ACK;
            I2C1_CR1 |= I2C_CR1_STOP;
        }

        while (!(I2C1_SR1 & I2C_SR1_RXNE));
        *buf++ = (uint8_t)I2C1_DR;
        n--;
    }
}

/* ================================================================
 *                       DMA INITIALIZATION
 * ================================================================ */
 void dma1s0_enable_irq(void)
{
    /* Set bit in ISER0 */
    NVIC_INTERRUPT_SET_ENABLE_REGISTER_0_31 = (1UL << 11U);
}

void i2c1_dma_init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_DMA1EN_MORT, ENABLE_MORT);
    
    DMA1_S0CR &= ~1;
    while (DMA1_S0CR & 1);

    DMA1_LIFCR = 0x3F;
    dma1s0_enable_irq();
}

/* ================================================================
 *                     DMA NON-BLOCKING READ
 * ================================================================ */
uint8_t i2c1_dma_read(uint8_t dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    volatile uint32_t tmp;

    if (i2c1_dma_busy) return 1;
    if (len < 1) return 2;

    i2c1_dma_busy = 1;

    /* -------------------------------
     * Phase 1: Register write
     * ------------------------------- */
    while (I2C1_SR2 & I2C_SR2_BUSY);

    I2C1_CR1 |= I2C_CR1_START;
    while (!(I2C1_SR1 & I2C_SR1_SB));

    I2C1_DR = dev << 1;               // write mode
    while (!(I2C1_SR1 & I2C_SR1_ADDR));
    tmp = I2C1_SR2;                   // clear ADDR

    while (!(I2C1_SR1 & I2C_SR1_TXE));
    I2C1_DR = reg;

    while (!(I2C1_SR1 & I2C_SR1_BTF)); // wait for reg write to finish


    /* -------------------------------
     * Phase 2: Repeated START for read
     * ------------------------------- */
    I2C1_CR1 |= I2C_CR1_START;
    while (!(I2C1_SR1 & I2C_SR1_SB));

    I2C1_DR = (dev << 1) | 1;         // read mode


    /* -------------------------------
     * Phase 3: Configure DMA BEFORE clearing ADDR
     * ------------------------------- */
    DMA1_S0CR &= ~1;                  // disable stream
    while (DMA1_S0CR & 1);

    DMA1_LIFCR = 0x3F;                // clear all flags

    DMA1_S0PAR  = (uint32_t)&I2C1_DR; // peripheral = I2C_DR
    DMA1_S0M0AR = (uint32_t)buf;      // memory buffer
    DMA1_S0NDTR = len;

    /* Configure: Channel 1, MINC, TC interrupt */
    DMA1_S0CR =
        (1 << 25) |    // CHSEL = 1 (I2C1_RX)
        (1 << 10) |    // MINC
        (1 << 4);      // TCIE


    /* -------------------------------
     * NOW clear ADDR → this starts I2C DMA RX
     * ------------------------------- */
    while (!(I2C1_SR1 & I2C_SR1_ADDR));
    tmp = I2C1_SR2;                   // start reception


    /* Enable I2C DMA */
    I2C1_CR1 |= I2C_CR1_ACK;          // ACK all except last
    I2C1_CR2 |= I2C_CR2_DMAEN;        // enable DMA for I2C

    /* For last byte handling, LAST bit needed */
    I2C1_CR2 |= I2C_CR2_LAST;         // send N-1 ACK then NACK last


    /* Start DMA */
    DMA1_S0CR |= 1;

    return 0;
}


uint8_t i2c1_dma_isBusy(void)
{
    return i2c1_dma_busy;
}

void i2c1_dma_setCallback(void (*cb)(void))
{
    i2c1_dma_callback = cb;
}

/* ================================================================
 *                       DMA IRQ HANDLER
 * ================================================================ */
void DMA1_Stream0_IRQHandler(void)
{
    if (DMA1_LISR & DMA_FLAG_TCIF0)
    {
        DMA1_LIFCR = 0x3F;
        DMA1_S0CR &= ~1;

        I2C1_CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
        I2C1_CR1 |= I2C_CR1_STOP;

        i2c1_dma_busy = 0;

        if (i2c1_dma_callback)
            i2c1_dma_callback();
    }
}


// void i2c_bus_scan(void)
// {        
// 		int a=0; //address variable
//          for (uint8_t i=0;i<128;i++) //go through all 127 address
//    {
//             I2C1->CR1 |= I2C_CR1_START; //generate start 
//             while(!(I2C1->SR1 & I2C_SR1_SB)); // wait to start to be generated
//             I2C1->DR=(i<<1|0); // transmit the address
//             while(!(I2C1->SR1)|!(I2C1->SR2)){}; //clear status register
//             I2C1->CR1 |= I2C_CR1_STOP; //generate stop condition
//             delay(100);//minium wait time is 40 uS, but for sure, leave it 100 uS
//             a=(I2C1->SR1&I2C_SR1_ADDR); //read the status register address set
//             if (a==2)//if the address is valid
//          {
// 					 //print the address
// 					 printf(data,"Found I2C device at adress 0x%X (hexadecimal), or %d (decimal)\r\n",i,i);
//            UART_Write_String(data);
//          }
//      }
// }

