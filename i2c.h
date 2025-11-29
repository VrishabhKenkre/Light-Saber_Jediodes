#ifndef __HARDWARE_STM_I2C1_H_
#define __HARDWARE_STM_I2C1_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void i2c1_init(void);
void i2c1_writeByte(uint8_t dev, uint8_t reg, uint8_t data);
uint8_t i2c1_readByte(uint8_t dev, uint8_t reg);
void i2c1_readMulti(uint8_t dev, uint8_t reg, int len, uint8_t *buf);

/* DMA API */
void i2c1_dma_init(void);
uint8_t i2c1_dma_read(uint8_t dev, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t i2c1_dma_isBusy(void);
void i2c1_dma_setCallback(void(*cb)(void));

#ifdef __cplusplus
}
#endif

#endif

