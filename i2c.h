#ifndef __i2c_H
#define __i2c_H

#include "stm32f4xx_rcc_mort.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void my_i2c_init(void);
uint8_t i2c_readByte(uint8_t saddr, uint8_t maddr, uint8_t *data);
void i2c_bus_scan(void);
void i2c_write_byte(uint8_t saddr,uint8_t maddr, uint8_t data);
void i2c_ReadMulti(uint8_t saddr,uint8_t maddr, int n, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
