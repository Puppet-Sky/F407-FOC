#ifndef STM32_I2C_H
#define STM32_I2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int stm32_i2c_read_mem(uint8_t addr_7bit, uint8_t reg, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
