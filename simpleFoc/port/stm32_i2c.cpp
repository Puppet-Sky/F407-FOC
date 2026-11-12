#include "stm32_i2c.h"

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

int stm32_i2c_read_mem(uint8_t addr_7bit, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c1, addr_7bit << 1, reg, I2C_MEMADD_SIZE_8BIT,
                         data, len, 50) != HAL_OK)
    {
        return -1;
    }
    return 0;
}
