#include <rtthread.h>
#include "main_Cpp.h"
#include "simpleFoc/sensor/AS5600_I2C.h"

#define DBG_TAG "main_Cpp"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

extern struct AS5600_I2CConfig_s AS5600_I2C_Config;
extern I2C_HandleTypeDef hi2c1;

AS5600_I2C AS5600_I2C_1(AS5600_I2C_Config);


void main_Cpp(void) {

    AS5600_I2C_1.init(&hi2c1);
    float velocity = 0;
    while (1) {
        AS5600_I2C_1.update();

        velocity = AS5600_I2C_1.getVelocity();

        LOG_I("--------------------------------");
        LOG_I("AS5600_I2C_1.getSensorAngle(): %d.%03d", (int)AS5600_I2C_1.getSensorAngle(), (int)(AS5600_I2C_1.getSensorAngle() * 1000) % 1000);
        LOG_I("AS5600_I2C_1.getVelocity(): %d.%03d\n", (int)velocity, (int)(velocity * 1000) % 1000);
        rt_thread_mdelay(10);
    }
}