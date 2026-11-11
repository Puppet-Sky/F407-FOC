#include <rtthread.h>
#include <stdlib.h>
#include <finsh.h>
#include "main_Cpp.h"
#include "simpleFoc/sensor/AS5600_I2C.h"
#include "simpleFoc/drivers/BLDCDriver3PWM.h"
#include "simpleFoc/BLDCMotor.h"
#include "board.h"
#include "stm32f4xx_hal_tim.h"
#include "lowpass_filter.h"

#define DBG_TAG "main_Cpp"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

extern struct AS5600_I2CConfig_s AS5600_I2C_Config;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

AS5600_I2C AS5600_I2C_1(AS5600_I2C_Config);
BLDCDriver3PWM motor_driver(PWM_A_PIN, PWM_B_PIN, PWM_C_PIN);
BLDCMotor motor(7);

static volatile float target_velocity = 20.0f;
static volatile float measured_velocity = 0.0f;
static volatile int velocity_print_enable = 0;
static LowPassFilter *velocity_lpf = RT_NULL;

static void format_fixed(char *buf, rt_size_t len, float value)
{
    int negative = value < 0.0f;
    float magnitude = negative ? -value : value;
    int integer = (int)magnitude;
    int fraction = (int)((magnitude - (float)integer) * 1000.0f + 0.5f);
    if (fraction >= 1000)
    {
        fraction -= 1000;
        integer += 1;
    }
    rt_snprintf(buf, len, "%s%d.%03d", negative ? "-" : "", integer, fraction);
}

static void sensor_entry(void *parameter)
{
    (void)parameter;
    while (1)
    {
        AS5600_I2C_1.update();
        if (velocity_lpf != RT_NULL)
        {
            measured_velocity = (*velocity_lpf)(AS5600_I2C_1.getVelocity());
        }
        rt_thread_mdelay(1);
    }
}

extern "C" int speed(int argc, char **argv)
{
    char target[16];
    char measured[16];

    if (argc >= 2)
    {
        target_velocity = (float)atof(argv[1]);
    }
    format_fixed(target, sizeof(target), target_velocity);
    format_fixed(measured, sizeof(measured), measured_velocity);
    rt_kprintf("target %s rad/s, measured %s rad/s\n", target, measured);
    return 0;
}
MSH_CMD_EXPORT(speed, set or show openloop speed in rad/s);

extern "C" int vprint(int argc, char **argv)
{
    if (argc >= 2)
    {
        velocity_print_enable = atoi(argv[1]) ? 1 : 0;
    }
    rt_kprintf("velocity print %s\n", velocity_print_enable ? "on" : "off");
    return 0;
}
MSH_CMD_EXPORT(vprint, vprint 1 to enable speed log);

void main_Cpp(void) {

    AS5600_I2C_1.init(&hi2c1);

    motor_driver.voltage_power_supply = 12;
    motor_driver.voltage_limit = 10;
    motor_driver.init();

    motor.linkDriver(&motor_driver);
    motor.voltage_limit = 5;
    motor.controller = MotionControlType::velocity_openloop;
    motor.init();

    rt_pin_write(PWM_ENABLE_PIN, PIN_HIGH);

    rt_thread_mdelay(1000);
    HAL_TIM_Base_Start_IT(&htim2);

    /* 1 ms 采样，避免高速时两次角度差超过半圈而被判成反转 */
    AS5600_I2C_1.min_elapsed_time = 0.001f;
    static LowPassFilter lpf(0.05f);
    velocity_lpf = &lpf;

    static struct rt_thread sensor_thread;
    static rt_uint8_t sensor_stack[2048];
    if (rt_thread_init(&sensor_thread, "sensor", sensor_entry, RT_NULL,
                       sensor_stack, sizeof(sensor_stack), 5, 10) == RT_EOK)
    {
        rt_thread_startup(&sensor_thread);
    }

    while (1)
    {
        if (velocity_print_enable)
        {
            char target[16];
            char measured[16];
            format_fixed(target, sizeof(target), target_velocity);
            format_fixed(measured, sizeof(measured), measured_velocity);
            rt_kprintf("target %s rad/s, measured %s rad/s\n", target, measured);
        }
        rt_thread_mdelay(500);
    }
}

void _writeDutyCycle3PWM(TIM_HandleTypeDef *htimx, float dc_a, float dc_b, float dc_c)
{
    // 计算CCR值
    uint32_t ccr_a = (uint32_t)(dc_a * __HAL_TIM_GET_AUTORELOAD(htimx));
    uint32_t ccr_b = (uint32_t)(dc_b * __HAL_TIM_GET_AUTORELOAD(htimx));
    uint32_t ccr_c = (uint32_t)(dc_c * __HAL_TIM_GET_AUTORELOAD(htimx));

    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_3, ccr_c);
}

extern "C" void foc_timer_isr(void)
{
    motor.move(target_velocity);
}
