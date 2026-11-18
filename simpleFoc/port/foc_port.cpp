#include "Sensor.h"
#include "foc_api.h"

#include "../BLDCMotor.h"
#include "../common/lowpass_filter.h"
#include "../drivers/BLDCDriver3PWM.h"
#include "../sensor/AS5600_I2C.h"
#include "board.h"
#include "rtdef.h"
#include "rtthread.h"
#include <stdint.h>

namespace
{

const AS5600_I2CConfig_s as5600_config = {
    0x36,
    12,
    0x0C,
    11
};

class FocAxis
{
public:
    FocAxis(const AS5600_I2CConfig_s &sensor_config, int pole_pairs,
            int pin_a, int pin_b, int pin_c, int pin_en, int axis_index)
        : sensor(sensor_config),
          driver(pin_a, pin_b, pin_c, pin_en),
          motor(pole_pairs),
          lpf(0.05f),
          target_velocity(20.0f),
          measured_velocity(0.0f),
          axis_index(axis_index),
          sample_div(0)
    {
    }

    void init()
    {
        sensor.init();
        sensor.min_elapsed_time = 0.001f;

        driver.voltage_power_supply = 12;
        driver.voltage_limit = 6;
        driver.init();
        motor.linkDriver(&driver);
        motor.linkSensor(&sensor);

        motor.controller = MotionControlType::angle;

        /* PID 速度环*/
        motor.PID_velocity.P = 0.1f;
        motor.PID_velocity.I = 5.0f;
        motor.PID_velocity.D = 0.0f;
        motor.PID_velocity.output_ramp = 2000.0f;

        /* 速度环低通滤波器，时间常数设置为8ms*/
        motor.LPF_velocity.Tf = 0.008f;

        /* 位置环*/
        motor.P_angle.P = 50.0f;

        motor.voltage_limit = 6;
        motor.velocity_limit = 60.0f;
        motor.init();

        motor.sensor_direction = Direction::UNKNOWN;
        motor.initFOC();

        driver.enable();
        rt_kprintf("\nzero electric angle: %d.%03d\n", (int)motor.zero_electric_angle, (int)((motor.zero_electric_angle - (int)motor.zero_electric_angle) * 1000));

        delay(1000);
    }

    void poll()
    {
        sensor.update();
        measured_velocity = lpf(sensor.getVelocity());
    }

    void isr();

    float velocity() const
    {
        return measured_velocity;
    }

    float target() const
    {
        return target_velocity;
    }

    void set_target(float rad_s)
    {
        target_velocity = rad_s;
    }

private:
    AS5600_I2C sensor;
    BLDCDriver3PWM driver;
    BLDCMotor motor;
    LowPassFilter lpf;
    volatile float target_velocity;
    volatile float target_current;
    volatile float target_angle = 1.0f;
    volatile float measured_current = 0.0f;
    volatile float measured_angle = 0.0f;
    volatile float measured_velocity = 0.0f;
    uint32_t axis_index;                    // 电机索引
    uint32_t sample_div;                    // TIM中断采样分频
};

struct AxisGroup
{
    FocAxis *list[4];
    int count;
    struct rt_event event;
};

/* 文件作用域静态对象：构造发生在 INIT_COMPONENT 阶段（晚于 INIT_BOARD），
 * 运行期访问不再经过函数内静态变量的初始化守卫，中断中可直接使用。 */
FocAxis motor0(as5600_config, 7, PWM_A_PIN, PWM_B_PIN, PWM_C_PIN, PWM_ENABLE_PIN, 0);
AxisGroup axis_group = { { &motor0 }, 1, {} };

void FocAxis::isr() //目前中断设置的为5Khz, 编码器采样频率为5/2KHZ
{
    motor.loopFOC();
    motor.move(target_angle);

    if(++sample_div >= 2)
    {
        sample_div = 0;
        rt_event_send(&axis_group.event, (1u << axis_index));
    }
}

FocAxis *axis_at(int index)
{
    if (index < 0 || index >= axis_group.count)
    {
        return 0;
    }
    return axis_group.list[index];
}

}

void foc_init(void)
{
    AxisGroup &group = axis_group;
    rt_event_init(&group.event, "event", RT_IPC_FLAG_FIFO);
    for (int i = 0; i < group.count; i++)
    {
        group.list[i]->init();
    }
}

void foc_sensor_poll(void)
{
    AxisGroup &group = axis_group;
    uint32_t received;
    if(rt_event_recv(&group.event, 0xFFFFFFFF,
                      RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                      RT_WAITING_FOREVER, &received) == RT_EOK)
    {
        for (int i = 0; i < group.count; i++)
        {
            if (received & (1u << i))
                group.list[i]->poll();
        }
    }
}

void foc_timer_isr(void)
{
    AxisGroup &group = axis_group;
    for (int i = 0; i < group.count; i++)
    {
        group.list[i]->isr();
    }
}

float foc_get_velocity(int axis)
{
    FocAxis *item = axis_at(axis);
    return item ? item->velocity() : 0.0f;
}

void foc_set_target_velocity(int axis, float rad_s)
{
    FocAxis *item = axis_at(axis);
    if (item)
    {
        item->set_target(rad_s);
    }
}

float foc_get_target_velocity(int axis)
{
    FocAxis *item = axis_at(axis);
    return item ? item->target() : 0.0f;
}
