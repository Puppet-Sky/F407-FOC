#include "Sensor.h"
#include "foc_api.h"

#include "../BLDCMotor.h"
#include "../common/lowpass_filter.h"
#include "../drivers/BLDCDriver3PWM.h"
#include "../sensor/AS5600_I2C.h"
#include "board.h"
#include "rtthread.h"

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
            int pin_a, int pin_b, int pin_c, int pin_en)
        : sensor(sensor_config),
          driver(pin_a, pin_b, pin_c, pin_en),
          motor(pole_pairs),
          lpf(0.05f),
          target_velocity(20.0f),
          measured_velocity(0.0f)
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

    void isr()
    {
        motor.loopFOC();
        motor.move(target_angle);
    }

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
};

struct AxisGroup
{
    FocAxis *list[4];
    int count;
};

AxisGroup &axes()
{
    static FocAxis motor0(as5600_config, 7, PWM_A_PIN, PWM_B_PIN, PWM_C_PIN, PWM_ENABLE_PIN);
    static AxisGroup group = { { &motor0 }, 1 };
    return group;
}

FocAxis *axis_at(int index)
{
    AxisGroup &group = axes();
    if (index < 0 || index >= group.count)
    {
        return 0;
    }
    return group.list[index];
}

}

void foc_init(void)
{
    AxisGroup &group = axes();
    for (int i = 0; i < group.count; i++)
    {
        group.list[i]->init();
    }
}

void foc_sensor_poll(void)
{
    AxisGroup &group = axes();
    for (int i = 0; i < group.count; i++)
    {
        group.list[i]->poll();
    }
}

void foc_timer_isr(void)
{
    AxisGroup &group = axes();
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
