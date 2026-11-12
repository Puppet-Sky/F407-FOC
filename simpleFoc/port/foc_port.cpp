#include "foc_api.h"

#include "../BLDCMotor.h"
#include "../common/lowpass_filter.h"
#include "../drivers/BLDCDriver3PWM.h"
#include "../sensor/AS5600_I2C.h"
#include "board.h"

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
        driver.voltage_limit = 10;
        driver.init();

        motor.linkDriver(&driver);
        motor.voltage_limit = 5;
        motor.controller = MotionControlType::velocity_openloop;
        motor.init();

        driver.enable();
    }

    void poll()
    {
        sensor.update();
        measured_velocity = lpf(sensor.getVelocity());
    }

    void isr()
    {
        motor.move(target_velocity);
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
    volatile float measured_velocity;
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
