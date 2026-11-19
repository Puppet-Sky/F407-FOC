#include "Sensor.h"
#include "foc_api.h"

#include "../BLDCMotor.h"
#include "../common/lowpass_filter.h"
#include "trajectory.h"
#include "../drivers/BLDCDriver3PWM.h"
#include "../sensor/AS5600_I2C.h"
#include "board.h"
#include "rtdef.h"
#include "rtthread.h"
#include <stdint.h>
#include <string.h>

namespace
{

/* 可在线整定的参数表(名字与 FocAxis::set_param/get_param 一一对应) */
struct ParamDesc
{
    const char *name;
    const char *desc;
};

const ParamDesc kParamTable[] = {
    { "ap",    "angle loop P [rad/s per rad]" },
    { "vp",    "velocity loop P [V per rad/s]" },
    { "vi",    "velocity loop I" },
    { "vd",    "velocity loop D" },
    { "vtf",   "velocity LPF time constant [s]" },
    { "vlim",  "velocity limit [rad/s]" },
    { "vramp", "velocity PID output ramp [V/s], 0=off" },
};
const int kParamCount = (int)(sizeof(kParamTable) / sizeof(kParamTable[0]));

const AS5600_I2CConfig_s as5600_config = {
    0x36,
    12,
    0x0C,
    11
};

/* 定时器中断频率(TIM1: 20kHz PWM, 重复计数 7 => 5kHz)，loopFOC 按此频率运行 */
const float kIsrFreqHz = 5000.0f;
/* 位置环/速度环(motor.move)与轨迹规划的分频：5kHz / 5 = 1kHz。
 * 分频由 FocAxis::isr 自己做，motor.motion_downsample 保持 0，
 * 保证 traj.update() 与 move() 同频、dt 一致 */
const uint32_t kMotionDiv = 1;
const float kMotionDt = (float)kMotionDiv / kIsrFreqHz;

/* 角度轨迹(S 曲线)默认参数，可在运行时通过 shell 命令 traj 调整 */
const float kTrajVmax = 3.0f;       // rad/s    最大速度，需小于 motor.velocity_limit 留出位置环修正余量
const float kTrajAmax = 10.0f;      // rad/s^2  最大加速度
const float kTrajSmooth = 0.15f;    // s        平滑时间，峰值加加速度约 amax/smooth
const float kTrajVff = 1.0f;        // 速度前馈增益(0~1)

class FocAxis
{
public:
    FocAxis(const AS5600_I2CConfig_s &sensor_config, int pole_pairs,
            int pin_a, int pin_b, int pin_c, int pin_en, int axis_index)
        : sensor(sensor_config),
          driver(pin_a, pin_b, pin_c, pin_en),
          motor(pole_pairs),
          lpf(0.05f),
          traj(kTrajVmax, kTrajAmax, kTrajSmooth),
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

        // motor.controller = MotionControlType::velocity_openloop;
       motor.controller = MotionControlType::velocity;

        /* PID 速度环*/
        motor.PID_velocity.P = 0.04f;
        motor.PID_velocity.I = 0.1f;
        motor.PID_velocity.D = 0.0f;
        motor.PID_velocity.output_ramp = 2000.0f;

        /* 速度环低通滤波器，时间常数设置为8ms*/
        motor.LPF_velocity.Tf = 0.003f;

        /* 位置环*/
        motor.P_angle.P = 10.0f;

        motor.voltage_limit = 6;
        motor.velocity_limit = 10.0f;
        motor.motion_downsample = 0;    // 分频由 FocAxis::isr 中的 kMotionDiv 负责
        motor.init();

        motor.sensor_direction = Direction::UNKNOWN;
        motor.foc_modulation = FOCModulationType::SinePWM;
        motor.initFOC();

        driver.enable();
        rt_kprintf("\nzero electric angle: %d.%03d\n", (int)motor.zero_electric_angle, (int)((motor.zero_electric_angle - (int)motor.zero_electric_angle) * 1000));

        delay(1000);

        /* 轨迹以电机当前实际位置为起点，目标=当前位置：上电保持不动，避免起步阶跃 */
        sensor.update();
        traj.reset(motor.shaftAngle());
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

    /* 最终目标角度(用户设定值)，经 S 曲线规划后才送入位置环 */
    float get_target_angle() const
    {
        return traj.target();
    }

    void set_target_angle(float rad)
    {
        traj.setTarget(rad);
    }

    /* 当前规划(参考)角度 / 速度，即实际送给位置环的给定值 */
    float get_planned_angle() const
    {
        return traj.position();
    }

    float get_planned_velocity() const
    {
        return traj.velocity();
    }

    /* 轨迹参数在线调整：vmax[rad/s] amax[rad/s^2] smooth_time[s] vff 速度前馈增益 */
    void set_trajectory(float vmax, float amax, float smooth_time, float vff)
    {
        traj.setLimits(vmax, amax, smooth_time);
        vff_gain = vff < 0.0f ? 0.0f : (vff > 1.0f ? 1.0f : vff);
    }

    void get_trajectory(float *vmax, float *amax, float *smooth_time, float *vff) const
    {
        *vmax = traj.vmax();
        *amax = traj.amax();
        *smooth_time = traj.smoothTime();
        *vff = vff_gain;
    }

    void set_trajectory_enable(bool en)
    {
        traj.enable(en);
    }

    bool get_trajectory_enable() const
    {
        return traj.enabled();
    }

    /* 状态快照，供日志使用(仅读取，不改变控制状态) */
    void get_state(foc_state_t *s) const
    {
        s->target = traj.target();
        s->planned = traj.position();
        s->planned_vel = traj.velocity();
        s->angle = motor.shaft_angle;
        s->velocity = motor.shaft_velocity;
        s->vel_sp = motor.shaft_velocity_sp;
        s->uq = motor.voltage.q;
        s->ua = motor.Ua;
        s->ub = motor.Ub;
        s->uc = motor.Uc;
    }

    /* 控制环参数在线整定。返回 0 成功，-1 名字未知，-2 数值非法 */
    int set_param(const char *name, float v)
    {
        if (strcmp(name, "ap") == 0)        { if (v < 0.0f) return -2; motor.P_angle.P = v; }
        else if (strcmp(name, "vp") == 0)   { if (v < 0.0f) return -2; motor.PID_velocity.P = v; }
        else if (strcmp(name, "vi") == 0)   { if (v < 0.0f) return -2; motor.PID_velocity.I = v; }
        else if (strcmp(name, "vd") == 0)   { if (v < 0.0f) return -2; motor.PID_velocity.D = v; }
        else if (strcmp(name, "vtf") == 0)  { if (v < 0.0f) return -2; motor.LPF_velocity.Tf = v; }
        else if (strcmp(name, "vlim") == 0) { if (v <= 0.0f) return -2; motor.velocity_limit = v; motor.P_angle.limit = v; }
        else if (strcmp(name, "vramp") == 0){ if (v < 0.0f) return -2; motor.PID_velocity.output_ramp = v; }
        else return -1;
        return 0;
    }

    int get_param(const char *name, float *v) const
    {
        if (strcmp(name, "ap") == 0)         *v = motor.P_angle.P;
        else if (strcmp(name, "vp") == 0)    *v = motor.PID_velocity.P;
        else if (strcmp(name, "vi") == 0)    *v = motor.PID_velocity.I;
        else if (strcmp(name, "vd") == 0)    *v = motor.PID_velocity.D;
        else if (strcmp(name, "vtf") == 0)   *v = motor.LPF_velocity.Tf;
        else if (strcmp(name, "vlim") == 0)  *v = motor.velocity_limit;
        else if (strcmp(name, "vramp") == 0) *v = motor.PID_velocity.output_ramp;
        else return -1;
        return 0;
    }

    float get_measured_angle() const
    {
        return measured_angle;
    }

    float get_actual_angle()
    {
        return sensor.getMechanicalAngle();
    }

private:
    AS5600_I2C sensor;
    BLDCDriver3PWM driver;
    BLDCMotor motor;
    LowPassFilter lpf;
    TrajectoryGenerator traj;               // 角度 S 曲线轨迹生成器
    volatile float vff_gain = kTrajVff;     // 速度前馈增益
    volatile float target_velocity;
    volatile float target_current;
    volatile float measured_current = 0.0f;
    volatile float measured_angle = 0.0f;
    volatile float measured_velocity = 0.0f;
    uint32_t axis_index;                    // 电机索引
    uint32_t sample_div;                    // TIM中断采样分频
    uint32_t motion_div = 0;                // 位置环/轨迹规划分频计数
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

    /* 外环按 kMotionDiv 分频运行；move 入参随 motor.controller 切换：
     *   angle*: 轨迹规划角度 + 速度前馈
     *   velocity*: 速度给定 target_velocity
     */
    if(++motion_div >= kMotionDiv)
    {
        motion_div = 0;
        switch (motor.controller)
        {
        case MotionControlType::angle:
        case MotionControlType::angle_openloop:
        {
            const float ref_angle = traj.update(kMotionDt);
            motor.feed_forward_velocity = vff_gain * traj.velocity();
            motor.move(ref_angle);
            break;
        }
        case MotionControlType::velocity:
        case MotionControlType::velocity_openloop:
            motor.move(target_velocity);
            break;
        default:
            motor.move(motor.target);
            break;
        }
    }

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

float foc_get_target_angle(int axis)
{
    FocAxis *item = axis_at(axis);
    return item ? item->get_target_angle() : 0.0f;
}

void foc_set_target_angle(int axis, float rad)
{
    FocAxis *item = axis_at(axis);
    if (item)
    {
        item->set_target_angle(rad);
    }
}

float foc_get_planned_angle(int axis)
{
    FocAxis *item = axis_at(axis);
    return item ? item->get_planned_angle() : 0.0f;
}

float foc_get_planned_velocity(int axis)
{
    FocAxis *item = axis_at(axis);
    return item ? item->get_planned_velocity() : 0.0f;
}

void foc_set_trajectory(int axis, float vmax, float amax, float smooth_time, float vff)
{
    FocAxis *item = axis_at(axis);
    if (item)
    {
        item->set_trajectory(vmax, amax, smooth_time, vff);
    }
}

void foc_get_trajectory(int axis, float *vmax, float *amax, float *smooth_time, float *vff)
{
    FocAxis *item = axis_at(axis);
    if (item)
    {
        item->get_trajectory(vmax, amax, smooth_time, vff);
    }
}

void foc_set_trajectory_enable(int axis, int enable)
{
    FocAxis *item = axis_at(axis);
    if (item)
    {
        item->set_trajectory_enable(enable != 0);
    }
}

int foc_get_trajectory_enable(int axis)
{
    FocAxis *item = axis_at(axis);
    return item ? (item->get_trajectory_enable() ? 1 : 0) : 0;
}

void foc_get_state(int axis, foc_state_t *state)
{
    FocAxis *item = axis_at(axis);
    if (item)
    {
        item->get_state(state);
    }
}

int foc_param_count(void)
{
    return kParamCount;
}

const char *foc_param_name(int index)
{
    return (index >= 0 && index < kParamCount) ? kParamTable[index].name : 0;
}

const char *foc_param_desc(int index)
{
    return (index >= 0 && index < kParamCount) ? kParamTable[index].desc : 0;
}

int foc_set_param(int axis, const char *name, float value)
{
    FocAxis *item = axis_at(axis);
    return item ? item->set_param(name, value) : -1;
}

int foc_get_param(int axis, const char *name, float *value)
{
    FocAxis *item = axis_at(axis);
    return item ? item->get_param(name, value) : -1;
}

float foc_get_measured_angle(int axis)
{
    FocAxis *item = axis_at(axis);
    return item ? item->get_measured_angle() : 0.0f;
}

float foc_get_actual_angle(int axis)
{
    FocAxis *item = axis_at(axis);
    return item ? item->get_actual_angle() : 0.0f;
}
