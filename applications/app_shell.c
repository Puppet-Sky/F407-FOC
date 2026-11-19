#include <rtthread.h>
#include <stdlib.h>
#include <string.h>
#include <finsh.h>
#include "simpleFoc/port/foc_api.h"
#include "jscope_rtt.h"
#include "hw_current.h"

/* 日志：flog on [period_ms] / flog off，周期由 main 循环(10ms 节拍)驱动 */
#define LOG_PERIOD_MIN_MS 10
static volatile int log_enable = 0;
static volatile rt_uint32_t log_period_ms = 50;
static rt_tick_t log_last_tick = 0;

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

static int speed(int argc, char **argv)
{
    char target[16];
    char measured[16];

    if (argc >= 2)
    {
        foc_set_target_velocity(0, (float)atof(argv[1]));
    }
    format_fixed(target, sizeof(target), foc_get_target_velocity(0));
    format_fixed(measured, sizeof(measured), foc_get_velocity(0));
    rt_kprintf("target %s rad/s, measured %s rad/s\n", target, measured);
    return 0;
}
MSH_CMD_EXPORT(speed, set or show openloop speed in rad/s);

static int angle(int argc, char **argv)
{
    char target[16];
    char planned[16];
    char rad[16];

    if (argc >= 2)
    {
        foc_set_target_angle(0, (float)atof(argv[1]));
    }
    format_fixed(target, sizeof(target), foc_get_target_angle(0));
    format_fixed(planned, sizeof(planned), foc_get_planned_angle(0));
    format_fixed(rad, sizeof(rad), foc_get_actual_angle(0));
    rt_kprintf("target %s rad, planned %s rad, actual angle %s rad\n", target, planned, rad);
    return 0;
}
MSH_CMD_EXPORT(angle, set or show shaft angle in rad);

/* traj                          显示当前轨迹参数
 * traj on|off                   开/关 S 曲线平滑(off 为阶跃，便于对比)
 * traj <vmax> <amax> <smooth_ms> [vff]
 *                               vmax rad/s, amax rad/s^2, smooth_ms 平滑时间(毫秒), vff 速度前馈 0~1 */
static int traj(int argc, char **argv)
{
    float vmax, amax, smooth, vff;
    char s_vmax[16], s_amax[16], s_smooth[16], s_vff[16];

    foc_get_trajectory(0, &vmax, &amax, &smooth, &vff);

    if (argc == 2 && (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "off") == 0))
    {
        foc_set_trajectory_enable(0, argv[1][1] == 'n');
    }
    else if (argc >= 4)
    {
        vmax = (float)atof(argv[1]);
        amax = (float)atof(argv[2]);
        smooth = (float)atof(argv[3]) * 0.001f;
        if (argc >= 5)
        {
            vff = (float)atof(argv[4]);
        }
        foc_set_trajectory(0, vmax, amax, smooth, vff);
        foc_get_trajectory(0, &vmax, &amax, &smooth, &vff);
    }
    else if (argc != 1)
    {
        rt_kprintf("usage: traj [on|off] | traj <vmax rad/s> <amax rad/s^2> <smooth ms> [vff 0~1]\n");
    }

    format_fixed(s_vmax, sizeof(s_vmax), vmax);
    format_fixed(s_amax, sizeof(s_amax), amax);
    format_fixed(s_smooth, sizeof(s_smooth), smooth * 1000.0f);
    format_fixed(s_vff, sizeof(s_vff), vff);
    rt_kprintf("traj %s: vmax %s rad/s, amax %s rad/s^2, smooth %s ms, vff %s\n",
               foc_get_trajectory_enable(0) ? "on" : "off", s_vmax, s_amax, s_smooth, s_vff);
    return 0;
}
MSH_CMD_EXPORT(traj, show or set S-curve trajectory params);

/* 参数用 4 位小数显示(vd、vtf 等数值较小) */
static void format_param(char *buf, rt_size_t len, float value)
{
    int negative = value < 0.0f;
    float magnitude = negative ? -value : value;
    int integer = (int)magnitude;
    int fraction = (int)((magnitude - (float)integer) * 10000.0f + 0.5f);

    if (fraction >= 10000)
    {
        fraction -= 10000;
        integer += 1;
    }
    rt_snprintf(buf, len, "%s%d.%04d", negative ? "-" : "", integer, fraction);
}

/* tune                 列出所有可整定参数及当前值
 * tune <name>          查看单个参数
 * tune <name> <value>  修改参数，立即生效(不保存，重启后恢复代码中的默认值) */
static int tune(int argc, char **argv)
{
    char buf[20];
    float value;
    int i;

    if (argc >= 3)
    {
        int ret = foc_set_param(0, argv[1], (float)atof(argv[2]));
        if (ret == -1)
        {
            rt_kprintf("unknown param '%s', type 'tune' to list\n", argv[1]);
            return -1;
        }
        if (ret == -2)
        {
            rt_kprintf("invalid value for '%s'\n", argv[1]);
            return -1;
        }
    }

    for (i = 0; i < foc_param_count(); i++)
    {
        const char *name = foc_param_name(i);
        if (argc >= 2 && strcmp(argv[1], name) != 0)
        {
            continue;
        }
        foc_get_param(0, name, &value);
        format_param(buf, sizeof(buf), value);
        rt_kprintf("%-6s = %-10s %s\n", name, buf, foc_param_desc(i));
    }
    return 0;
}
MSH_CMD_EXPORT(tune, list or set control loop params: tune [name] [value]);

/* flog                    显示日志状态
 * flog on [period_ms]     开启日志，周期最小 10ms(115200 波特率下建议 >= 20ms)
 * flog <period_ms>        等同于 flog on <period_ms>
 * flog off                关闭日志
 * 输出为 CSV：ms,target,planned,angle,vel,vel_sp,uq
 *   target/planned/angle 单位 rad，vel/vel_sp 单位 rad/s，uq 单位 V
 *   angle 是多圈累计角度(与 target 同坐标系)，可直接对比 */
static void log_print_header(void)
{
    rt_kprintf("ms,target,planned,angle,vel,vel_sp,uq\n");
}

static int flog(int argc, char **argv)
{
    int period = 0;

    if (argc >= 2)
    {
        if (strcmp(argv[1], "off") == 0)
        {
            log_enable = 0;
        }
        else
        {
            if (strcmp(argv[1], "on") == 0)
            {
                period = (argc >= 3) ? atoi(argv[2]) : 0;
            }
            else
            {
                period = atoi(argv[1]);
                if (period <= 0)
                {
                    rt_kprintf("usage: flog [on [period_ms] | off | period_ms]\n");
                    return -1;
                }
            }
            if (period > 0)
            {
                log_period_ms = (rt_uint32_t)(period < LOG_PERIOD_MIN_MS ? LOG_PERIOD_MIN_MS : period);
            }
            log_last_tick = rt_tick_get();
            log_print_header();
            log_enable = 1;
        }
    }
    rt_kprintf("log %s, period %d ms\n", log_enable ? "on" : "off", (int)log_period_ms);
    return 0;
}
MSH_CMD_EXPORT(flog, flog [on [period_ms] | off | period_ms] - csv state log);

/* jscope            显示 J-Scope RTT 采样状态
 * jscope on|off     开/关 TIM1 中断里的高频采样（默认开）
 * 曲线顺序见 jscope_rtt.c 通道名注释。drops 为缓冲满后丢掉的包数 */
static int jscope(int argc, char **argv)
{
    if (argc >= 2)
    {
        if (strcmp(argv[1], "on") == 0)
        {
            jscope_set_enable(1);
        }
        else if (strcmp(argv[1], "off") == 0)
        {
            jscope_set_enable(0);
        }
        else
        {
            rt_kprintf("usage: jscope [on|off]\n");
            return -1;
        }
    }
    rt_kprintf("jscope %s, drops %u\n", jscope_get_enable() ? "on" : "off", jscope_drop_count());
    rt_kprintf("JScope_t4f4f4f4f4f4f4f4f4f4f4: us,target,planned,planned_vel,angle,velocity,vel_sp,uq,ua,ub,uc\n");
    return 0;
}
MSH_CMD_EXPORT(jscope, jscope [on|off] - J-Scope RTT sample at TIM1 rate);

/* icur                 打印一次 Ia/Ib/Ic 与原始 ADC
 * icur cal [n]         零点校准（默认 256 点，电机尽量无电流）
 * icur on [period_ms]  周期打印（默认 200ms）
 * icur off             关闭周期打印 */
static int icur(int argc, char **argv)
{
    hw_current_sample_t s;
    char sa[16], sb[16], sc[16];

    if (argc >= 2)
    {
        if (strcmp(argv[1], "cal") == 0)
        {
            uint16_t off_a;
            uint16_t off_b;
            uint32_t n = (argc >= 3) ? (uint32_t)atoi(argv[2]) : 256u;
            hw_current_calibrate(n);
            hw_current_get_offset(&off_a, &off_b);
            rt_kprintf("cal done: offA=%u offB=%u (n=%u)\n",
                       (unsigned)off_a, (unsigned)off_b, (unsigned)n);
            return 0;
        }
        if (strcmp(argv[1], "on") == 0)
        {
            uint32_t ms = (argc >= 3) ? (uint32_t)atoi(argv[2]) : 200u;
            if (ms < 20u)
            {
                ms = 20u;
            }
            hw_current_set_print(ms);
            rt_kprintf("icur print on, period %u ms\n", (unsigned)ms);
            return 0;
        }
        if (strcmp(argv[1], "off") == 0)
        {
            hw_current_set_print(0);
            rt_kprintf("icur print off\n");
            return 0;
        }
        rt_kprintf("usage: icur | icur cal [n] | icur on [ms] | icur off\n");
        return -1;
    }

    hw_current_get(&s);
    format_fixed(sa, sizeof(sa), s.ia);
    format_fixed(sb, sizeof(sb), s.ib);
    format_fixed(sc, sizeof(sc), s.ic);
    rt_kprintf("rawA=%u rawB=%u Ia=%sA Ib=%sA Ic=%sA seq=%u print=%ums\n",
               (unsigned)s.raw_a, (unsigned)s.raw_b,
               sa, sb, sc, (unsigned)s.seq,
               (unsigned)hw_current_get_print_period());
    return 0;
}
MSH_CMD_EXPORT(icur, icur | cal | on [ms] | off - phase current);

void app_log_poll(void)
{
    char s_target[16], s_planned[16], s_angle[16], s_vel[16], s_vsp[16], s_uq[16];
    foc_state_t st;
    rt_tick_t now;
    rt_tick_t period_ticks;
    rt_uint32_t ms;

    if (!log_enable)
    {
        return;
    }

    now = rt_tick_get();
    period_ticks = (log_period_ms * RT_TICK_PER_SECOND) / 1000;
    if (period_ticks == 0)
    {
        period_ticks = 1;
    }
    if ((rt_tick_t)(now - log_last_tick) < period_ticks)
    {
        return;
    }
    log_last_tick = now;

    foc_get_state(0, &st);
    format_fixed(s_target, sizeof(s_target), st.target);
    format_fixed(s_planned, sizeof(s_planned), st.planned);
    format_fixed(s_angle, sizeof(s_angle), st.angle);
    format_fixed(s_vel, sizeof(s_vel), st.velocity);
    format_fixed(s_vsp, sizeof(s_vsp), st.vel_sp);
    format_fixed(s_uq, sizeof(s_uq), st.uq);

    /* 先除后乘，避免 tick*1000 在长时间运行后溢出 */
    ms = (now / RT_TICK_PER_SECOND) * 1000 + (now % RT_TICK_PER_SECOND) * 1000 / RT_TICK_PER_SECOND;
    rt_kprintf("%d,%s,%s,%s,%s,%s,%s\n", (int)ms, s_target, s_planned, s_angle, s_vel, s_vsp, s_uq);
}
