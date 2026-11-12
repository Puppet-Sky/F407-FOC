#include <rtthread.h>
#include <stdlib.h>
#include <finsh.h>
#include "simpleFoc/port/foc_api.h"

static volatile int velocity_print_enable = 0;

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

static int vprint(int argc, char **argv)
{
    if (argc >= 2)
    {
        velocity_print_enable = atoi(argv[1]) ? 1 : 0;
    }
    rt_kprintf("velocity print %s\n", velocity_print_enable ? "on" : "off");
    return 0;
}
MSH_CMD_EXPORT(vprint, vprint 1 to enable speed log);

void app_log_poll(void)
{
    char target[16];
    char measured[16];

    if (!velocity_print_enable)
    {
        return;
    }
    format_fixed(target, sizeof(target), foc_get_target_velocity(0));
    format_fixed(measured, sizeof(measured), foc_get_velocity(0));
    rt_kprintf("target %s rad/s, measured %s rad/s\n", target, measured);
}
