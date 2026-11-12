#include <rtthread.h>
#include "simpleFoc/port/foc_api.h"

static void sensor_entry(void *parameter)
{
    (void)parameter;
    while (1)
    {
        foc_sensor_poll();
        rt_thread_mdelay(1);
    }
}

void app_sensor_start(void)
{
    static struct rt_thread sensor_thread;
    static rt_uint8_t sensor_stack[2048];

    if (rt_thread_init(&sensor_thread, "sensor", sensor_entry, RT_NULL,
                       sensor_stack, sizeof(sensor_stack), 5, 10) == RT_EOK)
    {
        rt_thread_startup(&sensor_thread);
    }
}
