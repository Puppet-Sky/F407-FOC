#include <rtthread.h>
#include "simpleFoc/port/foc_api.h"
#include "hw_board.h"
#include "hw_current.h"
#include "app_sensor.h"
#include "app_shell.h"
#include "jscope_rtt.h"

int main(void)
{
    jscope_init();
    foc_init();
    rt_thread_mdelay(1000);
    board_foc_timer_start();
    app_sensor_start();

    /* 电流检测验证：等若干注入完成 → 零点校准 → 周期打印（uart1 / icur 可改） */
    rt_thread_mdelay(50);
    hw_current_calibrate(256);
    hw_current_set_print(200);
    rt_kprintf("current sense: ACS712-05B @3.3V, print 200ms (icur to control)\n");

    while (1)
    {
        app_log_poll();
        // hw_current_print_poll();
        rt_thread_mdelay(10);   /* 日志节拍 10ms，实际打印周期由 flog/icur 命令设置 */
    }

    return RT_EOK;
}
