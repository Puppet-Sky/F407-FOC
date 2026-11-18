#include <rtthread.h>
#include "simpleFoc/port/foc_api.h"
#include "hw_board.h"
#include "app_sensor.h"
#include "app_shell.h"

int main(void)
{
    foc_init();
    rt_thread_mdelay(1000);
    board_foc_timer_start();
    app_sensor_start();

    while (1)
    {
        app_log_poll();
        rt_thread_mdelay(10);   /* 日志节拍 10ms，实际打印周期由 flog 命令设置 */
    }

    return RT_EOK;
}
