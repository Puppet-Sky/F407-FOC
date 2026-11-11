/*
 * Copyright (c) 2006-2026, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-11-10     RT-Thread    first version
 */

#include <rtthread.h>
#include "main.h"

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

extern rt_err_t as5600_stream_start(rt_uint32_t period_us);

int main(void)
{
    int count = 1;
//    as5600_stream_start(500);

    while (count++)
    {
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}

int __board_init(void)
{
    LOG_D("__board_init");
    MX_CubeMX_Init();
    return RT_EOK;
}
INIT_BOARD_EXPORT(__board_init);
