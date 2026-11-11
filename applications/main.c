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
#include "main_Cpp.h"

extern TIM_HandleTypeDef htim2;
/* TIM5 is a 32-bit counter on APB1, same clock as TIM2. PSC makes one tick 1 us. */
static void micros_counter_init(void)
{
    uint32_t timclk = HAL_RCC_GetPCLK1Freq();
    RCC_ClkInitTypeDef clkcfg;
    uint32_t flash_latency;

    HAL_RCC_GetClockConfig(&clkcfg, &flash_latency);
    if (clkcfg.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        timclk *= 2U;
    }

    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->CR1 = 0;
    TIM5->PSC = (timclk / 1000000U) - 1U;
    TIM5->ARR = 0xFFFFFFFFU;
    TIM5->CNT = 0;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR = 0;
    TIM5->CR1 = TIM_CR1_CEN;
}

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

int main(void)
{
    int count = 1;
    main_Cpp();

    return RT_EOK;
}

int __board_init(void)
{
    MX_init();
    micros_counter_init();
    return RT_EOK;
}INIT_BOARD_EXPORT(__board_init);


unsigned long micros(void)
{
    return TIM5->CNT;
}

void micros_delay(uint32_t us)
{
    uint32_t start = micros();
    while (micros() - start < us)
    {
    }
}

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  foc_timer_isr();
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}