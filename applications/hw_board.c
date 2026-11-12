#include <rtthread.h>
#include "main.h"
#include "simpleFoc/port/foc_api.h"

extern TIM_HandleTypeDef htim2;

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

int __board_init(void)
{
    MX_init();
    micros_counter_init();
    return RT_EOK;
}
INIT_BOARD_EXPORT(__board_init);

void board_foc_timer_start(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
    foc_timer_isr();
}
