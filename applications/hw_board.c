#include <rtthread.h>
#include "main.h"
#include "simpleFoc/port/foc_api.h"
#include "jscope_rtt.h"
#include "hw_current.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_gpio.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

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
    /* 先使能注入组（等 TRGO），再开 TIM1，保证首拍起就能采到电流 */
    if (hw_current_start() != 0)
    {
        rt_kprintf("hw_current_start failed\n");
    }
    HAL_TIM_Base_Start_IT(&htim1);
}

/* TIM1 Update：loopFOC / J-Scope；与 ADC TRGO 同源，频率由 RCR 决定 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
    foc_timer_isr();
    // jscope_sample();
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
    // foc_timer_isr();
}

/* 注入 JEOC → HAL → HAL_ADCEx_InjectedConvCpltCallback(hw_current.c) */
void ADC_IRQHandler(void)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    HAL_ADC_IRQHandler(&hadc1);
}
