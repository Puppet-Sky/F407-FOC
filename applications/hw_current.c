/**
 * @file hw_current.c
 * @brief TIM1-TRGO 触发的 ADC1 注入采样 + ACS712 相电流换算
 *
 * 硬件链路：
 *   TIM1 中心对齐 PWM → TRGO(Update, 谷底) → ADC1 注入组
 *   Rank1 = IN2(PA2) → Ia
 *   Rank2 = IN3(PA3) → Ib
 *   Ic 由基尔霍夫估计：Ic = -(Ia + Ib)
 *
 * 传感器 ACS712ELCTR-05B：
 *   标称 5V / 185 mV/A；本板 Vcc=3.3V，按比例缩放灵敏度。
 *   零电流理想输出 Vcc/2，实际偏置用 hw_current_calibrate() 实测。
 *
 * 中断路径：ADC_IRQHandler → HAL_ADC_IRQHandler → 本文件 InjectedConvCpltCallback
 * 打印走 uart 控制台（rt_kprintf），勿在 ISR 内打印。
 */

#include "hw_current.h"

#include <rtthread.h>
#include <rthw.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "jscope_rtt.h"

extern ADC_HandleTypeDef hadc1;

/* ---------- ACS712-05B @ 3.3V ---------- */
#define ACS712_VREF_V          3.3f
#define ACS712_ADC_MAX         4095.0f          /* 12-bit 满量程 */
#define ACS712_SENS_5V_V_PER_A 0.185f           /* 数据手册 @5V */
/* 比例传感器：灵敏度随 Vcc 缩放 → 122.1 mV/A @3.3V */
#define ACS712_SENS_V_PER_A    (ACS712_SENS_5V_V_PER_A * (ACS712_VREF_V / 5.0f))
/* 预先合成：ADC counts → 安培 */
#define ACS712_A_PER_COUNT     ((ACS712_VREF_V / ACS712_ADC_MAX) / ACS712_SENS_V_PER_A)
#define ACS712_ZERO_DEFAULT    2048u            /* 理想零点：Vcc/2 → mid-scale */

/* ISR 与线程共享：写在注入完成回调，读时关中断拷贝 */
static volatile uint16_t g_raw_a;
static volatile uint16_t g_raw_b;
static volatile float g_ia;
static volatile float g_ib;
static volatile uint32_t g_seq;                 /* 完成计数，确认采样在跑 */

static volatile uint16_t g_offset_a = ACS712_ZERO_DEFAULT;
static volatile uint16_t g_offset_b = ACS712_ZERO_DEFAULT;

static volatile uint32_t g_print_period_ms;      /* 0 = 关闭周期打印 */
static rt_tick_t g_print_last_tick;

/** raw/offset(counts) → 有符号电流 [A] */
static float counts_to_amp(uint16_t raw, uint16_t offset)
{
    return ((float)((int32_t)raw - (int32_t)offset)) * ACS712_A_PER_COUNT;
}

/**
 * 使能 ADC1 注入组 + JEOC 中断，等待 TIM1 TRGO 触发。
 * 须在 MX_ADC1_Init 之后调用；建议先于 / 紧挨 TIM1 启动。
 */
int hw_current_start(void)
{
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

/**
 * 零点校准：对连续 n 次注入结果取平均作为 offset。
 * 调用时电机应尽量无电流（驱动关闭或 Iq=0），否则会把负载电流吃进偏置。
 */
void hw_current_calibrate(uint32_t samples)
{
    uint64_t sum_a = 0;
    uint64_t sum_b = 0;
    uint32_t n = samples ? samples : 256u;
    uint32_t got = 0;
    uint32_t last_seq = g_seq;
    uint32_t guard = 0;

    /* 按 seq 变化取新样本；guard 防止 ADC 未触发时死等 */
    while (got < n && guard < (n * 20u + 1000u))
    {
        uint32_t seq = g_seq;
        if (seq != last_seq)
        {
            last_seq = seq;
            sum_a += g_raw_a;
            sum_b += g_raw_b;
            got++;
        }
        else
        {
            rt_thread_mdelay(1);
            guard++;
        }
    }

    if (got > 0u)
    {
        g_offset_a = (uint16_t)(sum_a / got);
        g_offset_b = (uint16_t)(sum_b / got);
    }
}

/** 原子读取最新一拍；Ic 在线程侧由 Ia+Ib 推出 */
void hw_current_get(hw_current_sample_t *out)
{
    uint16_t raw_a;
    uint16_t raw_b;
    float ia;
    float ib;
    uint32_t seq;

    if (!out)
    {
        return;
    }

    // rt_base_t level = rt_hw_interrupt_disable();
    raw_a = g_raw_a;
    raw_b = g_raw_b;
    ia = g_ia;
    ib = g_ib;
    seq = g_seq;
    // rt_hw_interrupt_enable(level);

    out->raw_a = raw_a;
    out->raw_b = raw_b;
    out->ia = ia;
    out->ib = ib;
    out->ic = -(ia + ib);   /* 两电阻采样，第三相合成 */
    out->seq = seq;
}

void hw_current_get_offset(uint16_t *offset_a, uint16_t *offset_b)
{
    if (offset_a)
    {
        *offset_a = g_offset_a;
    }
    if (offset_b)
    {
        *offset_b = g_offset_b;
    }
}

void hw_current_set_print(uint32_t period_ms)
{
    g_print_period_ms = period_ms;
    g_print_last_tick = rt_tick_get();
}

uint32_t hw_current_get_print_period(void)
{
    return g_print_period_ms;
}

/**
 * 主循环轮询：按 g_print_period_ms 输出 raw / Ia Ib Ic / seq / offset。
 * 使用整数拼接浮点，避免 printf 浮点依赖。
 */
void hw_current_print_poll(void)
{
    hw_current_sample_t s;
    char sa[16];
    char sb[16];
    char sc[16];
    int neg;
    float mag;
    int ip;
    int fp;
    rt_tick_t now;
    uint32_t period = g_print_period_ms;

    if (period == 0u)
    {
        return;
    }

    now = rt_tick_get();
    if ((now - g_print_last_tick) < rt_tick_from_millisecond(period))
    {
        return;
    }
    g_print_last_tick = now;

    hw_current_get(&s);

    neg = s.ia < 0.0f;
    mag = neg ? -s.ia : s.ia;
    ip = (int)mag;
    fp = (int)((mag - (float)ip) * 1000.0f + 0.5f);
    if (fp >= 1000) { fp -= 1000; ip += 1; }
    rt_snprintf(sa, sizeof(sa), "%s%d.%03d", neg ? "-" : "", ip, fp);

    neg = s.ib < 0.0f;
    mag = neg ? -s.ib : s.ib;
    ip = (int)mag;
    fp = (int)((mag - (float)ip) * 1000.0f + 0.5f);
    if (fp >= 1000) { fp -= 1000; ip += 1; }
    rt_snprintf(sb, sizeof(sb), "%s%d.%03d", neg ? "-" : "", ip, fp);

    neg = s.ic < 0.0f;
    mag = neg ? -s.ic : s.ic;
    ip = (int)mag;
    fp = (int)((mag - (float)ip) * 1000.0f + 0.5f);
    if (fp >= 1000) { fp -= 1000; ip += 1; }
    rt_snprintf(sc, sizeof(sc), "%s%d.%03d", neg ? "-" : "", ip, fp);

    rt_kprintf("I: rawA=%u rawB=%u Ia=%sA Ib=%sA Ic=%sA seq=%u off=%u/%u\n",
               (unsigned)s.raw_a, (unsigned)s.raw_b,
               sa, sb, sc, (unsigned)s.seq,
               (unsigned)g_offset_a, (unsigned)g_offset_b);
}

/**
 * ADC 注入序列完成回调（JEOC）。
 * TIM1 TRGO 触发后硬件自动采完 Rank1/2，此处只读 JDR 并换算，保持尽量短。
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint16_t raw_a;
    uint16_t raw_b;

    if (hadc->Instance != ADC1)
    {
        return;
    }

    raw_a = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1); /* PA2 / Ia */
    raw_b = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2); /* PA3 / Ib */

    g_raw_a = raw_a;
    g_raw_b = raw_b;
    g_ia = counts_to_amp(raw_a, g_offset_a);
    g_ib = counts_to_amp(raw_b, g_offset_b);
    g_seq++;
    jscope_sample();
}
