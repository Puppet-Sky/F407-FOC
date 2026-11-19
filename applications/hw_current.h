/**
 * @file hw_current.h
 * @brief 相电流采样接口（TIM1 TRGO + ADC1 注入 + ACS712-05B）
 */
#ifndef HW_CURRENT_H
#define HW_CURRENT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 一拍相电流快照（线程侧读取） */
typedef struct
{
    uint16_t raw_a;     /* ADC 原始值 Ia：IN2 / PA2 */
    uint16_t raw_b;     /* ADC 原始值 Ib：IN3 / PA3 */
    float ia;           /* A 相电流 [A] */
    float ib;           /* B 相电流 [A] */
    float ic;           /* C 相估计电流 [A]，Ic = -(Ia+Ib) */
    uint32_t seq;       /* 注入完成次数；停滞则说明触发/中断异常 */
} hw_current_sample_t;

/**
 * 启动 ADC 注入转换中断（等 TIM1 TRGO）。
 * @return 0 成功，-1 HAL 失败
 */
int hw_current_start(void);

/**
 * 用静止样本校准零点偏置（ADC counts）。
 * @param samples 平均点数，0 则默认 256；电机无电流时调用
 */
void hw_current_calibrate(uint32_t samples);

/** 读取最新一拍（ISR 写入，本函数关中断拷贝） */
void hw_current_get(hw_current_sample_t *out);

/** 读取当前零点偏置（ADC counts） */
void hw_current_get_offset(uint16_t *offset_a, uint16_t *offset_b);

/**
 * 周期打印开关。
 * @param period_ms 0=关闭；建议 200~500，经 uart 控制台输出
 */
void hw_current_set_print(uint32_t period_ms);
uint32_t hw_current_get_print_period(void);

/** 由 main 循环调用，按设定周期 rt_kprintf 输出 */
void hw_current_print_poll(void);

#ifdef __cplusplus
}
#endif

#endif
