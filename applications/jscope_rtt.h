#ifndef JSCOPE_RTT_H
#define JSCOPE_RTT_H

#ifdef __cplusplus
extern "C" {
#endif

/* 在启动 TIM1 之前调用。建立 J-Link 能搜到的 RTT 控制块。 */
void jscope_init(void);

/* 在 FOC 定时中断末尾调用。关闭或缓冲满时立即返回，不阻塞中断。 */
void jscope_sample(void);

void jscope_set_enable(int enable);
int jscope_get_enable(void);
unsigned jscope_drop_count(void);

#ifdef __cplusplus
}
#endif

#endif
