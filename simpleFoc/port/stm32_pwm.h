#ifndef STM32_PWM_H
#define STM32_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

long stm32_pwm_start(int pin_a, int pin_b, int pin_c);
void stm32_pwm_write(int pin_a, int pin_b, int pin_c, float dc_a, float dc_b, float dc_c);
void stm32_pwm_enable(int enable_pin, int enable);

#ifdef __cplusplus
}
#endif

#endif
