#ifndef MAIN_CPP_H
#define MAIN_CPP_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void main_Cpp(void);
void foc_timer_isr(void);

extern TIM_HandleTypeDef htim1;
void _writeDutyCycle3PWM(TIM_HandleTypeDef *htimx, float dc_a, float dc_b, float dc_c);

#ifdef __cplusplus
}
#endif

#endif // MAIN_CPP_H