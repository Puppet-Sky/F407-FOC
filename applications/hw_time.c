#include "main.h"

unsigned long micros(void)
{
    return TIM5->CNT;
}

void micros_delay(uint32_t us)
{
    uint32_t start = micros();

    while ((micros() - start) < us)
    {
    }
}

void delay(unsigned long ms)
{
    micros_delay((uint32_t)ms * 1000u);
}
