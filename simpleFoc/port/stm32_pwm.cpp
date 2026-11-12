#include "stm32_pwm.h"
#include "main.h"
#include <rtthread.h>
#include "board.h"


extern TIM_HandleTypeDef htim1;

struct stm32_pwm_pin
{
    uint16_t pin;
    TIM_HandleTypeDef *timer;
    uint32_t channel;
};

static const stm32_pwm_pin pwm_map[] = {
    { GET_PIN(A, 8),  &htim1, TIM_CHANNEL_1 },
    { GET_PIN(A, 9),  &htim1, TIM_CHANNEL_2 },
    { GET_PIN(A, 10), &htim1, TIM_CHANNEL_3 },
};

static const stm32_pwm_pin *find_pin(int pin)
{
    for (unsigned int i = 0; i < sizeof(pwm_map) / sizeof(pwm_map[0]); i++)
    {
        if (pwm_map[i].pin == (uint16_t)pin)
        {
            return &pwm_map[i];
        }
    }
    return 0;
}

static int resolve_pins(int pin_a, int pin_b, int pin_c,
                        const stm32_pwm_pin **phase_a,
                        const stm32_pwm_pin **phase_b,
                        const stm32_pwm_pin **phase_c)
{
    *phase_a = find_pin(pin_a);
    *phase_b = find_pin(pin_b);
    *phase_c = find_pin(pin_c);
    if (*phase_a == 0 || *phase_b == 0 || *phase_c == 0)
    {
        return -1;
    }
    if ((*phase_a)->timer != (*phase_b)->timer || (*phase_a)->timer != (*phase_c)->timer)
    {
        return -1;
    }
    return 0;
}

long stm32_pwm_start(int pin_a, int pin_b, int pin_c)
{
    const stm32_pwm_pin *phase_a;
    const stm32_pwm_pin *phase_b;
    const stm32_pwm_pin *phase_c;

    if (resolve_pins(pin_a, pin_b, pin_c, &phase_a, &phase_b, &phase_c) != 0)
    {
        return 0;
    }

    HAL_TIM_PWM_Start(phase_a->timer, phase_a->channel);
    HAL_TIM_PWM_Start(phase_b->timer, phase_b->channel);
    HAL_TIM_PWM_Start(phase_c->timer, phase_c->channel);

    return (long)__HAL_TIM_GET_AUTORELOAD(phase_a->timer);
}

void stm32_pwm_write(int pin_a, int pin_b, int pin_c, float dc_a, float dc_b, float dc_c)
{
    const stm32_pwm_pin *phase_a;
    const stm32_pwm_pin *phase_b;
    const stm32_pwm_pin *phase_c;
    uint32_t reload;

    if (resolve_pins(pin_a, pin_b, pin_c, &phase_a, &phase_b, &phase_c) != 0)
    {
        return;
    }

    reload = __HAL_TIM_GET_AUTORELOAD(phase_a->timer);
    __HAL_TIM_SET_COMPARE(phase_a->timer, phase_a->channel, (uint32_t)(dc_a * reload));
    __HAL_TIM_SET_COMPARE(phase_b->timer, phase_b->channel, (uint32_t)(dc_b * reload));
    __HAL_TIM_SET_COMPARE(phase_c->timer, phase_c->channel, (uint32_t)(dc_c * reload));
}

void stm32_pwm_enable(int enable_pin, int enable)
{
    rt_pin_write(enable_pin, enable ? PIN_HIGH : PIN_LOW);
}
