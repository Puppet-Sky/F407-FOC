#ifndef FOC_API_H
#define FOC_API_H

#ifdef __cplusplus
extern "C" {
#endif

void foc_init(void);
void foc_sensor_poll(void);
void foc_timer_isr(void);
float foc_get_velocity(int axis);
void foc_set_target_velocity(int axis, float rad_s);
float foc_get_target_velocity(int axis);

#ifdef __cplusplus
}
#endif

#endif
