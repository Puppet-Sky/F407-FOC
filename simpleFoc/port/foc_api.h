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
float foc_get_target_angle(int axis);
void foc_set_target_angle(int axis, float rad);
float foc_get_measured_angle(int axis);

/* 角度 S 曲线轨迹：目标角度经规划后再送入位置环，避免阶跃起步/停止冲击 */
float foc_get_planned_angle(int axis);      /* 当前规划(参考)角度 rad */
float foc_get_planned_velocity(int axis);   /* 当前规划速度 rad/s */
/* vmax rad/s, amax rad/s^2, smooth_time s(越大越柔和), vff 速度前馈增益 0~1 */
void foc_set_trajectory(int axis, float vmax, float amax, float smooth_time, float vff);
void foc_get_trajectory(int axis, float *vmax, float *amax, float *smooth_time, float *vff);
void foc_set_trajectory_enable(int axis, int enable);   /* 0: 旁路(阶跃)，便于对比 */
int foc_get_trajectory_enable(int axis);

/* 调试用状态快照(用于日志)。angle/velocity 与目标角度同坐标系(多圈累计、已含方向修正) */
typedef struct
{
    float target;       /* 用户目标角度 rad */
    float planned;      /* 规划角度 rad */
    float planned_vel;  /* 规划速度 rad/s */
    float angle;        /* 实际角度 rad (motor.shaft_angle) */
    float velocity;     /* 实际速度 rad/s (速度环使用的滤波后值) */
    float vel_sp;       /* 位置环输出的速度给定 rad/s */
    float uq;           /* 速度环输出的 q 轴电压 V */
} foc_state_t;
void foc_get_state(int axis, foc_state_t *state);

/* 控制环参数在线整定，按名字访问，免重新烧录。
 *   ap    位置环 P            vp    速度环 P        vi   速度环 I       vd   速度环 D
 *   vtf   速度低通时间常数 s   vlim  速度限幅 rad/s   vramp 速度环输出斜率限制 V/s (0=不限)
 * foc_set_param 返回 0 成功，-1 名字未知，-2 数值非法 */
int foc_param_count(void);
const char *foc_param_name(int index);
const char *foc_param_desc(int index);
int foc_set_param(int axis, const char *name, float value);
int foc_get_param(int axis, const char *name, float *value);
float foc_get_actual_angle(int axis);

#ifdef __cplusplus
}
#endif

#endif
