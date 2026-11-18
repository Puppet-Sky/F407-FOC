#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>

/**
 * 在线 S 曲线轨迹生成器(梯形速度规划 + 速度滑动平均平滑)
 *
 * 作用：把"阶跃"的目标位置变成位置/速度/加速度都连续的平滑参考轨迹，
 *       供位置环跟踪。起步与停止时加速度是斜坡过渡，不会在加速度突变处产生冲击。
 *
 * 结构(两级)：
 *   第 1 级  梯形速度规划：由目标与当前规划位置之差，生成受 vmax / amax 约束的速度 v1，
 *            末段按 sqrt(2*amax*距离) 减速，最后一拍精确落在目标上。
 *   第 2 级  对 v1 做长度为 smooth_time 的滑动平均(FIR 盒式滤波)，得到输出速度。
 *            加速度突变被"抹"成长度 smooth_time 的斜坡 => 加加速度(jerk)有限。
 *            滤波器增益为 1，积分面积不变 => 终点位置精确等于目标，不会过冲。
 *
 * 特性：
 *   - 纯算法类，不依赖硬件 / RT-Thread，可在 PC 上单独测试。
 *   - 逐周期在线递推，运动中随时可改目标/改参数；改目标反向时同样平滑。
 *   - 整个过程无阶跃、无过冲；静止时 update() 几乎不占用 CPU。
 *
 * 参数(均可在线修改)：
 *   vmax        最大速度    [rad/s]
 *   amax        最大加速度  [rad/s^2]
 *   smooth_time 平滑时间    [s]    越大越柔和，起停冲击越小，总行程时间 = 梯形时间 + smooth_time
 *                                  峰值加加速度约为 amax / smooth_time
 *                                  (没有匀速段的短行程、或运动中反向时最大为 2 倍)
 *                                  有效范围受缓冲区限制：<= kMaxTaps * dt (5 kHz 下约 0.2 s)
 *
 * 线程说明：update() 在定时器中断调用；setTarget()/setLimits() 可在线程调用
 *           (均为单 float 写入，Cortex-M4 上原子)。
 */
class TrajectoryGenerator
{
public:
    /** 滑动平均缓冲区长度(2 的幂)，决定 smooth_time 上限 = kMaxTaps * dt，占用 4 KB RAM */
    static const int kMaxTaps = 1024;

    TrajectoryGenerator(float vmax = 3.0f, float amax = 10.0f, float smooth_time = 0.15f);

    /** 修改限幅(非法值会被钳到下限)，运动过程中可随时调用 */
    void setLimits(float vmax, float amax, float smooth_time);
    float vmax() const { return vmax_; }
    float amax() const { return amax_; }
    float smoothTime() const { return smooth_time_; }

    /** 使能/旁路。旁路时 update() 直接输出目标值(阶跃)，便于对比效果 */
    void enable(bool en) { enabled_ = en; }
    bool enabled() const { return enabled_; }

    /** 以给定位置为起点复位：速度/加速度清零，目标=当前位置 */
    void reset(float pos);

    void setTarget(float target) { target_ = target; }
    float target() const { return target_; }

    /**
     * 推进一个控制周期
     * @param dt 周期 [s]，应保持固定
     * @return   新的参考位置
     */
    float update(float dt);

    float position() const { return pos_; }
    float velocity() const { return vel_; }
    float acceleration() const { return acc_; }
    /** 轨迹是否已经到达目标并静止 */
    bool done() const { return done_; }

private:
    void clearFilter();

    volatile float vmax_;
    volatile float amax_;
    volatile float smooth_time_;
    volatile float target_;
    volatile bool enabled_;

    /* 第 1 级：梯形规划状态 */
    float p1_;
    float v1_;

    /* 第 2 级：滑动平均状态 */
    float buf_[kMaxTaps];
    uint32_t head_;
    uint32_t taps_;
    float sum_;
    uint32_t idle_;     // v1 连续为零(且已落在目标上)的拍数，达到 taps_ 说明滤波器已排空

    /* 输出 */
    float pos_;
    float vel_;
    float acc_;
    bool done_;
};

#endif
