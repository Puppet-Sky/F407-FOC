#include "trajectory.h"
#include <math.h>
#include <string.h>

namespace
{
const float kMinVmax = 1e-3f;
const float kMinAmax = 1e-2f;
const float kMinSmooth = 0.0f;

inline float sign(float x) { return x < 0.0f ? -1.0f : 1.0f; }
inline float absf(float x) { return x < 0.0f ? -x : x; }
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
}

TrajectoryGenerator::TrajectoryGenerator(float vmax, float amax, float smooth_time)
    : vmax_(vmax), amax_(amax), smooth_time_(smooth_time),
      target_(0.0f), enabled_(true),
      p1_(0.0f), v1_(0.0f),
      head_(0), taps_(1), sum_(0.0f), idle_(0),
      pos_(0.0f), vel_(0.0f), acc_(0.0f), done_(true)
{
    memset(buf_, 0, sizeof(buf_));
    setLimits(vmax, amax, smooth_time);
}

void TrajectoryGenerator::setLimits(float vmax, float amax, float smooth_time)
{
    vmax_ = vmax > kMinVmax ? vmax : kMinVmax;
    amax_ = amax > kMinAmax ? amax : kMinAmax;
    smooth_time_ = smooth_time > kMinSmooth ? smooth_time : kMinSmooth;
}

void TrajectoryGenerator::clearFilter()
{
    memset(buf_, 0, sizeof(buf_));
    head_ = 0;
    sum_ = 0.0f;
    idle_ = 0;
}

void TrajectoryGenerator::reset(float pos)
{
    clearFilter();
    p1_ = pos;
    v1_ = 0.0f;
    pos_ = pos;
    vel_ = 0.0f;
    acc_ = 0.0f;
    target_ = pos;
    done_ = true;
}

float TrajectoryGenerator::update(float dt)
{
    if (dt <= 0.0f)
    {
        return pos_;
    }

    const float target = target_;

    /* 旁路：直接输出阶跃目标，同时让内部状态跟随，便于随时切回 */
    if (!enabled_)
    {
        if (!done_ || p1_ != target)
        {
            clearFilter();
            p1_ = target;
            v1_ = 0.0f;
            vel_ = 0.0f;
            acc_ = 0.0f;
            done_ = true;
        }
        pos_ = target;
        return pos_;
    }

    /* 静止且目标未变：不做任何运算 */
    if (done_ && target == p1_)
    {
        return pos_;
    }
    done_ = false;

    /* 本周期使用的参数快照，防止线程中途修改造成不一致 */
    const float vmax = vmax_;
    const float amax = amax_;
    float n_f = smooth_time_ / dt + 0.5f;
    uint32_t taps = n_f < 1.0f ? 1u : (n_f > (float)(kMaxTaps - 1) ? (uint32_t)(kMaxTaps - 1) : (uint32_t)n_f);

    /* ---------- 第 1 级：梯形速度规划 ---------- */
    const float err = target - p1_;
    const float dist = absf(err);

    /* 期望速度：不超过 vmax，且能在剩余距离内按 amax 刹住。
     * 刹车速度用"离散精确解"：每拍减速 amax*dt、位置按 p += v*dt 累加，
     * 从 n 拍前的速度 n*amax*dt 刹停所走距离 = amax*dt^2 * n(n+1)/2，
     * 反解 n 得 v_brake = n * amax * dt，这样落点处速度几乎为零，没有末端速度跳变 */
    const float dv_max = amax * dt;
    const float k = dist / (dv_max * dt);
    const float n_brake = 0.5f * (sqrtf(1.0f + 8.0f * k) - 1.0f);
    float v_des = fminf(vmax, n_brake * dv_max);
    v_des = fminf(v_des, dist / dt);
    v_des *= sign(err);

    v1_ += clampf(v_des - v1_, -dv_max, dv_max);

    bool landed = false;
    if (dist <= absf(v1_) * dt && v1_ * err > 0.0f && absf(v1_) <= 2.0f * dv_max)
    {
        /* 已减速到接近零且本拍会到达目标：精确落点，速度归零。
         * 注意：若运动中改目标导致来不及刹车，不在此处硬停(否则速度突变)，
         * 而是允许越过目标后再平滑折返 */
        p1_ = target;
        v1_ = 0.0f;
        landed = true;
    }
    else
    {
        p1_ += v1_ * dt;
    }

    /* ---------- 第 2 级：对 v1 做 taps 点滑动平均 ---------- */
    /* head_ 指向最新样本；写入后窗口为 [next-taps+1, next]，被剔除的是 next-taps 处的样本 */
    const uint32_t mask = kMaxTaps - 1;
    const uint32_t next = (head_ + 1u) & mask;
    if (taps != taps_)
    {
        /* 窗口长度改变：重新累加(新样本 + 最近 taps-1 个旧样本)，罕见，仅调参时发生 */
        float s = v1_;
        for (uint32_t i = 0; i + 1u < taps; i++)
        {
            s += buf_[(head_ - i) & mask];
        }
        sum_ = s;
        taps_ = taps;
    }
    else
    {
        sum_ += v1_ - buf_[(next - taps) & mask];
    }
    buf_[next] = v1_;
    head_ = next;

    const float vel_new = sum_ / (float)taps;
    acc_ = (vel_new - vel_) / dt;
    vel_ = vel_new;
    pos_ += vel_ * dt;

    /* ---------- 到位判定：规划已落点且滤波器排空 ---------- */
    if (landed || (p1_ == target && v1_ == 0.0f))
    {
        if (++idle_ >= taps)
        {
            clearFilter();
            pos_ = target;
            vel_ = 0.0f;
            acc_ = 0.0f;
            done_ = true;
        }
    }
    else
    {
        idle_ = 0;
    }

    return pos_;
}
