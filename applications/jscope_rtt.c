#include "jscope_rtt.h"

#include "main.h"
#include "simpleFoc/port/foc_api.h"
#include "hw_current.h"

/*
 * J-Scope 同步采样（RTT 模式，不是 HSS，也不是 RTT Plus）。
 *
 * 通道名即数据格式，J-Scope 连上后自动识别，不需要 elf 符号表：
 *   JScope_t4f4f4f4f4f4f4f4f4f4f4
 *   t4  微秒时间戳（TIM5，作横轴）
 *   f4  target       用户目标角度 rad
 *   f4  planned      规划角度 rad
 *   f4  planned_vel  规划速度 rad/s
 *   f4  angle        实际多圈角度 rad
 *   f4  velocity     速度环使用的实测速度 rad/s
 *   f4  vel_sp       位置环速度给定 rad/s
 *   f4  uq           q 轴电压 V
 *   f4  ua           A 相去偏置电压 V
 *   f4  ub           B 相去偏置电压 V
 *   f4  uc           C 相去偏置电压 V
 *
 * 在 TIM1 中断里每圈写一整包（当前约 5 kHz）。缓冲用 NO_BLOCK_SKIP：
 * 空间不够就丢掉这一包，避免 FOC 中断被 RTT 堵住。
 *
 * 控制块符号必须叫 _SEGGER_RTT，魔数 "SEGGER RTT" 放在 RAM，
 * J-Link 按这个布局扫描，描述符尺寸不能改。
 */

#define JSCOPE_UP_BUF_SIZE 8192u

typedef struct
{
    const char *sName;
    char *pBuffer;
    unsigned SizeOfBuffer;
    unsigned WrOff;
    volatile unsigned RdOff;
    unsigned Flags;
} rtt_up_t;

typedef struct
{
    const char *sName;
    char *pBuffer;
    unsigned SizeOfBuffer;
    volatile unsigned WrOff;
    unsigned RdOff;
    unsigned Flags;
} rtt_down_t;

typedef struct
{
    char acID[16];
    int MaxNumUpBuffers;
    int MaxNumDownBuffers;
    rtt_up_t aUp[1];
    rtt_down_t aDown[1];
} rtt_cb_t;

typedef char rtt_up_size_ok[(sizeof(rtt_up_t) == 24) ? 1 : -1];
typedef char rtt_down_size_ok[(sizeof(rtt_down_t) == 24) ? 1 : -1];

typedef struct __attribute__((packed))
{
    uint32_t us;
    float target;
    float planned;
    float planned_vel;
    float angle;
    float velocity;
    float vel_sp;
    float uq;
    float ua;
    float ub;
    float uc;
} jscope_pkt_t;

typedef char jscope_pkt_size_ok[(sizeof(jscope_pkt_t) == 44) ? 1 : -1];

__attribute__((used, aligned(16)))
rtt_cb_t _SEGGER_RTT;

static char jscope_up_buf[JSCOPE_UP_BUF_SIZE];
static char jscope_down_buf[16];
static volatile int jscope_ready = 0;
static volatile int jscope_enable = 1;
static volatile unsigned jscope_drops = 0;

static const char jscope_ch_name[] = "JScope_t4f4f4f4f4f4f4f4f4f4f4";

static void rtt_write_skip(const void *data, unsigned num)
{
    rtt_up_t *up = &_SEGGER_RTT.aUp[0];
    unsigned size = up->SizeOfBuffer;
    unsigned wr = up->WrOff;
    unsigned rd = up->RdOff;
    unsigned avail;
    unsigned rem;
    const char *src;

    if (rd <= wr)
    {
        avail = size - 1u - wr + rd;
    }
    else
    {
        avail = rd - wr - 1u;
    }
    if (avail < num)
    {
        jscope_drops++;
        return;
    }

    src = (const char *)data;
    rem = size - wr;
    if (rem > num)
    {
        memcpy(up->pBuffer + wr, src, num);
        __DMB();
        up->WrOff = wr + num;
    }
    else
    {
        memcpy(up->pBuffer + wr, src, rem);
        if (num > rem)
        {
            memcpy(up->pBuffer, src + rem, num - rem);
        }
        __DMB();
        up->WrOff = num - rem;
    }
}

void jscope_init(void)
{
    volatile char *id;

    if (jscope_ready)
    {
        return;
    }

    _SEGGER_RTT.MaxNumUpBuffers = 1;
    _SEGGER_RTT.MaxNumDownBuffers = 1;

    _SEGGER_RTT.aUp[0].sName = jscope_ch_name;
    _SEGGER_RTT.aUp[0].pBuffer = jscope_up_buf;
    _SEGGER_RTT.aUp[0].SizeOfBuffer = JSCOPE_UP_BUF_SIZE;
    _SEGGER_RTT.aUp[0].WrOff = 0;
    _SEGGER_RTT.aUp[0].RdOff = 0;
    _SEGGER_RTT.aUp[0].Flags = 0;

    _SEGGER_RTT.aDown[0].sName = "Terminal";
    _SEGGER_RTT.aDown[0].pBuffer = jscope_down_buf;
    _SEGGER_RTT.aDown[0].SizeOfBuffer = sizeof(jscope_down_buf);
    _SEGGER_RTT.aDown[0].WrOff = 0;
    _SEGGER_RTT.aDown[0].RdOff = 0;
    _SEGGER_RTT.aDown[0].Flags = 0;

    /* 描述符写完再发布魔数，避免 J-Link 扫到半初始化的控制块。
     * 分三段写，固件里也不会留下连续的 "SEGGER RTT" 字面量。 */
    id = _SEGGER_RTT.acID;
    id[7] = 'R';
    id[8] = 'T';
    id[9] = 'T';
    id[10] = '\0';
    __DMB();
    id[6] = ' ';
    __DMB();
    id[0] = 'S';
    id[1] = 'E';
    id[2] = 'G';
    id[3] = 'G';
    id[4] = 'E';
    id[5] = 'R';
    __DMB();

    jscope_ready = 1;
}

void jscope_sample(void)
{
    foc_state_t st = {0};
    jscope_pkt_t pkt;
    hw_current_sample_t cur;

    if (!jscope_ready || !jscope_enable)
    {
        return;
    }

    // foc_get_state(0, &st);
    hw_current_get(&cur);

    pkt.us = TIM5->CNT;
    pkt.target = st.target;
    pkt.planned = st.planned;
    pkt.planned_vel = st.planned_vel;
    pkt.angle = st.angle;
    pkt.velocity = st.velocity;
    pkt.vel_sp = st.vel_sp;
    pkt.uq = st.uq;     
    pkt.ua = cur.ia;
    pkt.ub = cur.ib;
    pkt.uc = cur.ic;
    rtt_write_skip(&pkt, sizeof(pkt));
}

void jscope_set_enable(int enable)
{
    if (enable)
    {
        jscope_drops = 0;
    }
    jscope_enable = enable ? 1 : 0;
}

int jscope_get_enable(void)
{
    return jscope_enable;
}

unsigned jscope_drop_count(void)
{
    return jscope_drops;
}
