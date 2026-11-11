/*
 * AS5600 encoder test + UART stream for PC visualization
 *
 * Bus: hardware I2C1 @ 400kHz (PB8/PB9), soft I2C for spot check
 *
 * Binary frame (15 bytes, LE) — raw I2C register bytes, host parses:
 *   AA 55
 *   seq:u32
 *   status          @0x0B
 *   raw_h, raw_l    @0x0C..0x0D   (one burst with status+angle)
 *   ang_h, ang_l    @0x0E..0x0F
 *   agc             @0x1A
 *   mag_h, mag_l    @0x1B..0x1C
 *   chk = XOR of bytes [2..13]
 *
 * MSH:
 *   as5600 hw|sw|scan hw|sw
 *   as5600 start [period_us]   binary stream, default 500us
 *   as5600 watch [period_ms]   console print, default 50ms
 *   as5600 stop
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"

#define DBG_TAG "as5600"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define AS5600_ADDR           0x36u
#define AS5600_REG_ZPOS_H     0x01u
#define AS5600_REG_ZPOS_L     0x02u
#define AS5600_REG_MPOS_H     0x03u
#define AS5600_REG_MPOS_L     0x04u
#define AS5600_REG_STATUS     0x0Bu
#define AS5600_REG_RAW_ANGLE  0x0Cu
#define AS5600_REG_ANGLE      0x0Eu
#define AS5600_REG_AGC        0x1Au
#define AS5600_REG_MAG_H      0x1Bu

#define AS5600_SOFT_BUS       "i2c1"
#define AS5600_I2C_TIMEOUT_MS 20u

#define AS5600_FRAME_MAGIC0   0xAAu
#define AS5600_FRAME_MAGIC1   0x55u
#define AS5600_FRAME_SIZE     15u
#define AS5600_DEFAULT_PERIOD_US 500u

#pragma pack(push, 1)
struct as5600_frame
{
    rt_uint8_t  magic0;
    rt_uint8_t  magic1;
    rt_uint32_t seq;
    /* exact I2C bytes: one read 0x0B..0x0F, then 0x1A, then 0x1B..0x1C */
    rt_uint8_t  status;
    rt_uint8_t  raw_h;
    rt_uint8_t  raw_l;
    rt_uint8_t  ang_h;
    rt_uint8_t  ang_l;
    rt_uint8_t  agc;
    rt_uint8_t  mag_h;
    rt_uint8_t  mag_l;
    rt_uint8_t  chk;
};
#pragma pack(pop)

#define AS5600_MODE_IDLE   0
#define AS5600_MODE_BIN    1
#define AS5600_MODE_WATCH  2

#define AS5600_WATCH_DEFAULT_MS 50u

static volatile rt_uint8_t s_stream_mode = AS5600_MODE_IDLE;
static volatile rt_bool_t s_stream_run = RT_FALSE;
static rt_uint32_t s_stream_period_us = AS5600_DEFAULT_PERIOD_US;
static rt_uint32_t s_watch_period_ms = AS5600_WATCH_DEFAULT_MS;
static rt_thread_t s_stream_tid = RT_NULL;
static rt_device_t s_serial = RT_NULL;

/* console uart is opened with RT_DEVICE_FLAG_STREAM (inserts 0x0D before 0x0A).
 * Binary plot must clear it temporarily — same idea as ymodem. */
static rt_uint16_t s_console_oflag_saved;
static rt_bool_t s_console_binary_mode = RT_FALSE;

static void as5600_console_enter_binary(void)
{
    rt_device_t dev;
    rt_base_t level;

    dev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
    if (dev == RT_NULL)
    {
        return;
    }

    level = rt_hw_interrupt_disable();
    s_console_oflag_saved = dev->open_flag;
    dev->open_flag &= ~RT_DEVICE_FLAG_STREAM;
    s_console_binary_mode = RT_TRUE;
    rt_hw_interrupt_enable(level);
}

static void as5600_console_leave_binary(void)
{
    rt_device_t dev;
    rt_base_t level;

    if (!s_console_binary_mode)
    {
        return;
    }

    dev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
    if (dev == RT_NULL)
    {
        s_console_binary_mode = RT_FALSE;
        return;
    }

    level = rt_hw_interrupt_disable();
    if (s_console_oflag_saved & RT_DEVICE_FLAG_STREAM)
    {
        dev->open_flag |= RT_DEVICE_FLAG_STREAM;
    }
    s_console_binary_mode = RT_FALSE;
    rt_hw_interrupt_enable(level);
}

static void as5600_dwt_init(void)
{
    static rt_bool_t inited = RT_FALSE;
    if (inited)
    {
        return;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    inited = RT_TRUE;
}

static void as5600_delay_until(rt_uint32_t start_cycles, rt_uint32_t period_us)
{
    rt_uint32_t target = period_us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start_cycles) < target)
    {
    }
}

static void as5600_prepare_hw(void)
{
    HAL_I2C_DeInit(&hi2c1);
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        LOG_E("HAL_I2C_Init failed");
    }
}

static int as5600_hw_i2c_restore(void)
{
    as5600_prepare_hw();
    return RT_EOK;
}
INIT_APP_EXPORT(as5600_hw_i2c_restore);

static void as5600_prepare_sw(void)
{
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_RESET)
    {
        HAL_I2C_DeInit(&hi2c1);
    }

    rt_pin_mode(BSP_I2C1_SCL_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_mode(BSP_I2C1_SDA_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_write(BSP_I2C1_SCL_PIN, PIN_HIGH);
    rt_pin_write(BSP_I2C1_SDA_PIN, PIN_HIGH);
}

static rt_err_t as5600_hw_read_regs(rt_uint8_t reg, rt_uint8_t *buf, rt_uint16_t len)
{
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Read(&hi2c1,
                          (uint16_t)(AS5600_ADDR << 1),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          len,
                          AS5600_I2C_TIMEOUT_MS);
    return (st == HAL_OK) ? RT_EOK : -RT_ERROR;
}

static rt_uint16_t as5600_u12(const rt_uint8_t *buf)
{
    return (rt_uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);
}

/**
 * Read raw I2C register bytes for debug/stream.
 * blk5: STATUS+RAW+ANGLE from 0x0B (single transaction)
 * agc / mag: separate reads; on fail fill 0xFF
 */
static rt_err_t as5600_read_i2c_raw(rt_uint8_t blk5[5],
                                    rt_uint8_t *agc,
                                    rt_uint8_t mag[2])
{
    if (as5600_hw_read_regs(AS5600_REG_STATUS, blk5, 5) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (as5600_hw_read_regs(AS5600_REG_AGC, agc, 1) != RT_EOK)
    {
        *agc = 0xFFu;
    }

    if (as5600_hw_read_regs(AS5600_REG_MAG_H, mag, 2) != RT_EOK)
    {
        mag[0] = 0xFFu;
        mag[1] = 0xFFu;
    }

    return RT_EOK;
}

static void as5600_diag(void)
{
    rt_uint8_t status = 0;
    rt_uint8_t agc = 0;
    rt_uint8_t buf[2];
    rt_uint16_t raw, angle, zpos, mpos, mag;

    as5600_prepare_hw();

    if (as5600_hw_read_regs(AS5600_REG_STATUS, &status, 1) != RT_EOK ||
        as5600_hw_read_regs(AS5600_REG_AGC, &agc, 1) != RT_EOK ||
        as5600_hw_read_regs(AS5600_REG_RAW_ANGLE, buf, 2) != RT_EOK)
    {
        rt_kprintf("[AS5600] diag read failed\n");
        return;
    }
    raw = as5600_u12(buf);

    if (as5600_hw_read_regs(AS5600_REG_ANGLE, buf, 2) != RT_EOK)
    {
        rt_kprintf("[AS5600] diag ANGLE failed\n");
        return;
    }
    angle = as5600_u12(buf);

    as5600_hw_read_regs(AS5600_REG_ZPOS_H, buf, 2);
    zpos = as5600_u12(buf);
    as5600_hw_read_regs(AS5600_REG_MPOS_H, buf, 2);
    mpos = as5600_u12(buf);
    as5600_hw_read_regs(AS5600_REG_MAG_H, buf, 2);
    mag = as5600_u12(buf);

    rt_kprintf("[AS5600][diag] STATUS=0x%02X MD=%d ML=%d MH=%d\n",
               status, (status >> 5) & 1, (status >> 4) & 1, (status >> 3) & 1);
    rt_kprintf("[AS5600][diag] AGC=%u MAG=%u\n", agc, mag);
    rt_kprintf("[AS5600][diag] RAW=%u (%u.%u deg) ANGLE=%u (%u.%u deg)\n",
               raw,
               (rt_uint32_t)raw * 360u / 4096u,
               ((rt_uint32_t)raw * 3600u / 4096u) % 10u,
               angle,
               (rt_uint32_t)angle * 360u / 4096u,
               ((rt_uint32_t)angle * 3600u / 4096u) % 10u);
    rt_kprintf("[AS5600][diag] ZPOS=%u MPOS=%u (if RAW!=ANGLE a lot, use RAW)\n",
               zpos, mpos);
}

static rt_err_t as5600_hw_read_regs_checked(rt_uint8_t reg, rt_uint8_t *buf, rt_uint16_t len)
{
    as5600_prepare_hw();
    return as5600_hw_read_regs(reg, buf, len);
}

static rt_err_t as5600_sw_read_regs(rt_uint8_t reg, rt_uint8_t *buf, rt_uint16_t len)
{
    struct rt_i2c_bus_device *bus;
    struct rt_i2c_msg msgs[2];
    rt_uint8_t reg_addr = reg;

    as5600_prepare_sw();

    bus = rt_i2c_bus_device_find(AS5600_SOFT_BUS);
    if (bus == RT_NULL)
    {
        LOG_E("soft bus %s not found", AS5600_SOFT_BUS);
        return -RT_ERROR;
    }

    msgs[0].addr  = AS5600_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].len  = 1;
    msgs[0].buf  = &reg_addr;

    msgs[1].addr  = AS5600_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].len  = len;
    msgs[1].buf  = buf;

    if (rt_i2c_transfer(bus, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_uint8_t as5600_frame_checksum(const struct as5600_frame *f)
{
    const rt_uint8_t *p = (const rt_uint8_t *)f;
    rt_uint8_t chk = 0;
    rt_uint8_t i;

    /* XOR payload between magic and chk */
    for (i = 2; i < (AS5600_FRAME_SIZE - 1); i++)
    {
        chk ^= p[i];
    }
    return chk;
}

static void as5600_send_frame(rt_uint32_t seq,
                              const rt_uint8_t blk5[5],
                              rt_uint8_t agc,
                              const rt_uint8_t mag[2])
{
    struct as5600_frame frame;

    RT_ASSERT(sizeof(frame) == AS5600_FRAME_SIZE);

    frame.magic0 = AS5600_FRAME_MAGIC0;
    frame.magic1 = AS5600_FRAME_MAGIC1;
    frame.seq    = seq;
    frame.status = blk5[0];
    frame.raw_h  = blk5[1];
    frame.raw_l  = blk5[2];
    frame.ang_h  = blk5[3];
    frame.ang_l  = blk5[4];
    frame.agc    = agc;
    frame.mag_h  = mag[0];
    frame.mag_l  = mag[1];
    frame.chk    = as5600_frame_checksum(&frame);

    if (s_serial)
    {
        rt_device_write(s_serial, 0, &frame, sizeof(frame));
    }
}

static void as5600_print_result(const char *mode, rt_uint8_t status,
                                rt_uint16_t raw, rt_uint16_t angle)
{
    rt_uint32_t milli_deg = (rt_uint32_t)angle * 360000UL / 4096UL;

    rt_kprintf("[AS5600][%s] status=0x%02X MD=%d ML=%d MH=%d\n",
               mode, status,
               (status >> 5) & 0x01,
               (status >> 4) & 0x01,
               (status >> 3) & 0x01);
    rt_kprintf("[AS5600][%s] raw=%u (0x%03X) angle=%u (0x%03X) deg=%u.%03u\n",
               mode, raw, raw, angle, angle,
               milli_deg / 1000UL, milli_deg % 1000UL);
}

static rt_err_t as5600_test_once(rt_bool_t use_hw)
{
    rt_uint8_t status = 0;
    rt_uint8_t raw_buf[2] = {0};
    rt_uint8_t ang_buf[2] = {0};
    rt_uint16_t raw, angle;
    rt_err_t (*read_regs)(rt_uint8_t, rt_uint8_t *, rt_uint16_t);
    const char *mode = use_hw ? "HW-I2C" : "SW-I2C";

    read_regs = use_hw ? as5600_hw_read_regs_checked : as5600_sw_read_regs;

    if (read_regs(AS5600_REG_STATUS, &status, 1) != RT_EOK)
    {
        rt_kprintf("[AS5600][%s] read STATUS failed (addr=0x%02X, PB8/PB9)\n",
                   mode, AS5600_ADDR);
        return -RT_ERROR;
    }

    if (read_regs(AS5600_REG_RAW_ANGLE, raw_buf, 2) != RT_EOK)
    {
        rt_kprintf("[AS5600][%s] read RAW_ANGLE failed\n", mode);
        return -RT_ERROR;
    }

    if (read_regs(AS5600_REG_ANGLE, ang_buf, 2) != RT_EOK)
    {
        rt_kprintf("[AS5600][%s] read ANGLE failed\n", mode);
        return -RT_ERROR;
    }

    raw   = (rt_uint16_t)(((raw_buf[0] & 0x0F) << 8) | raw_buf[1]);
    angle = (rt_uint16_t)(((ang_buf[0] & 0x0F) << 8) | ang_buf[1]);
    as5600_print_result(mode, status, raw, angle);

    if (!use_hw)
    {
        as5600_prepare_hw();
    }

    return RT_EOK;
}

static void as5600_scan(rt_bool_t use_hw)
{
    rt_uint8_t addr;
    rt_uint32_t found = 0;

    rt_kprintf("[AS5600] %s bus scan 0x08~0x77 ...\n", use_hw ? "HW-I2C" : "SW-I2C");

    if (use_hw)
    {
        as5600_prepare_hw();
        for (addr = 0x08; addr <= 0x77; addr++)
        {
            if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2, 20) == HAL_OK)
            {
                rt_kprintf("  found 0x%02X\n", addr);
                found++;
            }
        }
    }
    else
    {
        struct rt_i2c_bus_device *bus;
        struct rt_i2c_msg msg;
        rt_uint8_t dummy;

        as5600_prepare_sw();
        bus = rt_i2c_bus_device_find(AS5600_SOFT_BUS);
        if (bus == RT_NULL)
        {
            rt_kprintf("  soft bus %s not found\n", AS5600_SOFT_BUS);
            return;
        }

        for (addr = 0x08; addr <= 0x77; addr++)
        {
            msg.addr  = addr;
            msg.flags = RT_I2C_RD;
            msg.len   = 1;
            msg.buf   = &dummy;
            if (rt_i2c_transfer(bus, &msg, 1) == 1)
            {
                rt_kprintf("  found 0x%02X\n", addr);
                found++;
            }
        }
        as5600_prepare_hw();
    }

    rt_kprintf("[AS5600] scan done, %u device(s)\n", found);
}

static void as5600_stream_entry(void *param)
{
    rt_uint32_t seq = 0;
    rt_uint8_t blk5[5];
    rt_uint8_t agc;
    rt_uint8_t mag[2];
    rt_uint32_t t0;

    (void)param;

    as5600_dwt_init();
    as5600_prepare_hw();

    s_serial = rt_device_find(RT_CONSOLE_DEVICE_NAME);
    if (s_serial == RT_NULL)
    {
        rt_kprintf("[AS5600] serial %s not found, stream abort\n", RT_CONSOLE_DEVICE_NAME);
        s_stream_run = RT_FALSE;
        s_stream_mode = AS5600_MODE_IDLE;
        return;
    }

    /* Must run AFTER last rt_kprintf of start path; STREAM off for binary. */
    as5600_console_enter_binary();

    while (s_stream_run)
    {
        t0 = DWT->CYCCNT;

        /* send raw I2C bytes; host does all parsing */
        if (as5600_read_i2c_raw(blk5, &agc, mag) == RT_EOK)
        {
            as5600_send_frame(seq++, blk5, agc, mag);
        }

        as5600_delay_until(t0, s_stream_period_us);
    }

    as5600_console_leave_binary();
    s_stream_tid = RT_NULL;
    s_stream_mode = AS5600_MODE_IDLE;
}

static void as5600_watch_entry(void *param)
{
    rt_uint8_t blk5[5];
    rt_uint8_t agc;
    rt_uint8_t mag[2];
    rt_uint16_t raw, angle, magnitude;
    rt_uint32_t seq = 0;
    rt_uint32_t milli_deg;

    (void)param;

    as5600_prepare_hw();
    rt_kprintf("[AS5600] watch start, period=%ums (as5600 stop to end)\n",
               s_watch_period_ms);

    while (s_stream_run)
    {
        if (as5600_read_i2c_raw(blk5, &agc, mag) == RT_EOK)
        {
            raw = as5600_u12(&blk5[1]);
            angle = as5600_u12(&blk5[3]);
            magnitude = as5600_u12(mag);
            milli_deg = (rt_uint32_t)raw * 360000UL / 4096UL;

            rt_kprintf("[%u] i2c %02X %02X %02X %02X %02X | %02X %02X %02X"
                       "  STAT=0x%02X MD=%d ML=%d MH=%d"
                       "  RAW=%u(%u.%03u) ANG=%u AGC=%u MAG=%u\n",
                       seq++,
                       blk5[0], blk5[1], blk5[2], blk5[3], blk5[4],
                       agc, mag[0], mag[1],
                       blk5[0],
                       (blk5[0] >> 5) & 1,
                       (blk5[0] >> 4) & 1,
                       (blk5[0] >> 3) & 1,
                       raw,
                       milli_deg / 1000UL,
                       milli_deg % 1000UL,
                       angle,
                       agc,
                       magnitude);
        }
        else
        {
            rt_kprintf("[AS5600] watch I2C read failed\n");
        }

        rt_thread_mdelay(s_watch_period_ms);
    }

    s_stream_tid = RT_NULL;
    s_stream_mode = AS5600_MODE_IDLE;
}

static rt_err_t as5600_thread_start(rt_uint8_t mode,
                                    void (*entry)(void *param),
                                    const char *name)
{
    if (s_stream_run)
    {
        rt_kprintf("[AS5600] already running, use: as5600 stop\n");
        return -RT_EBUSY;
    }

    s_stream_mode = mode;
    s_stream_run = RT_TRUE;
    s_stream_tid = rt_thread_create(name, entry, RT_NULL, 1536, 12, 10);
    if (s_stream_tid == RT_NULL)
    {
        s_stream_run = RT_FALSE;
        s_stream_mode = AS5600_MODE_IDLE;
        rt_kprintf("[AS5600] create thread failed\n");
        return -RT_ERROR;
    }

    rt_thread_startup(s_stream_tid);
    return RT_EOK;
}

rt_err_t as5600_stream_start(rt_uint32_t period_us)
{
    if (period_us < 200u)
    {
        period_us = 200u;
    }
    if (period_us > 100000u)
    {
        period_us = 100000u;
    }

    s_stream_period_us = period_us;

    /* print while STREAM still on, then thread clears STREAM for binary TX */
    rt_kprintf("[AS5600] stream start, I2C=400kHz period=%uus raw-i2c frame=%uB\n",
               s_stream_period_us, (unsigned)AS5600_FRAME_SIZE);
    rt_kprintf("[AS5600] clear console STREAM on uart1 for binary (no extra UART)\n");

    if (as5600_thread_start(AS5600_MODE_BIN, as5600_stream_entry, "as5600s") != RT_EOK)
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t as5600_watch_start(rt_uint32_t period_ms)
{
    if (period_ms < 10u)
    {
        period_ms = 10u;
    }
    if (period_ms > 5000u)
    {
        period_ms = 5000u;
    }

    s_watch_period_ms = period_ms;
    return as5600_thread_start(AS5600_MODE_WATCH, as5600_watch_entry, "as5600w");
}

static void as5600_stream_stop(void)
{
    if (!s_stream_run)
    {
        rt_kprintf("[AS5600] not running\n");
        return;
    }

    s_stream_run = RT_FALSE;
    /* wait thread exit (watch period may be long; binary also needs leave_binary) */
    rt_thread_mdelay((s_stream_mode == AS5600_MODE_WATCH) ?
                     (s_watch_period_ms + 20) : 20);
    /* belt-and-suspenders if thread already exited */
    as5600_console_leave_binary();
    s_stream_mode = AS5600_MODE_IDLE;
    rt_kprintf("[AS5600] stop\n");
}

static void as5600_usage(void)
{
    rt_kprintf("Usage:\n");
    rt_kprintf("  as5600 hw                 - one-shot hardware I2C read\n");
    rt_kprintf("  as5600 sw                 - one-shot software I2C read\n");
    rt_kprintf("  as5600 scan hw|sw         - scan I2C bus\n");
    rt_kprintf("  as5600 diag               - STATUS/AGC/ZPOS/MPOS/RAW/ANGLE\n");
    rt_kprintf("  as5600 watch [period_ms]  - console print RAW (default 50ms)\n");
    rt_kprintf("  as5600 start [period_us]  - binary stream for PC plot (default 500)\n");
    rt_kprintf("  as5600 stop               - stop watch/stream\n");
    rt_kprintf("PC plot: python tools/as5600_plot.py -p COMx -b 460800\n");
}

static int as5600(int argc, char **argv)
{
    if (argc < 2)
    {
        as5600_usage();
        return 0;
    }

    if (!strcmp(argv[1], "hw"))
    {
        return (as5600_test_once(RT_TRUE) == RT_EOK) ? 0 : -1;
    }
    else if (!strcmp(argv[1], "sw"))
    {
        return (as5600_test_once(RT_FALSE) == RT_EOK) ? 0 : -1;
    }
    else if (!strcmp(argv[1], "scan"))
    {
        if (argc < 3)
        {
            as5600_usage();
            return 0;
        }
        if (!strcmp(argv[2], "hw"))
        {
            as5600_scan(RT_TRUE);
            return 0;
        }
        if (!strcmp(argv[2], "sw"))
        {
            as5600_scan(RT_FALSE);
            return 0;
        }
    }
    else if (!strcmp(argv[1], "diag"))
    {
        as5600_diag();
        return 0;
    }
    else if (!strcmp(argv[1], "watch"))
    {
        rt_uint32_t period = AS5600_WATCH_DEFAULT_MS;
        if (argc >= 3)
        {
            period = (rt_uint32_t)atoi(argv[2]);
        }
        return (as5600_watch_start(period) == RT_EOK) ? 0 : -1;
    }
    else if (!strcmp(argv[1], "start"))
    {
        rt_uint32_t period = AS5600_DEFAULT_PERIOD_US;
        if (argc >= 3)
        {
            period = (rt_uint32_t)atoi(argv[2]);
        }
        return (as5600_stream_start(period) == RT_EOK) ? 0 : -1;
    }
    else if (!strcmp(argv[1], "stop"))
    {
        as5600_stream_stop();
        return 0;
    }

    as5600_usage();
    return 0;
}
MSH_CMD_EXPORT(as5600, AS5600: hw|sw|diag|watch|start|stop);
