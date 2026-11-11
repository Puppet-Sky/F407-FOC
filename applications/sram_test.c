/*
 * External SRAM (FSMC Bank4 / NE4) test
 *
 * Base address: 0x6C000000
 * Default size: 1MB (A0~A18, 16-bit)
 *
 * MSH:
 *   sram_test          - full 1MB test
 *   sram_test 64       - test first 64KB
 *   sram_test quick    - fast smoke test
 */

#include <rtthread.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"

#define EXT_SRAM_BASE   ((volatile rt_uint16_t *)0x6C000000UL)
#define EXT_SRAM_BYTES  (1024UL * 1024UL)   /* 1MB */
#define EXT_SRAM_WORDS  (EXT_SRAM_BYTES / 2UL)

static rt_err_t sram_check_word(rt_uint32_t index, rt_uint16_t expect)
{
    rt_uint16_t got = EXT_SRAM_BASE[index];
    if (got != expect)
    {
        rt_kprintf("[SRAM] FAIL @0x%08X write=0x%04X read=0x%04X\n",
                   (rt_uint32_t)(0x6C000000UL + index * 2UL), expect, got);
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_err_t sram_test_patterns(rt_uint32_t words)
{
    const rt_uint16_t patterns[] = {0x0000, 0xFFFF, 0x5555, 0xAAAA, 0x00FF, 0xFF00};
    rt_uint32_t i, p;

    rt_kprintf("[SRAM] pattern test, words=%u\n", words);

    for (p = 0; p < sizeof(patterns) / sizeof(patterns[0]); p++)
    {
        for (i = 0; i < words; i++)
        {
            EXT_SRAM_BASE[i] = patterns[p];
        }
        for (i = 0; i < words; i++)
        {
            if (sram_check_word(i, patterns[p]) != RT_EOK)
            {
                return -RT_ERROR;
            }
        }
        rt_kprintf("  pattern 0x%04X OK\n", patterns[p]);
    }

    return RT_EOK;
}

static rt_err_t sram_test_address(rt_uint32_t words)
{
    rt_uint32_t i;

    rt_kprintf("[SRAM] address test, words=%u\n", words);

    for (i = 0; i < words; i++)
    {
        EXT_SRAM_BASE[i] = (rt_uint16_t)(i & 0xFFFF);
    }
    for (i = 0; i < words; i++)
    {
        if (sram_check_word(i, (rt_uint16_t)(i & 0xFFFF)) != RT_EOK)
        {
            return -RT_ERROR;
        }
    }

    rt_kprintf("  address march OK\n");
    return RT_EOK;
}

static rt_err_t sram_test_walking(void)
{
    rt_uint32_t bit;
    rt_uint32_t index;

    rt_kprintf("[SRAM] walking-1 address/data smoke\n");

    /* walking bit on data at base */
    for (bit = 0; bit < 16; bit++)
    {
        rt_uint16_t v = (rt_uint16_t)(1u << bit);
        EXT_SRAM_BASE[0] = v;
        if (sram_check_word(0, v) != RT_EOK)
        {
            return -RT_ERROR;
        }
    }

    /* walking address: write unique marker at power-of-two offsets */
    for (bit = 0; bit < 19; bit++)
    {
        index = 1UL << bit;
        if (index >= EXT_SRAM_WORDS)
        {
            break;
        }
        EXT_SRAM_BASE[index] = (rt_uint16_t)(0xA000u | bit);
    }
    for (bit = 0; bit < 19; bit++)
    {
        index = 1UL << bit;
        if (index >= EXT_SRAM_WORDS)
        {
            break;
        }
        if (sram_check_word(index, (rt_uint16_t)(0xA000u | bit)) != RT_EOK)
        {
            return -RT_ERROR;
        }
    }

    rt_kprintf("  walking test OK\n");
    return RT_EOK;
}

static rt_err_t sram_test_quick(void)
{
    rt_uint32_t i;
    const rt_uint32_t n = 256;

    rt_kprintf("[SRAM] quick test @0x%08X\n", 0x6C000000UL);

    if (sram_test_walking() != RT_EOK)
    {
        return -RT_ERROR;
    }

    for (i = 0; i < n; i++)
    {
        EXT_SRAM_BASE[i] = (rt_uint16_t)(0x5A00u | (i & 0xFF));
    }
    for (i = 0; i < n; i++)
    {
        if (sram_check_word(i, (rt_uint16_t)(0x5A00u | (i & 0xFF))) != RT_EOK)
        {
            return -RT_ERROR;
        }
    }

    /* corner addresses */
    EXT_SRAM_BASE[0] = 0x1234;
    EXT_SRAM_BASE[EXT_SRAM_WORDS - 1] = 0xCDEF;
    if (sram_check_word(0, 0x1234) != RT_EOK)
    {
        return -RT_ERROR;
    }
    if (sram_check_word(EXT_SRAM_WORDS - 1, 0xCDEF) != RT_EOK)
    {
        return -RT_ERROR;
    }

    rt_kprintf("[SRAM] quick PASS\n");
    return RT_EOK;
}

static void sram_usage(void)
{
    rt_kprintf("Usage:\n");
    rt_kprintf("  sram_test          - full 1MB pattern+address test\n");
    rt_kprintf("  sram_test <KB>     - test first N KB (e.g. sram_test 64)\n");
    rt_kprintf("  sram_test quick    - fast smoke test\n");
    rt_kprintf("FSMC Bank4 NE4 base=0x6C000000 size=1MB width=16bit\n");
}

static int sram_test(int argc, char **argv)
{
    rt_uint32_t bytes = EXT_SRAM_BYTES;
    rt_uint32_t words;
    rt_tick_t t0, t1;

    if (argc >= 2)
    {
        if (!strcmp(argv[1], "quick") || !strcmp(argv[1], "-q"))
        {
            return (sram_test_quick() == RT_EOK) ? 0 : -1;
        }
        if (!strcmp(argv[1], "help") || !strcmp(argv[1], "-h"))
        {
            sram_usage();
            return 0;
        }

        bytes = (rt_uint32_t)atoi(argv[1]) * 1024UL;
        if (bytes == 0 || bytes > EXT_SRAM_BYTES || (bytes & 1u))
        {
            rt_kprintf("invalid size, use 1~1024 (KB)\n");
            sram_usage();
            return -1;
        }
    }

    words = bytes / 2UL;
    rt_kprintf("[SRAM] start test base=0x%08X bytes=%u\n", 0x6C000000UL, bytes);

    t0 = rt_tick_get();
    if (sram_test_walking() != RT_EOK)
    {
        rt_kprintf("[SRAM] FAIL\n");
        return -1;
    }
    if (sram_test_patterns(words) != RT_EOK)
    {
        rt_kprintf("[SRAM] FAIL\n");
        return -1;
    }
    if (sram_test_address(words) != RT_EOK)
    {
        rt_kprintf("[SRAM] FAIL\n");
        return -1;
    }
    t1 = rt_tick_get();

    rt_kprintf("[SRAM] PASS, elapsed %u ms\n",
               (rt_uint32_t)((t1 - t0) * 1000 / RT_TICK_PER_SECOND));
    return 0;
}
MSH_CMD_EXPORT(sram_test, External SRAM FSMC test: sram_test [KB|quick]);
