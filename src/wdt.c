/* SPDX-License-Identifier: MIT */

#include "wdt.h"
#include "adt.h"
#include "types.h"
#include "utils.h"

#define WDT_COUNT 0x10
#define WDT_ALARM 0x14
#define WDT_CTL   0x1c

#define WD0_CUR_TIME  0x00
#define WD0_BITE_TIME 0x04
#define WD0_BARK_TIME 0x08
#define WD0_CTRL      0x0c
#define WD_CTRL_IRQ_EN     BIT(0)
#define WD_CTRL_IRQ_STATUS BIT(1)
#define WD_CTRL_RESET_EN   BIT(2)

static u64 wdt_base = 0;

void wdt_disable(void)
{
    int path[8];
    int node = adt_path_offset_trace(adt, "/arm-io/wdt", path);

    if (node < 0) {
        printf("WDT node not found!\n");
        return;
    }

    if (adt_get_reg(adt, path, "reg", 0, &wdt_base, NULL)) {
        printf("Failed to get WDT reg property!\n");
        return;
    }

    printf("WDT registers @ 0x%lx\n", wdt_base);

    write32(wdt_base + WDT_CTL, 0);

    printf("WDT disabled\n");
}

void wdt_reboot(void)
{
    if (!wdt_base)
        return;

    write32(wdt_base + WDT_ALARM, 0x100000);
    write32(wdt_base + WDT_COUNT, 0);
    write32(wdt_base + WDT_CTL, 4);
}

/* WDT counts at the 24 MHz system counter rate */
void wdt_arm(u32 timeout_ms)
{
    if (!wdt_base)
        return;

    write32(wdt_base + WDT_COUNT, 0);
    write32(wdt_base + WDT_ALARM, timeout_ms * 24000);
    write32(wdt_base + WDT_CTL, 4);
}

void wdt_stop(void)
{
    if (!wdt_base)
        return;

    write32(wdt_base + WDT_CTL, 0);
}

void wdt_wd0_irq_arm(u32 timeout_ms)
{
    if (!wdt_base)
        return;

    u32 ctrl = read32(wdt_base + WD0_CTRL);
    printf("WD0: before: CUR=%u BITE=%u BARK=%u CTRL=0x%x\n", read32(wdt_base + WD0_CUR_TIME),
           read32(wdt_base + WD0_BITE_TIME), read32(wdt_base + WD0_BARK_TIME), ctrl);

    /* keep whatever RESET_EN state iBoot left, only manage the IRQ bits */
    u32 keep = ctrl & WD_CTRL_RESET_EN;

    write32(wdt_base + WD0_CTRL, keep | WD_CTRL_IRQ_STATUS); /* clear stale status, IRQ off */
    write32(wdt_base + WD0_CUR_TIME, 0);
    write32(wdt_base + WD0_BARK_TIME, timeout_ms * 24000);
    write32(wdt_base + WD0_CTRL, keep | WD_CTRL_IRQ_EN);
}

void wdt_wd0_irq_clear(void)
{
    if (!wdt_base)
        return;

    u32 keep = read32(wdt_base + WD0_CTRL) & WD_CTRL_RESET_EN;
    write32(wdt_base + WD0_CTRL, keep | WD_CTRL_IRQ_STATUS);
    write32(wdt_base + WD0_CTRL, keep);
}

int wdt_wd0_irq_adt(void)
{
    int node = adt_path_offset(adt, "/arm-io/wdt");
    if (node < 0)
        return -1;

    u32 len;
    const u32 *irqs = adt_getprop(adt, node, "interrupts", &len);
    if (!irqs)
        return -1;

    printf("WD0: ADT wdt interrupts:");
    for (u32 i = 0; i < len / 4; i++)
        printf(" %u", irqs[i]);
    printf("\n");

    return len >= 4 ? (int)irqs[0] : -1;
}
