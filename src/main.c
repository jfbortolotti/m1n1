/* SPDX-License-Identifier: MIT */

#include "../build/build_cfg.h"
#include "../build/build_tag.h"

#include "../config.h"

#include "adt.h"
#include "aic.h"
#include "cpufreq.h"
#include "display.h"
#include "exception.h"
#include "fb.h"
#include "firmware.h"
#include "gxf.h"
#include "heapblock.h"
#include "mcc.h"
#include "memory.h"
#include "nvme.h"
#include "payload.h"
#include "pcie.h"
#include "pmgr.h"
#include "sep.h"
#include "smp.h"
#include "string.h"
#include "uart.h"
#include "uartproxy.h"
#include "usb.h"
#include "utils.h"
#include "wdt.h"
#include "xnuboot.h"

struct vector_args next_stage;

const char version_tag[] = "##m1n1_ver##" BUILD_TAG;
const char *const m1n1_version = version_tag + 12;

u32 board_id = ~0, chip_id = ~0;

void get_device_info(void)
{
    const char *model = (const char *)adt_getprop(adt, 0, "model", NULL);
    const char *target = (const char *)adt_getprop(adt, 0, "target-type", NULL);

    printf("Device info:\n");

    if (model)
        printf("  Model: %s\n", model);

    if (target)
        printf("  Target: %s\n", target);

    is_mac = !!strstr(model, "Mac");

    int chosen = adt_path_offset(adt, "/chosen");
    if (chosen > 0) {
        if (ADT_GETPROP(adt, chosen, "board-id", &board_id) < 0)
            printf("Failed to find board-id\n");
        if (ADT_GETPROP(adt, chosen, "chip-id", &chip_id) < 0)
            printf("Failed to find chip-id\n");

        printf("  Board-ID: 0x%x\n", board_id);
        printf("  Chip-ID: 0x%x\n", chip_id);
    } else {
        printf("No chosen node!\n");
    }

    printf("\n");
}

void run_actions(void)
{
    bool usb_up = false;

#ifndef BRINGUP
#ifdef EARLY_PROXY_TIMEOUT
    int node = adt_path_offset(adt, "/chosen/asmb");
    u64 lp_sip0 = 0;

    if (node >= 0) {
        ADT_GETPROP(adt, node, "lp-sip0", &lp_sip0);
        printf("Boot policy: sip0 = %ld\n", lp_sip0);
    }

    if (!cur_boot_args.video.display && lp_sip0 == 127) {
        printf("Bringing up USB for early debug...\n");

        usb_init();
        usb_iodev_init();

        usb_up = true;

        printf("Waiting for proxy connection... ");
        for (int i = 0; i < EARLY_PROXY_TIMEOUT * 100; i++) {
            for (int j = 0; j < USB_IODEV_COUNT; j++) {
                iodev_id_t iodev = IODEV_USB0 + j;

                if (!(iodev_get_usage(iodev) & USAGE_UARTPROXY))
                    continue;

                usb_iodev_vuart_setup(iodev);
                iodev_handle_events(iodev);
                if (iodev_can_write(iodev) || iodev_can_write(IODEV_USB_VUART)) {
                    printf(" Connected!\n");
                    uartproxy_run(NULL);
                    return;
                }
            }

            mdelay(10);
            if (i % 100 == 99)
                printf(".");
        }
        printf(" Timed out\n");
    }
#endif
#endif

    printf("Checking for payloads...\n");

    if (payload_run() == 0) {
        printf("Valid payload found\n");
        return;
    }
    fb_set_active(true);

    printf("No valid payload found\n");

#ifndef BRINGUP
    if (!usb_up) {
        usb_init();
        usb_iodev_init();
    }
#endif

    printf("Running proxy...\n");

    uartproxy_run(NULL);
}

/*
 * Early battery meter (2026-11-20): read the gauge keys once, before any proxy/USB, so a run that
 * ends in a reset (whole-SoC attempt + 178 s watchdog) can be measured: the PC reads this struct
 * later over the proxy (proxyclient: early_meter.py), after the USB-C is plugged in and charging.
 */
#include "smc.h"
struct early_meter {
    u64 magic; /* 'BAACMETR' when valid */
    u64 cntpct, cntfrq;
    s64 baac;
    u16 b0rm, b0av;
    s16 b0ac;
    u16 ok;
};
struct early_meter early_meter;

static void early_meter_read(void)
{
    smc_dev_t *smc = smc_init();
    s64 baac = 0;
    u16 rm = 0, av = 0;
    s16 ac = 0;

    if (!smc) {
        printf("early_meter: no SMC\n");
        return;
    }
    early_meter.cntpct = mrs(CNTPCT_EL0);
    early_meter.cntfrq = mrs(CNTFRQ_EL0);
    early_meter.ok = !smc_read(smc, 0x42414143 /* BAAC */, &baac, 8);
    early_meter.ok |= (!smc_read(smc, 0x4230524d /* B0RM */, &rm, 2)) << 1;
    early_meter.ok |= (!smc_read(smc, 0x42304143 /* B0AC */, &ac, 2)) << 2;
    early_meter.ok |= (!smc_read(smc, 0x42304156 /* B0AV */, &av, 2)) << 3;
    early_meter.baac = baac;
    early_meter.b0rm = rm;
    early_meter.b0ac = ac;
    early_meter.b0av = av;
    early_meter.magic = 0x5254454d43414142ULL; /* "BAACMETR" */
    smc_shutdown(smc);
    printf("early_meter @%p: BAAC %ld B0RM %u mAh B0AC %d mA B0AV %u mV (ok %#x) at %lu\n",
           &early_meter, baac, rm,
           ac, av, early_meter.ok, early_meter.cntpct);
}

void m1n1_main(void)
{
    printf("\n\nm1n1 %s\n", m1n1_version);
    printf("Copyright The Asahi Linux Contributors\n");
    printf("Licensed under the MIT license\n\n");

    printf("Running in EL%lu\n\n", mrs(CurrentEL) >> 2);

    firmware_init();

    heapblock_init();

#ifndef BRINGUP
    if (supports_gxf())
        gxf_init();
    mcc_init();
    mmu_init();
    aic_init();
#endif
    wdt_disable();
#ifndef BRINGUP
    pmgr_init();
    early_meter_read();
#ifdef USE_FB
    display_init();
    // Kick DCP to sleep, so dodgy monitors which cause reconnect cycles don't cause us to lose the
    // framebuffer.
    display_shutdown(DCP_SLEEP_IF_EXTERNAL);
    // On idevice we need to always clear, because otherwise it looks scuffed on white devices
    fb_init(!is_mac);
    fb_display_logo();
#ifdef FB_SILENT_MODE
    fb_set_active(!cur_boot_args.video.display);
#else
    fb_set_active(true);
#endif
#endif

    cpufreq_fixup();
    sep_init();
#endif

    printf("Initialization complete.\n");

    run_actions();

    if (!next_stage.entry) {
        panic("Nothing to do!\n");
    }

    printf("Preparing to run next stage at %p...\n", next_stage.entry);

    nvme_shutdown();
    exception_shutdown();
#ifndef BRINGUP
    usb_iodev_shutdown();
    display_shutdown(DCP_SLEEP_IF_EXTERNAL);
#ifdef USE_FB
    fb_shutdown(next_stage.restore_logo);
#endif
    mmu_shutdown();
#endif

    printf("Vectoring to next stage...\n");

    next_stage.entry(next_stage.args[0], next_stage.args[1], next_stage.args[2], next_stage.args[3],
                     next_stage.args[4]);

    panic("Next stage returned!\n");
}
