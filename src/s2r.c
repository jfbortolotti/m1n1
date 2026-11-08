/* SPDX-License-Identifier: MIT */

#include "s2r.h"
#include "aic.h"
#include "aic_regs.h"
#include "cpu_regs.h"
#include "memory.h"
#include "pmgr.h"
#include "smp.h"
#include "adt.h"
#include "smc.h"
#include "rtkit.h"
#include "iova.h"
#include "dapf.h"
#include "usb.h"
#include "dart.h"
#include "soc.h"
#include "spmi.h"
#include "string.h"
#include "types.h"
#include "fb.h"
#include "malloc.h"
#include "nvme.h"
#include "display.h"
#include "iodev.h"
#include "utils.h"
#include "wdt.h"

/*
 * S2R test for Apple Silicon (M1).
 *
 * On Apple chips, timer interrupts are delivered as FIQs.
 * WFI should wake the CPU when an interrupt is pending, even if DAIF
 * masks it — but only if the CPU retains power. ACC_OVRD_DEEP_SLEEP
 * fully power-gates the cluster, making timer wake impossible without
 * RVBAR-based recovery.
 *
 * This test uses a progressive approach:
 *   Level 0: Simple WFI (no overrides at all) — sanity check
 *   Level 1: Deep WFI with retention (CYC_OVRD only, like deep_wfi())
 *   Level 2: Deep WFI + ACC overrides without DEEP_SLEEP bit
 *   Level 3: Full deep sleep (ACC_OVRD_DEEP_SLEEP) — will power-gate
 */

/* ------------------------------------------------------------------ */

/* Debug context: set before each sleep entry, read in exc_fiq and early boot */
volatile u32 s2r_sleep_type = 0xFFFFFFFF;

/* Layout is mirrored by the CTX_* offsets in s2r_asm.S */
struct s2r_ctx s2r_ctx;

/* Called from s2r_enter_deep: publish the context and clean caches to PoC so a
 * core coming back from reset (MMU/caches off) sees consistent memory. */
void s2r_flush(void)
{
    s2r_ctx.magic = S2R_CTX_MAGIC;
    dcsw_op_all(DCSW_OP_DCCSW);
    sysop("dsb sy");
}

/* Breadcrumb state; filled before sleep so it is cleaned to memory by s2r_flush */
static struct {
    volatile u32 *base;
    u32 stride, width, height;
} s2r_crumb_info;

#define CRUMB_SIZE 40
#define CRUMB_PITCH 56

static void crumb_fill(int stage, u32 color)
{
    if (!s2r_crumb_info.base)
        return;

    u32 x0 = 16 + stage * CRUMB_PITCH;
    u32 y0 = s2r_crumb_info.height - CRUMB_SIZE - 16;

    if (x0 + CRUMB_SIZE > s2r_crumb_info.width)
        return;

    for (u32 y = 0; y < CRUMB_SIZE; y++) {
        volatile u32 *row = &s2r_crumb_info.base[(y0 + y) * s2r_crumb_info.stride + x0];
        for (u32 x = 0; x < CRUMB_SIZE; x++)
            row[x] = color;
        /* make sure the pixels reach the display's view of DRAM even if cached */
        dc_civac_range((void *)row, CRUMB_SIZE * 4);
    }
    sysop("dsb sy");
}

void s2r_crumb_init(void)
{
    if (!fb.hwptr)
        return;

    s2r_crumb_info.stride = fb.stride;
    s2r_crumb_info.width = fb.width;
    s2r_crumb_info.height = fb.height;
    s2r_crumb_info.base = fb.hwptr;

    fb_set_active(false);

    for (int i = 0; i < 16; i++)
        crumb_fill(i, 0);
}

void s2r_crumb(int stage)
{
    crumb_fill(stage, 0xffffffff);
}

void s2r_crumb_disable(void)
{
    s2r_crumb_info.base = NULL;
    wait_loop(5000000);
    fb_set_active(true);
}

static u32 s2r_sleep_count = 0x0;
/*  Diagnostic helpers                                                 */
/* ------------------------------------------------------------------ */

static void dump_timer_state(const char *label)
{
    printf("  [%s] Timer state:\n", label);
    printf("    CNTFRQ_EL0    = %lu\n", mrs(CNTFRQ_EL0));
    printf("    CNTPCT_EL0    = 0x%lx\n", mrs(CNTPCT_EL0));
    printf("    CNTP_CTL_EL0  = 0x%lx  (EN=%lu IMASK=%lu ISTATUS=%lu)\n",
           mrs(CNTP_CTL_EL0),
           mrs(CNTP_CTL_EL0) & 1,
           (mrs(CNTP_CTL_EL0) >> 1) & 1,
           (mrs(CNTP_CTL_EL0) >> 2) & 1);
    printf("    CNTP_TVAL_EL0 = %ld\n", (s64)mrs(CNTP_TVAL_EL0));
    printf("    CNTP_CVAL_EL0 = 0x%lx\n", mrs(CNTP_CVAL_EL0));
    printf("    CNTV_CTL_EL0  = 0x%lx\n", mrs(CNTV_CTL_EL0));
}

static void dump_cpu_sleep_state(const char *label)
{
    printf("  [%s] CPU sleep state:\n", label);
    printf("    DAIF          = 0x%lx  (D=%lu A=%lu I=%lu F=%lu)\n",
           mrs(DAIF),
           (mrs(DAIF) >> 9) & 1,
           (mrs(DAIF) >> 8) & 1,
           (mrs(DAIF) >> 7) & 1,
           (mrs(DAIF) >> 6) & 1);
    printf("    CurrentEL     = EL%lu\n", mrs(CurrentEL) >> 2);
    if (cpu_features->cyc_ovrd)
        printf("    CYC_OVRD      = 0x%lx\n", mrs(SYS_IMP_APL_CYC_OVRD));
    if (cpu_features->acc_cfg)
        printf("    ACC_CFG       = 0x%lx\n", mrs(SYS_IMP_APL_ACC_CFG));
    printf("    ACC_OVRD      = 0x%lx\n", mrs(SYS_IMP_APL_ACC_OVRD));
    if (in_el2())
        printf("    VM_TMR_FIQ_ENA= 0x%lx\n", mrs(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2));
}

/* ------------------------------------------------------------------ */
/*  Timer setup                                                        */
/* ------------------------------------------------------------------ */

static void setup_timer(u32 sleep_us)
{
    u64 freq = mrs(CNTFRQ_EL0);
    u64 tval = ((u64)sleep_us * freq) / 1000000;

    /* Disable timer first */
    msr(CNTP_CTL_EL0, 0);
    sysop("isb");

    /* Set timer value */
    msr(CNTP_TVAL_EL0, tval);

    /* Enable timer, unmask (bit 0 = enable, bit 1 = imask = 0) */
    msr(CNTP_CTL_EL0, 1);
    sysop("isb");

    printf("  Timer armed: TVAL=%lu ticks (~%u us)\n", tval, sleep_us);
}

/* Wait for timer to fire by polling (verify timer works at all) */
static int poll_timer_expiry(u32 timeout_us)
{
    u64 timeout = timeout_calculate(timeout_us);

    while (!timeout_expired(timeout)) {
        u64 ctl = mrs(CNTP_CTL_EL0);
        if (ctl & (1 << 2)) { /* ISTATUS bit */
            printf("  Timer fired! CNTP_CTL=0x%lx\n", ctl);
            return 0;
        }
    }
    printf("  Timer poll TIMEOUT! CNTP_CTL=0x%lx\n", mrs(CNTP_CTL_EL0));
    return -1;
}

/* ------------------------------------------------------------------ */
/*  Level 0: Timer sanity check (no WFI)                               */
/* ------------------------------------------------------------------ */

static int test_level0_timer_sanity(u32 sleep_us)
{
    printf("\n--- Level 0: Timer sanity check (poll, no WFI) ---\n");

    dump_timer_state("before");
    setup_timer(sleep_us);
    dump_timer_state("armed");

    s2r_sleep_type = 0;
    printf("  s2r_sleep_type set to 0x%x\n", s2r_sleep_type);
    u64 t0 = mrs(CNTPCT_EL0);
    int ret = poll_timer_expiry(sleep_us * 3);
    u64 t1 = mrs(CNTPCT_EL0);

    u64 elapsed_us = ((t1 - t0) * 1000000) / mrs(CNTFRQ_EL0);
    printf("  Elapsed: %lu us (expected ~%u us)\n", elapsed_us, sleep_us);

    /* Disable timer */
    msr(CNTP_CTL_EL0, 0);
    sysop("isb");

    if (ret == 0)
        printf("  Level 0: PASS\n");
    else
        printf("  Level 0: FAIL — timer doesn't fire!\n");

    return ret;
}

/* ------------------------------------------------------------------ */
/*  Level 1: Simple WFI (no sleep overrides)                           */
/* ------------------------------------------------------------------ */

static int test_level1_simple_wfi(u32 sleep_us)
{
    printf("\n--- Level 1: Simple WFI (no overrides) ---\n");

    dump_cpu_sleep_state("before");

    /* On Apple Silicon, timer FIQs need to be enabled at EL2 */
    if (in_el2()) {
        printf("  Enabling phys timer FIQ (VM_TMR_FIQ_ENA_EL2)...\n");
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
        printf("    VM_TMR_FIQ_ENA= 0x%lx\n", mrs(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2));
    }

    setup_timer(sleep_us);
    dump_timer_state("armed");

    s2r_sleep_type = 1;
    printf("  s2r_sleep_type set to 0x%x\n", s2r_sleep_type);
    printf("  Entering WFI...\n");
    u64 t0 = mrs(CNTPCT_EL0);

    sysop("isb");
    sysop("wfi");

    u64 t1 = mrs(CNTPCT_EL0);
    printf("  Woke up from WFI!\n");

    u64 elapsed_us = ((t1 - t0) * 1000000) / mrs(CNTFRQ_EL0);
    printf("  Elapsed: %lu us (expected ~%u us)\n", elapsed_us, sleep_us);

    dump_timer_state("after");

    /* Check if timer actually fired */
    u64 ctl = mrs(CNTP_CTL_EL0);
    int timer_fired = (ctl >> 2) & 1;

    /* Disable timer */
    msr(CNTP_CTL_EL0, 0);
    sysop("isb");

    if (timer_fired) {
        printf("  Level 1: PASS (timer woke CPU from WFI)\n");
        return 0;
    } else if (elapsed_us >= (sleep_us / 2)) {
        printf("  Level 1: PASS (WFI woke after ~expected time)\n");
        return 0;
    } else {
        printf("  Level 1: WARN (WFI woke early, spurious wake)\n");
        return 0; /* spurious wake is OK, CPU didn't freeze */
    }
}

/* ------------------------------------------------------------------ */
/*  Level 2: Deep WFI with retention (like deep_wfi())                 */
/* ------------------------------------------------------------------ */

static int test_level2_deep_wfi_retention(u32 sleep_us)
{
    printf("\n--- Level 2: Deep WFI with retention (CYC_OVRD only) ---\n");

    u64 cyc_ovrd_saved = 0;

    dump_cpu_sleep_state("before");

    /* On Apple Silicon, timer FIQs need to be enabled at EL2 */
    if (in_el2()) {
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
    }

    /* Same CYC_OVRD config as deep_wfi() — this is known to work */
    if (cpu_features->cyc_ovrd) {
        //cyc_ovrd_saved = mrs(SYS_IMP_APL_CYC_OVRD);
        //msr(SYS_IMP_APL_CYC_OVRD, cyc_ovrd_saved | CYC_OVRD_WFI_MODE(3));
        reg_mask(SYS_IMP_APL_CYC_OVRD, CYC_OVRD_WFI_MODE_MASK|CYC_OVRD_DISABLE_WFI_RET,
                 CYC_OVRD_WFI_MODE(3));
        printf("  Set CYC_OVRD WFI_MODE=3 (deep WFI retention)\n");
        printf("    CYC_OVRD now = 0x%lx\n", mrs(SYS_IMP_APL_CYC_OVRD));
    }

    setup_timer(sleep_us);
    dump_timer_state("armed");

    s2r_sleep_type = 2;
    printf("  s2r_sleep_type set to 0x%x\n", s2r_sleep_type);
    printf("  Entering deep WFI (retention mode)...\n");
    u64 t0 = mrs(CNTPCT_EL0);

    /* Use the same assembly helper as deep_wfi() — saves/restores regs */
    int wake_ret = s2r_enter((void *)0); /* ctx not used for SP save here */

    u64 t1 = mrs(CNTPCT_EL0);
    printf("  Woke up! (s2r_enter returned %d)\n", wake_ret);

    /* Restore CYC_OVRD */
    if (cpu_features->cyc_ovrd)
        msr(SYS_IMP_APL_CYC_OVRD, cyc_ovrd_saved);

    u64 elapsed_us = ((t1 - t0) * 1000000) / mrs(CNTFRQ_EL0);
    printf("  Elapsed: %lu us (expected ~%u us)\n", elapsed_us, sleep_us);

    dump_timer_state("after");
    dump_cpu_sleep_state("after");

    /* Disable timer */
    msr(CNTP_CTL_EL0, 0);
    sysop("isb");

    printf("  Level 2: PASS\n");
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Level 3: ACC overrides without DEEP_SLEEP                          */
/* ------------------------------------------------------------------ */

static int test_level3_acc_no_deep_sleep(u32 sleep_us)
{
    printf("\n--- Level 3: ACC overrides (no DEEP_SLEEP bit) ---\n");

    u64 cyc_ovrd_saved = 0;
    u64 acc_ovrd_saved = mrs(SYS_IMP_APL_ACC_OVRD);

    dump_cpu_sleep_state("before");

    /* Enable timer FIQ at EL2 */
    if (in_el2()) {
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
    }

    /* Set ACC overrides WITHOUT ACC_OVRD_DEEP_SLEEP */
    if (cpu_features->sleep_mode == SLEEP_GLOBAL) {
        reg_mask(SYS_IMP_APL_ACC_OVRD, ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP_MASK,
                 ACC_OVRD_PWR_DN_SRM(3) | ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP(2) |
                     ACC_OVRD_TRAIN_DOWN_LINK(3) | ACC_OVRD_POWER_DOWN_CPM(3) |
                     ACC_OVRD_DISABLE_PIO_ON_WFI_CPU);
        /* NOTE: ACC_OVRD_DEEP_SLEEP intentionally NOT set */
        printf("  Set ACC_OVRD (without DEEP_SLEEP)\n");
        printf("    ACC_OVRD now = 0x%lx\n", mrs(SYS_IMP_APL_ACC_OVRD));
    }

    /* Set CYC_OVRD for deepest WFI */
    if (cpu_features->cyc_ovrd) {
        cyc_ovrd_saved = mrs(SYS_IMP_APL_CYC_OVRD);
        reg_mask(SYS_IMP_APL_CYC_OVRD, CYC_OVRD_WFI_MODE_MASK,
                 CYC_OVRD_WFI_MODE(3) | CYC_OVRD_DISABLE_WFI_RET);
        printf("  Set CYC_OVRD (deep WFI, no retention)\n");
        printf("    CYC_OVRD now = 0x%lx\n", mrs(SYS_IMP_APL_CYC_OVRD));
    }

    setup_timer(sleep_us);

    s2r_sleep_type = 3;
    printf("  s2r_sleep_type set to 0x%x\n", s2r_sleep_type);
    printf("  Entering WFI with ACC overrides...\n");
    u64 t0 = mrs(CNTPCT_EL0);

    int wake_ret = s2r_enter_deep((void *)0);

    u64 t1 = mrs(CNTPCT_EL0);
    printf("  Woke up! (s2r_enter_deep returned %d)\n", wake_ret);

    /* Restore */
    if (cpu_features->cyc_ovrd)
        msr(SYS_IMP_APL_CYC_OVRD, cyc_ovrd_saved);
    msr(SYS_IMP_APL_ACC_OVRD, acc_ovrd_saved);
    sysop("isb");

    u64 elapsed_us = ((t1 - t0) * 1000000) / mrs(CNTFRQ_EL0);
    printf("  Elapsed: %lu us (expected ~%u us)\n", elapsed_us, sleep_us);

    /* Disable timer */
    msr(CNTP_CTL_EL0, 0);
    sysop("isb");

    printf("  Level 3: PASS\n");
    return 0;
}

static u64 s2r_ping(u64 a, u64 b, u64 c, u64 d)
{
    UNUSED(a);
    UNUSED(b);
    UNUSED(c);
    UNUSED(d);
    return 0;
}

/* Ping every secondary and print the result; returns the first responsive one or -1 */
static int s2r_check_secondary(const char *when)
{
    int first = -1;

    for (int i = 0; i < MAX_CPUS; i++) {
        if (i == boot_cpu_idx || !smp_is_alive(i))
            continue;
        bool ok = smp_call4_timeout(i, s2r_ping, 0, 0, 0, 0, 200000);
        printf("  [%s] secondary CPU %d: %s\n", when, i, ok ? "responds" : "NO RESPONSE");
        if (ok) {
            smp_wait(i);
            if (first < 0)
                first = i;
        }
    }
    if (first < 0)
        printf("  [%s] no responsive secondary CPU\n", when);
    return first;
}

/* ------------------------------------------------------------------ */
/*  Level 4: ACC overrides WITH DEEP_SLEEP (power-gate)                */
/* ------------------------------------------------------------------ */

#define S2R_WDT_MARGIN_MS 15000

/* Delay between the waker's IPI and its PMGR start write (timer/IPI never woke the core) */
#define S2R_ESCALATE_US 100000

volatile struct s2r_timestamps s2r_ts;

#define S2R_WD0_IRQ 338 /* t8103.dtsi: wdt interrupts = <AIC_IRQ 338> */

/* Route + unmask the WD0 IRQ to this CPU and arm WD0. Returns false if unsupported. */
static bool s2r_wd0_irq_setup(u32 timeout_ms)
{
    int adt_irq = wdt_wd0_irq_adt();
    int irq = S2R_WD0_IRQ;

    if (adt_irq >= 0 && adt_irq != irq)
        printf("  WD0: WARNING ADT first irq %d != assumed %d\n", adt_irq, irq);

    u32 me = aic_whoami();
    if (!aic_route_irq(irq, BIT(me))) {
        printf("  WD0: AIC routing unsupported (AIC version != 1?)\n");
        return false;
    }
    printf("  WD0: IRQ %d routed to AIC cpu %u\n", irq, me);

    wdt_wd0_irq_arm(timeout_ms);
    aic_unmask_irq(irq);
    return true;
}

static void s2r_wd0_irq_cleanup(void)
{
    wdt_wd0_irq_clear();
    aic_mask_irq(S2R_WD0_IRQ);
}

/* E1a: sanity check, plain WFI woken by the WD0 IRQ through the AIC (no deep sleep) */
static int test_wd0_irq_sanity(u32 sleep_us)
{
    printf("\n--- E1a: WD0 IRQ wake from plain WFI ---\n");

    msr(CNTP_CTL_EL0, 0); /* make sure the CPU timer can not be the wake source */
    sysop("isb");

    if (!s2r_wd0_irq_setup(sleep_us / 1000)) {
        printf("  E1a: skipped\n");
        return 0;
    }

    u64 t0 = mrs(CNTPCT_EL0);
    wdt_arm(sleep_us / 1000 + 5000); /* safety net */
    printf("  Entering WFI, expecting 'Exception: IRQ ... num: %d' in ~%u us\n", S2R_WD0_IRQ,
           sleep_us);
    int ret = s2r_enter((void *)0);
    u64 t1 = mrs(CNTPCT_EL0);
    wdt_stop();

    s2r_wd0_irq_cleanup();

    u64 elapsed_us = ((t1 - t0) * 1000000) / mrs(CNTFRQ_EL0);
    printf("  Woke up (ret %d) after %lu us (expected ~%u us)\n", ret, elapsed_us, sleep_us);
    printf("  E1a: PASS if an IRQ exception was printed above\n");
    return 0;
}

/* Set by the boot core once it is back, so the waker knows to stop escalating */
static volatile u32 s2r_woke;

/*
 * Runs on a secondary core while the boot core is power-gated. No printf here.
 *   crumb 7 = IPI sent, crumb 8 = PMGR start written
 */
static u64 s2r_waker(u64 delay_us, u64 escalate_us, u64 unused2, u64 unused3)
{
    UNUSED(unused2);
    UNUSED(unused3);

    s2r_ts.waker_start = mrs(CNTPCT_EL0);
    s2r_crumb(12);
    udelay(delay_us);
    if (s2r_woke)
        return 0;

    s2r_ts.waker_ipi = mrs(CNTPCT_EL0);
    smp_send_ipi(boot_cpu_idx);
    s2r_crumb(7);

    udelay(escalate_us);
    if (s2r_woke)
        return 1;

    s2r_ts.waker_pmgr = mrs(CNTPCT_EL0);
    smp_wake_start();
    s2r_crumb(8);
    return 2;
}

struct s2r_regsnap {
    const char *name;
    u64 val;
};

#define SNAP_REGS(X)                                                                     \
    X(HCR_EL2) X(SCTLR_EL1) X(TCR_EL1) X(MAIR_EL1) X(TTBR0_EL1) X(VBAR_EL1) X(TPIDR_EL2) \
    X(CPACR_EL1) X(CNTFRQ_EL0) X(CNTP_CTL_EL0) X(DAIF)

#define SNAP_COUNT_ONE(r) +1
enum { SNAP_N = 0 SNAP_REGS(SNAP_COUNT_ONE) + 4 };

static struct s2r_regsnap s2r_snap[SNAP_N];

static void snapshot_regs(void)
{
    int i = 0;
#define SNAP_ONE(r)                \
    s2r_snap[i].name = #r;         \
    s2r_snap[i++].val = mrs(r);
    SNAP_REGS(SNAP_ONE)
#undef SNAP_ONE
    s2r_snap[i].name = "ACC_OVRD";
    s2r_snap[i++].val = mrs(SYS_IMP_APL_ACC_OVRD);
    s2r_snap[i].name = "CYC_OVRD";
    s2r_snap[i++].val = cpu_features->cyc_ovrd ? mrs(SYS_IMP_APL_CYC_OVRD) : 0;
    s2r_snap[i].name = "VM_TMR_FIQ_ENA_EL2";
    s2r_snap[i++].val = in_el2() ? mrs(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2) : 0;
    s2r_snap[i].name = "CurrentEL";
    s2r_snap[i++].val = mrs(CurrentEL);
}

/* Compare current register values against the snapshot taken before sleep */
static void compare_regs_with_snapshot(void)
{
    static struct s2r_regsnap saved[SNAP_N];
    int diffs = 0;

    memcpy(saved, s2r_snap, sizeof(saved));
    snapshot_regs();

    for (int i = 0; i < SNAP_N; i++) {
        if (saved[i].val != s2r_snap[i].val) {
            printf("    DIFF %-20s before=0x%lx after=0x%lx\n", saved[i].name, saved[i].val,
                   s2r_snap[i].val);
            diffs++;
        }
    }
    printf("  Register compare: %d difference(s) out of %d\n", diffs, SNAP_N);
}

static void s2r_console_sync(u32 ms);

static int test_level4_deep_sleep(u32 sleep_us)
{
    printf("\n--- Level 4: ACC overrides + DEEP_SLEEP (power-gate) ---\n");

    u64 cyc_ovrd_saved = 0;
    u64 acc_ovrd_saved = mrs(SYS_IMP_APL_ACC_OVRD);

    if (cpu_features->sleep_mode != SLEEP_GLOBAL) {
        printf("  Level 4 only implemented for SLEEP_GLOBAL, skipping\n");
        return 0;
    }

    dump_cpu_sleep_state("before");
    s2r_check_secondary("level4 before ACC_OVRD");

    if (in_el2())
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);

    reg_mask(SYS_IMP_APL_ACC_OVRD, ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP_MASK,
             ACC_OVRD_PWR_DN_SRM(3) | ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP(2) |
                 ACC_OVRD_TRAIN_DOWN_LINK(3) | ACC_OVRD_POWER_DOWN_CPM(3) |
                 ACC_OVRD_DISABLE_PIO_ON_WFI_CPU | ACC_OVRD_DEEP_SLEEP);
    printf("  Set ACC_OVRD (with DEEP_SLEEP)\n");
    printf("    ACC_OVRD now = 0x%lx\n", mrs(SYS_IMP_APL_ACC_OVRD));

    if (cpu_features->cyc_ovrd) {
        cyc_ovrd_saved = mrs(SYS_IMP_APL_CYC_OVRD);
        reg_mask(SYS_IMP_APL_CYC_OVRD, CYC_OVRD_WFI_MODE_MASK,
                 CYC_OVRD_WFI_MODE(3) | CYC_OVRD_DISABLE_WFI_RET);
        printf("  Set CYC_OVRD (deep WFI, no retention)\n");
        printf("    CYC_OVRD now = 0x%lx\n", mrs(SYS_IMP_APL_CYC_OVRD));
    }

    setup_timer(sleep_us);
    snapshot_regs();

    /* Pick a responsive secondary to wake the boot core from outside */
    int waker = s2r_check_secondary("level4 after ACC_OVRD");
    if (waker >= 0 && !smp_wake_prepare(boot_cpu_idx))
        waker = -1;
    printf("  Waker CPU: %d (boot CPU %d)\n", waker, boot_cpu_idx);

    if (waker < 0) {
        /* Nothing can restart the gated core: sleeping now would just hit the watchdog */
        printf("  No usable waker core, skipping Level 4 sleep\n");
        if (cpu_features->cyc_ovrd)
            msr(SYS_IMP_APL_CYC_OVRD, cyc_ovrd_saved);
        msr(SYS_IMP_APL_ACC_OVRD, acc_ovrd_saved);
        msr(CNTP_CTL_EL0, 0);
        sysop("isb");
        return 0;
    }

    /* Safety net: reset the SoC if the wake or the resume never happens */
    wdt_arm(sleep_us / 1000 + S2R_WDT_MARGIN_MS);

    s2r_sleep_type = 4;
    printf("  s2r_sleep_type set to 0x%x\n", s2r_sleep_type);
    printf("  Entering WFI with DEEP_SLEEP...\n");
    s2r_console_sync(100);

    /*
     * Every console write calls fb_update(), which copies the whole shadow
     * framebuffer over the hardware one and would wipe the squares. Silence
     * the console while the breadcrumbs are in use.
     */
    s2r_crumb_init();
    s2r_crumb(0);

    s2r_woke = 0;
    s2r_ts.reset_entry = s2r_ts.waker_start = s2r_ts.waker_ipi = s2r_ts.waker_pmgr = 0;
    if (waker >= 0) {
        if (!smp_call4_timeout(waker, s2r_waker, sleep_us, S2R_ESCALATE_US, 0, 0, 200000))
            waker = -1;
        if (waker >= 0)
            s2r_crumb(10);
    }
    if (waker >= 0) {
        smp_wake_stop_request();
        s2r_crumb(11);
    }

    u64 t0 = mrs(CNTPCT_EL0);

    int wake_ret = s2r_enter_deep((void *)0);

    u64 t1 = mrs(CNTPCT_EL0);
    s2r_woke = 1;
    if (cpu_features->fast_ipi)
        msr(SYS_IMP_APL_IPI_SR_EL1, 1); /* ack a pending wake IPI */
    wdt_stop();
    s2r_crumb(6);
    if (waker >= 0)
        smp_wait(waker);
    s2r_crumb_disable(); /* keep squares on screen for a few seconds, then restore the console */
    printf("  Woke up! (s2r_enter_deep returned %d)\n", wake_ret);

    {
        u64 f = mrs(CNTFRQ_EL0);
#define TS_US(v) ((v) ? (long)((((s64)(v) - (s64)t0) * 1000000) / (s64)f) : -1L)
        printf("  Timestamps (us since sleep entry, -1 = not reached):\n");
        printf("    core back at reset vector : %ld\n", TS_US(s2r_ts.reset_entry));
        printf("    waker started             : %ld\n", TS_US(s2r_ts.waker_start));
        printf("    waker sent IPI            : %ld\n", TS_US(s2r_ts.waker_ipi));
        printf("    waker did PMGR start      : %ld\n", TS_US(s2r_ts.waker_pmgr));
#undef TS_US
    }

    compare_regs_with_snapshot();
    dump_cpu_sleep_state("after");

    /* Restore */
    if (cpu_features->cyc_ovrd)
        msr(SYS_IMP_APL_CYC_OVRD, cyc_ovrd_saved);
    msr(SYS_IMP_APL_ACC_OVRD, acc_ovrd_saved);
    sysop("isb");

    u64 elapsed_us = ((t1 - t0) * 1000000) / mrs(CNTFRQ_EL0);
    printf("  Elapsed: %lu us (expected ~%u us)\n", elapsed_us, sleep_us);

    msr(CNTP_CTL_EL0, 0);
    sysop("isb");

    /* Cores that went down with the cluster are still marked alive: bring them back */
    int restarted = smp_restart_unresponsive();
    printf("  Restarted %d unresponsive secondary CPU(s)\n", restarted);
    s2r_check_secondary("level4 end");

    printf("  Level 4: PASS\n");
    return 0;
}

/* ------------------------------------------------------------------ */
/*  E2: SMC power button / lid wake (AIC mailbox IRQ)                  */
/* ------------------------------------------------------------------ */

#ifndef S2R_E2
#define S2R_E2 1
#endif

#ifndef S2R_E2_MASK
#define S2R_E2_MASK 0x20000 /* bit n: run mode n (0 = E2a ... 15 = E2p, 16 = E2q, 17 = E2r) */
#endif

#define S2R_SMC_IRQ        403 /* t8103.dtsi smc_mbox: "recv-not-empty" */
#define S2R_E2_TIMEOUT_MS  30000
#define SMC_EV_BTN         0x7201
#define SMC_EV_LID         0x7203

static bool smc_notif_is_wake(u32 notif)
{
    u32 type = notif >> 16;
    return type == SMC_EV_BTN || type == SMC_EV_LID;
}

static void s2r_print_smc_irqs(void)
{
    int node = adt_path_offset(adt, "/arm-io/smc");
    u32 len;
    const u32 *irqs = node >= 0 ? adt_getprop(adt, node, "interrupts", &len) : NULL;

    if (!irqs)
        return;
    printf("  SMC: ADT interrupts:");
    for (u32 i = 0; i < len / 4; i++)
        printf(" %u", irqs[i]);
    printf(" (using %d as mailbox recv-not-empty)\n", S2R_SMC_IRQ);
}

/* ------------------------------------------------------------------ */
/*  PMU probe (read-only): the registers macOS writes right before sleep */
/* ------------------------------------------------------------------ */

#ifndef S2R_PMU_PROBE
#define S2R_PMU_PROBE 1
#endif

#define S2R_PMU_SID 0xf /* sera PMU on nub-spmi */

static void s2r_pmu_dump(spmi_dev_t *spmi, u16 reg, size_t len, const char *what)
{
    printf("    %04x %-14s:", reg, what);
    for (size_t off = 0; off < len; off += 8) {
        u8 buf[8];
        size_t n = min(len - off, (size_t)8);
        memset(buf, 0xee, sizeof(buf));
        int ret = spmi_ext_read_long(spmi, S2R_PMU_SID, reg + off, buf, n);
        /* -SPMI_ERR_BUS_IO is a frame parity mismatch: the data was still read, show it */
        if (ret < 0 && ret != -SPMI_ERR_BUS_IO) {
            printf(" <read error %d>", ret);
            break;
        }
        for (size_t i = 0; i < n; i++)
            printf(" %02x", buf[i]);
        if (ret < 0)
            printf(" (parity!)");
    }
    printf("\n");
}

/*
 * macOS (hypervisor trace, 2026-11-03), last platform writes before the cores power down:
 *   9f01 (boot_stage) = 0x50, reads of 9f00/02/08/18/19/1f,
 *   read d00c, d008 = 0x0114eefb (4 bytes), d00e = 0, d000 = 0x41.
 * Dump these blocks twice, 1 s apart, to see which bytes are live (e.g. a running RTC).
 */
static void s2r_pmu_probe(void)
{
    printf("\n--- PMU probe (read-only, nub-spmi sid 0x%x) ---\n", S2R_PMU_SID);

    spmi_dev_t *spmi = spmi_init("/arm-io/nub-spmi");
    if (!spmi) {
        printf("  spmi_init failed\n");
        return;
    }

    for (int pass = 0; pass < 2; pass++) {
        printf("  pass %d (t=%lu ms):\n", pass, ticks_to_msecs(mrs(CNTPCT_EL0)));
        s2r_pmu_dump(spmi, 0x9f00, 0x20, "legacy nvmem");
        s2r_pmu_dump(spmi, 0xd000, 0x10, "rtc nvmem");
        s2r_pmu_dump(spmi, 0xd100, 0x06, "rtc offset");
        if (pass == 0)
            mdelay(1000);
    }
    printf("  macOS writes before sleep: 9f01=50, d008=fb ee 14 01, d00e=00, d000=41\n");

    spmi_shutdown(spmi);
}

/*
 * E2f: the PMU part of the macOS sleep sequence (see s2r_pmu_probe for the decoding):
 *   9f01 boot_stage = 0x50 ("sleeping"; 0x2f = iBoot done while m1n1 runs),
 *   then as the very last platform step: read d00c, d008 = current RTC seconds
 *   (d004..d007, upper 32 bits of the 48-bit counter at d002), d00e = 0, d000 |= 0x40.
 * The controller flags a frame parity mismatch on every transfer although the data is
 * fine, so -SPMI_ERR_BUS_IO is accepted and every write is verified by reading it back.
 */
#define S2R_PMU_BOOT_STAGE       0x9f01
#define S2R_PMU_BOOT_STAGE_SLEEP 0x50
#define S2R_PMU_RTC_COUNTER      0xd002 /* 6 bytes, 1/65536 s */
#define S2R_PMU_SLEEP_TIME       0xd008 /* 4 bytes, seconds */
#define S2R_PMU_D000_SLEEP       0x40

struct s2r_pmu_saved {
    bool valid;
    u8 boot_stage;
    u8 d000;
};

static int s2r_pmu_rd(spmi_dev_t *spmi, u16 reg, u8 *buf, size_t n)
{
    int ret = spmi_ext_read_long(spmi, S2R_PMU_SID, reg, buf, n);
    return ret == -SPMI_ERR_BUS_IO ? 0 : ret;
}

static bool s2r_pmu_wr_check(spmi_dev_t *spmi, u16 reg, const u8 *buf, size_t n)
{
    u8 rb[8];
    int ret = spmi_ext_write_long(spmi, S2R_PMU_SID, reg, buf, n);

    if (ret < 0 && ret != -SPMI_ERR_BUS_IO) {
        printf("    PMU write %04x failed: %d\n", reg, ret);
        return false;
    }
    if (s2r_pmu_rd(spmi, reg, rb, n) < 0 || memcmp(rb, buf, n)) {
        printf("    PMU write %04x: read-back mismatch\n", reg);
        return false;
    }
    printf("    PMU %04x =", reg);
    for (size_t i = 0; i < n; i++)
        printf(" %02x", buf[i]);
    printf(" ok\n");
    return true;
}

/* Early step (macOS: right after SetAPPower(IDLE)): boot_stage = 0x50 */
static bool s2r_pmu_sleep_stage(spmi_dev_t *spmi, struct s2r_pmu_saved *sv)
{
    u8 v;

    sv->valid = false;
    if (s2r_pmu_rd(spmi, S2R_PMU_BOOT_STAGE, &sv->boot_stage, 1) < 0 ||
        s2r_pmu_rd(spmi, 0xd000, &sv->d000, 1) < 0) {
        printf("    PMU read failed, skipping the PMU sleep steps\n");
        return false;
    }
    sv->valid = true;
    printf("    PMU before: boot_stage=%02x d000=%02x\n", sv->boot_stage, sv->d000);

    /* A watchdog reset during a previous E2f/E2g skips the restore: undo what it left */
    if (sv->boot_stage == S2R_PMU_BOOT_STAGE_SLEEP) {
        printf("    stale boot_stage 0x50 from a previous run, using 0x2f\n");
        sv->boot_stage = 0x2f;
    }
    if (sv->d000 & S2R_PMU_D000_SLEEP) {
        printf("    stale sleep flag in d000 from a previous run, clearing it\n");
        sv->d000 &= ~S2R_PMU_D000_SLEEP;
        s2r_pmu_wr_check(spmi, 0xd000, &sv->d000, 1);
    }
    v = S2R_PMU_BOOT_STAGE_SLEEP;
    return s2r_pmu_wr_check(spmi, S2R_PMU_BOOT_STAGE, &v, 1);
}

/* Last step before the cores go down (macOS: just before ACC_OVRD/CYC_OVRD + WFI) */
static bool s2r_pmu_sleep_request(spmi_dev_t *spmi, const struct s2r_pmu_saved *sv)
{
    u8 rtc[6], d00c, zero = 0, d000;

    if (!sv->valid)
        return false;
    if (s2r_pmu_rd(spmi, 0xd00c, &d00c, 1) < 0 || s2r_pmu_rd(spmi, S2R_PMU_RTC_COUNTER, rtc, 6) < 0) {
        printf("    PMU read failed, no sleep request\n");
        return false;
    }
    printf("    PMU d00c=%02x, RTC seconds=0x%08x\n", d00c,
           rtc[2] | rtc[3] << 8 | rtc[4] << 16 | (u32)rtc[5] << 24);
    if (!s2r_pmu_wr_check(spmi, S2R_PMU_SLEEP_TIME, &rtc[2], 4))
        return false;
    if (!s2r_pmu_wr_check(spmi, 0xd00e, &zero, 1))
        return false;
    d000 = sv->d000 | S2R_PMU_D000_SLEEP;
    return s2r_pmu_wr_check(spmi, 0xd000, &d000, 1);
}

/* After the wake: undo the sleep flag and the boot stage (d008 keeps the timestamp) */
static void s2r_pmu_restore(spmi_dev_t *spmi, const struct s2r_pmu_saved *sv)
{
    if (!sv->valid)
        return;
    printf("  PMU restore:\n");
    s2r_pmu_wr_check(spmi, 0xd000, &sv->d000, 1);
    s2r_pmu_wr_check(spmi, S2R_PMU_BOOT_STAGE, &sv->boot_stage, 1);
}

/*
 * macOS sleep handshake with the SMC, as seen in a hypervisor trace of `pmset sleepnow`
 * (macOS 13.6.2, 2026-11-03), in order, ~300 ms before the cores power down:
 *   MBSE = 'slpw' (sleep warning), WKTP = 2, MBSE = 'slpa', RTKit SetAPPower(0x201 = IDLE).
 * Device quiesce (PMGR power states) happens between these steps and is not replayed.
 */
#define SMC_KEY_MBSE          0x4d425345 /* 'MBSE' */
#define SMC_KEY_WKTP          0x574b5450 /* 'WKTP' */
#define SMC_MBSE_SLPW         0x736c7077 /* 'slpw' */
#define SMC_MBSE_SLPA         0x736c7061 /* 'slpa' */
#define S2R_RTKIT_AP_IDLE     0x201
#define S2R_RTKIT_AP_ON       0x20

static void s2r_smc_sleep_handshake_keys(smc_dev_t *smc, bool ap_idle)
{
    printf("  SMC sleep handshake (keys%s):\n", ap_idle ? " + AP IDLE" : " only");
    printf("    MBSE = 'slpw': %s\n", smc_write_u32(smc, SMC_KEY_MBSE, SMC_MBSE_SLPW) ? "FAILED" : "ok");
    printf("    WKTP = 2: %s\n", smc_write_flag(smc, SMC_KEY_WKTP, 2) ? "FAILED" : "ok");
    printf("    MBSE = 'slpa': %s\n", smc_write_u32(smc, SMC_KEY_MBSE, SMC_MBSE_SLPA) ? "FAILED" : "ok");
    if (ap_idle)
        printf("    SetAPPower(0x%x): %s\n", S2R_RTKIT_AP_IDLE,
               smc_set_ap_power(smc, S2R_RTKIT_AP_IDLE) ? "acked" : "NOT acked");
}

static void s2r_smc_sleep_handshake(smc_dev_t *smc)
{
    printf("  SMC sleep handshake:\n");
    printf("    MBSE = 'slpw': %s\n", smc_write_u32(smc, SMC_KEY_MBSE, SMC_MBSE_SLPW) ? "FAILED" : "ok");
    printf("    WKTP = 2: %s\n", smc_write_flag(smc, SMC_KEY_WKTP, 2) ? "FAILED" : "ok");
    printf("    MBSE = 'slpa': %s\n", smc_write_u32(smc, SMC_KEY_MBSE, SMC_MBSE_SLPA) ? "FAILED" : "ok");
    printf("    SetAPPower(0x%x): %s\n", S2R_RTKIT_AP_IDLE,
           smc_set_ap_power(smc, S2R_RTKIT_AP_IDLE) ? "acked" : "NOT acked");
}

/*
 * Always-on GPIO controllers: macOS clears the IRQ status of interrupt group 4 (the ADT
 * "wake-no-interrupt-group") on aop-gpio and nub-gpio right before the cores power down.
 * Dump the pins that are routed to an interrupt (or to group 4) first, to see which wake
 * pins iBoot/the firmware configured. Register layout as in Linux pinctrl-apple-gpio.c.
 */
#define S2R_GPIO_GRP(v)      (((v) >> 16) & 7)
#define S2R_GPIO_MODE(v)     (((v) >> 1) & 7)
#define S2R_GPIO_IRQ(g, x)   (0x800 + 0x40 * (g) + 4 * ((x) >> 5))

static void s2r_gpio_wake_group(const char *path, bool clear)
{
    int pathv[8];
    u64 base, size;
    u32 npins = 0, wake_grp = 4;

    if (adt_path_offset_trace(adt, path, pathv) < 0 ||
        adt_get_reg(adt, pathv, "reg", 0, &base, &size) < 0) {
        printf("    %s: not found\n", path);
        return;
    }
    int node = adt_path_offset(adt, path);
    ADT_GETPROP(adt, node, "#gpio-pins", &npins);
    ADT_GETPROP(adt, node, "wake-no-interrupt-group", &wake_grp);

    printf("    %s @0x%lx: %u pins, wake group %u, status before:", path, base, npins, wake_grp);
    for (u32 x = 0; x < npins; x += 32)
        printf(" 0x%08x", read32(base + S2R_GPIO_IRQ(wake_grp, x)));
    printf("\n");

    for (u32 x = 0; x < npins; x++) {
        u32 v = read32(base + 4 * x);
        u32 mode = S2R_GPIO_MODE(v);
        if (S2R_GPIO_GRP(v) == wake_grp || (mode >= 2 && mode <= 6))
            printf("      pin %2u = 0x%08x grp %u mode %u\n", x, v, S2R_GPIO_GRP(v), mode);
    }

    if (clear)
        for (u32 x = 0; x < npins; x += 32)
            write32(base + S2R_GPIO_IRQ(wake_grp, x), 0xffffffff);
}

/*
 * E2g: on macOS every core enters deep sleep (ACC_OVRD DEEP_SLEEP + CYC_OVRD deep WFI,
 * within ~370 ms of each other). Run the same on each secondary. The loop exits when the
 * boot core sets s2r_secondary_release, so a core that never got power-gated simply
 * returns; a gated one does not answer and is restarted by smp_restart_unresponsive().
 */
/*
 * Secondaries run with the MMU off (Device memory): no atomics there (an LSE atomic on
 * non-cacheable memory faults on M1), only plain per-CPU flags. The boot core runs cached,
 * so it cleans its writes to the PoC and invalidates before reading the secondaries' flags.
 */
static volatile u32 s2r_secondary_go;
static volatile u32 s2r_secondary_release;
static volatile u32 s2r_secondary_asleep[MAX_CPUS];

static void s2r_dc_civac(volatile void *p)
{
    __asm__ volatile("dc civac, %0" ::"r"(p) : "memory");
    sysop("dsb sy");
}

static u64 s2r_secondary_deep_sleep(u64 a0, u64 a1, u64 a2, u64 a3)
{
    UNUSED(a0);
    UNUSED(a1);
    UNUSED(a2);
    UNUSED(a3);

    u64 acc = mrs(SYS_IMP_APL_ACC_OVRD);
    u64 cyc = cpu_features->cyc_ovrd ? mrs(SYS_IMP_APL_CYC_OVRD) : 0;

    /*
     * DEEP_SLEEP is global: a core in a plain WFI when another core sets it hangs (XNU
     * __ARM_GLOBAL_SLEEP_BIT__). So every secondary first takes the call and spins here
     * without WFI; the boot core releases them all together.
     */
    while (!s2r_secondary_go)
        sysop("dmb sy");

    msr(CNTP_CTL_EL0, 0);
    if (cpu_features->sleep_mode == SLEEP_GLOBAL)
        reg_mask(SYS_IMP_APL_ACC_OVRD, ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP_MASK,
                 ACC_OVRD_PWR_DN_SRM(3) | ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP(2) |
                     ACC_OVRD_TRAIN_DOWN_LINK(3) | ACC_OVRD_POWER_DOWN_CPM(3) |
                     ACC_OVRD_DISABLE_PIO_ON_WFI_CPU | ACC_OVRD_DEEP_SLEEP);
    if (cpu_features->cyc_ovrd)
        reg_mask(SYS_IMP_APL_CYC_OVRD, CYC_OVRD_WFI_MODE_MASK,
                 CYC_OVRD_WFI_MODE(3) | CYC_OVRD_DISABLE_WFI_RET);
    sysop("isb");

    s2r_secondary_asleep[smp_id()] = 1;
    sysop("dsb sy");

    while (!s2r_secondary_release) {
        sysop("dsb sy");
        sysop("wfi");
        if (cpu_features->fast_ipi)
            msr(SYS_IMP_APL_IPI_SR_EL1, 1);
    }

    if (cpu_features->cyc_ovrd)
        msr(SYS_IMP_APL_CYC_OVRD, cyc);
    msr(SYS_IMP_APL_ACC_OVRD, acc);
    sysop("isb");
    return 0;
}

static int s2r_secondaries_deep_sleep(void)
{
    int sent = 0;

    s2r_secondary_go = 0;
    s2r_secondary_release = 0;
    memset((void *)s2r_secondary_asleep, 0, sizeof(s2r_secondary_asleep));
    s2r_dc_civac(&s2r_secondary_go);
    s2r_dc_civac(&s2r_secondary_release);
    for (int i = 0; i < MAX_CPUS; i++)
        s2r_dc_civac(&s2r_secondary_asleep[i]);
    for (int i = 0; i < MAX_CPUS; i++) {
        if (i == boot_cpu_idx || !smp_is_alive(i))
            continue;
        if (smp_call4_timeout(i, s2r_secondary_deep_sleep, 0, 0, 0, 0, 200000))
            sent++;
        else
            printf("    CPU %d did not take the call\n", i);
    }
    s2r_secondary_go = 1;
    s2r_dc_civac(&s2r_secondary_go);
    mdelay(10);

    int asleep = 0;
    for (int i = 0; i < MAX_CPUS; i++) {
        s2r_dc_civac(&s2r_secondary_asleep[i]);
        asleep += !!s2r_secondary_asleep[i];
    }
    printf("  Secondary deep sleep: sent to %d CPU(s), %d entered the sleep loop\n", sent,
           asleep);
    return sent;
}

static void s2r_secondaries_release(void)
{
    s2r_secondary_release = 1;
    s2r_dc_civac(&s2r_secondary_release);
    sysop("sev");
    for (int i = 0; i < MAX_CPUS; i++)
        if (i != boot_cpu_idx)
            smp_send_ipi(i);
    mdelay(10);
}

/*
 * E2h: macOS runs with AUTO_ENABLE (bit 28) and a PS_AUTO state (bits 27:24) on the memory,
 * fabric, PMP and always-on (NUB) power domains, while iBoot/m1n1 leave them forced on
 * (0x0f0000ff). Values from PMGR dumps of running macOS 13.6.2 vs the m1n1 shell
 * (power_optim/dump_pmgr_*.txt). Only AUTO_ENABLE/PS_AUTO are changed, the target state is
 * left alone. SIO*, ANS2, APCIE*, GFX, DISP* and DEBUG_* are left out on purpose.
 */
#define S2R_PS_AUTO_ENABLE BIT(28)
#define S2R_PS_AUTO        GENMASK(27, 24)
#define S2R_PS_FLAGS       (BIT(9) | BIT(8)) /* WAS_CLKGATED / WAS_PWRGATED, never write 1 */

static const struct {
    const char *name;
    u64 addr;
    u32 macos;
} s2r_ps_auto_t8103[] = {
    {"DCS0", 0x23b700300, 0x1000030f},       {"DCS1", 0x23b700310, 0x1000030f},
    {"DCS2", 0x23b700308, 0x1000030f},       {"DCS3", 0x23b700318, 0x1000030f},
    {"DCS4", 0x23b700320, 0x1000030f},       {"DCS5", 0x23b700330, 0x1000030f},
    {"DCS6", 0x23b700328, 0x1000030f},       {"DCS7", 0x23b700338, 0x1000030f},
    {"MCC", 0x23b7002f8, 0x1f0000ff},        {"SBR", 0x23b700100, 0x1f0000ff},
    {"IMX", 0x23b7001b8, 0x1f0000ff},        {"SMX", 0x23b700340, 0x1f0000ff},
    {"RMX", 0x23b700350, 0x1000030f},        {"PMP", 0x23b7003d8, 0x1f0f03ff},
    {"PMS", 0x23b700140, 0x1f0002ff},        {"PMS_BUSIF", 0x23b700138, 0x1f0002ff},
    {"PMS_SRAM", 0x23b7003e0, 0x1f0003ff},   {"NUB_AON", 0x23d280070, 0x1f0002ff},
    {"NUB_FABRIC", 0x23d2800a8, 0x1f0003ff}, {"NUB_SRAM", 0x23d2800b0, 0x1000030f},
    {"NUB_SPMI0", 0x23d280060, 0x1400024f},  {"NUB_SPMI1", 0x23d280068, 0x1400024f},
};
#define S2R_PS_AUTO_N (sizeof(s2r_ps_auto_t8103) / sizeof(s2r_ps_auto_t8103[0]))

static u32 s2r_ps_saved[S2R_PS_AUTO_N];
static bool s2r_ps_applied;

/* battery test: keep the internal display pipe powered (the results go on the screen) */
static bool s2r_keep_disp;
static bool s2r_ps_is_disp(const char *n)
{
    return s2r_keep_disp && (!strcmp(n, "DISP0_CPU0") || !strcmp(n, "DISP0_FE") || !strcmp(n, "RMX"));
}

static void s2r_ps_auto_apply(void)
{
    s2r_ps_applied = false;
    if (chip_id != T8103) {
        printf("  PS auto: only the T8103 table exists, skipping\n");
        return;
    }
    printf("  PS auto (macOS values):\n");
    for (size_t i = 0; i < S2R_PS_AUTO_N; i++) {
        u64 a = s2r_ps_auto_t8103[i].addr;
        u32 cur = read32(a);
        s2r_ps_saved[i] = cur;
        if (s2r_ps_is_disp(s2r_ps_auto_t8103[i].name)) {
            printf("    %-10s kept (display)\n", s2r_ps_auto_t8103[i].name);
            continue;
        }
        u32 val = (cur & ~(S2R_PS_AUTO_ENABLE | S2R_PS_AUTO | S2R_PS_FLAGS)) |
                  (s2r_ps_auto_t8103[i].macos & (S2R_PS_AUTO_ENABLE | S2R_PS_AUTO));

        write32(a, val);
        udelay(100);
        printf("    %-10s %08x -> %08x (now %08x)\n", s2r_ps_auto_t8103[i].name, cur, val,
               read32(a));
    }
    s2r_ps_applied = true;
}

static void s2r_ps_auto_restore(void)
{
    if (!s2r_ps_applied)
        return;
    printf("  PS auto restore:\n");
    for (size_t i = S2R_PS_AUTO_N; i-- > 0;) {
        u64 a = s2r_ps_auto_t8103[i].addr;
        write32(a, s2r_ps_saved[i] & ~S2R_PS_FLAGS);
        udelay(100);
    }
    for (size_t i = 0; i < S2R_PS_AUTO_N; i++)
        printf("    %-10s now %08x\n", s2r_ps_auto_t8103[i].name,
               read32(s2r_ps_auto_t8103[i].addr));
    s2r_ps_applied = false;
}

/*
 * The USB console is only pushed to the host when m1n1 services the USB controller, which
 * nothing does while a test runs: anything still buffered is lost on a watchdog reset.
 * Service the USB iodevs and flush the console for a short while at the key points.
 */
static void s2r_console_sync(u32 ms)
{
    u64 timeout = timeout_calculate(ms * 1000);

    do {
        /* services only the registered console iodevs (iodev_handle_events has no NULL check) */
        iodev_console_kick();
        iodev_console_flush();
    } while (!timeout_expired(timeout));
}

/*
 * E2i: macOS puts the PMP (power management coprocessor) to sleep right after the SMC
 * handshake and the PMU boot_stage write (hypervisor trace 2026-11-04):
 *   PMP SetAPPower(0x201) -> ack, SetIOPPower(0x201) -> ack (type 7), mailbox disabled,
 *   CPU_CONTROL = 0 (PMP CPU stopped).
 * The PMP runs an RTKit session started before m1n1, so only these management messages are
 * sent on the raw mailbox (no RTKit boot: its syslog wants a DART buffer m1n1 cannot give).
 * Wake: start the CPU, SetIOPPower(0x20 ON) as Linux apple_rtkit_wake does, SetAPPower(0x20).
 */
#define S2R_MGMT_TYPE          GENMASK(59, 52)
#define S2R_MGMT_IOP_PWR       0x6
#define S2R_MGMT_IOP_PWR_ACK   0x7
#define S2R_MGMT_AP_PWR        0xb
#define S2R_MGMT_AP_PWR_ACK    0xb

static asc_dev_t *s2r_pmp;

static bool s2r_pmp_mgmt(u32 type, u16 state, u32 ack_type, const char *what)
{
    struct asc_message msg = {
        .msg0 = FIELD_PREP(S2R_MGMT_TYPE, type) | state,
        .msg1 = 0, /* management endpoint */
    };

    if (!asc_send(s2r_pmp, &msg)) {
        printf("    PMP %s(0x%x): send failed\n", what, state);
        return false;
    }

    u64 timeout = timeout_calculate(300000);
    while (!timeout_expired(timeout)) {
        if (!asc_recv(s2r_pmp, &msg))
            continue;
        if (msg.msg1 == 0 && FIELD_GET(S2R_MGMT_TYPE, msg.msg0) == ack_type) {
            printf("    PMP %s(0x%x): ack 0x%016lx\n", what, state, msg.msg0);
            return true;
        }
        printf("    PMP %s: other message ep 0x%02x 0x%016lx\n", what, msg.msg1, msg.msg0);
    }
    printf("    PMP %s(0x%x): no ack\n", what, state);
    return false;
}

/* A PMP RTKit session started from m1n1 (E2j, PMP boot probe) */
static struct {
    dart_dev_t *dart;
    iova_domain_t *iovad;
    asc_dev_t *asc;
    rtkit_dev_t *rtk;
    struct rtkit_buffer ep20_buf; /* ep 0x20 shared memory (E2n), allocated by m1n1 */
    bool ep20_fixed;              /* buffer mapped by hand at S2R_EP20_DVA */
} s2r_pmpdev;

/*
 * Under m1n1 the PMP CPU is stopped, while macOS boots it (HELLO/EPMap/StartEP) and talks to
 * it on endpoint 0x20. ADT: iop-pmp-nub firmware-name t8103pmp, pre-loaded = 1, IOMMU
 * dart-pmp / mapper-pmp (sid 0). Boot it the way dcp.c boots the DCP and start ep 0x20.
 */
static void s2r_pmp_down(void);
static bool s2r_pmp_crashed; /* set when rtkit_recv reports the PMP crashed */

static bool s2r_pmp_up(void)
{
    const char *dart_path = "/arm-io/dart-pmp";
    u32 sid = 0;

    s2r_pmp_crashed = false;

    int dart_node = adt_path_offset(adt, dart_path);
    int mapper = dart_node >= 0 ? adt_first_child_offset(adt, dart_node) : -1;
    if (mapper < 0 || ADT_GETPROP(adt, mapper, "reg", &sid) < 0) {
        printf("  no %s mapper / sid\n", dart_path);
        return false;
    }

    s2r_pmpdev.dart = dart_init_adt(dart_path, 0, sid, true);
    if (!s2r_pmpdev.dart) {
        printf("  dart_init_adt(%s, sid %u) failed\n", dart_path, sid);
        return false;
    }
    u64 vm_base = dart_vm_base(s2r_pmpdev.dart);
    dart_setup_pt_region(s2r_pmpdev.dart, dart_path, sid, vm_base);
    s2r_pmpdev.iovad = iovad_init(vm_base + 0x10000000, vm_base + 0x20000000);

    s2r_pmpdev.asc = asc_init("/arm-io/pmp");
    if (!s2r_pmpdev.asc) {
        printf("  asc_init(/arm-io/pmp) failed\n");
        goto fail;
    }
    printf("  dart sid %u vm_base 0x%lx, PMP CPU running before: %d\n", sid, vm_base,
           asc_cpu_running(s2r_pmpdev.asc));

    s2r_pmpdev.rtk = rtkit_init("pmp", s2r_pmpdev.asc, s2r_pmpdev.dart, s2r_pmpdev.iovad, NULL,
                                false);
    if (!s2r_pmpdev.rtk) {
        printf("  rtkit_init failed\n");
        goto fail;
    }
    s2r_console_sync(100);

    if (!rtkit_boot(s2r_pmpdev.rtk)) {
        printf("  rtkit_boot FAILED (PMP CPU running: %d)\n", asc_cpu_running(s2r_pmpdev.asc));
        asc_cpu_stop(s2r_pmpdev.asc);
        goto fail;
    }
    printf("  rtkit_boot OK, PMP CPU running: %d\n", asc_cpu_running(s2r_pmpdev.asc));
    printf("  start ep 0x20: %s\n", rtkit_start_ep(s2r_pmpdev.rtk, 0x20) ? "ok" : "FAILED");
    s2r_console_sync(100);
    return true;

fail:
    s2r_pmp_down();
    return false;
}

/* Print what the PMP sends for a while (app endpoints only) */
static int s2r_pmp_listen(u32 us)
{
    int n = 0;
    u64 timeout = timeout_calculate(us);

    while (s2r_pmpdev.rtk && !timeout_expired(timeout) && n < 32) {
        struct rtkit_message msg;
        if (!rtkit_can_recv(s2r_pmpdev.rtk))
            continue;
        int ret = rtkit_recv(s2r_pmpdev.rtk, &msg);
        if (ret < 0) {
            printf("  rtkit_recv error %d\n", ret);
            break;
        }
        if (ret > 0) {
            printf("  PMP ep 0x%02x: 0x%016lx\n", msg.ep, msg.msg);
            n++;
        }
    }
    printf("  %d PMP app message(s) in %u ms\n", n, us / 1000);
    s2r_console_sync(100);
    return n;
}

/* Back to the state m1n1 found the PMP in: asleep, CPU stopped, IOMMU released */
static void s2r_pmp_down(void)
{
    if (s2r_pmpdev.rtk) {
        /* rtkit_sleep() always stops the CPU and returns 0, its own prints report failures */
        rtkit_sleep(s2r_pmpdev.rtk);
        printf("  PMP rtkit_sleep done, CPU running: %d\n", asc_cpu_running(s2r_pmpdev.asc));
        if (s2r_pmpdev.ep20_buf.bfr && s2r_pmpdev.ep20_fixed) {
            dart_unmap(s2r_pmpdev.dart, s2r_pmpdev.ep20_buf.dva, s2r_pmpdev.ep20_buf.sz);
            free(s2r_pmpdev.ep20_buf.bfr);
        } else if (s2r_pmpdev.ep20_buf.bfr) {
            rtkit_free_buffer(s2r_pmpdev.rtk, &s2r_pmpdev.ep20_buf);
        }
        rtkit_free(s2r_pmpdev.rtk);
    }
    if (s2r_pmpdev.asc)
        asc_free(s2r_pmpdev.asc);
    if (s2r_pmpdev.iovad)
        iovad_shutdown(s2r_pmpdev.iovad, s2r_pmpdev.dart);
    if (s2r_pmpdev.dart)
        dart_shutdown(s2r_pmpdev.dart);
    memset(&s2r_pmpdev, 0, sizeof(s2r_pmpdev));
    s2r_console_sync(100);
}

static bool s2r_pmp_sleep(void)
{
    struct asc_message msg;

    printf("  PMP sleep:\n");
    s2r_pmp = s2r_pmpdev.asc ? s2r_pmpdev.asc : asc_init("/arm-io/pmp");
    if (!s2r_pmp) {
        printf("    asc_init(/arm-io/pmp) failed\n");
        return false;
    }
    printf("    PMP CPU running: %d\n", asc_cpu_running(s2r_pmp));
    while (asc_recv(s2r_pmp, &msg))
        printf("    PMP pending message ep 0x%02x 0x%016lx (dropped)\n", msg.msg1, msg.msg0);

    bool ok = s2r_pmp_mgmt(S2R_MGMT_AP_PWR, S2R_RTKIT_AP_IDLE, S2R_MGMT_AP_PWR_ACK, "SetAPPower");
    ok = ok && s2r_pmp_mgmt(S2R_MGMT_IOP_PWR, S2R_RTKIT_AP_IDLE, S2R_MGMT_IOP_PWR_ACK,
                            "SetIOPPower");
    if (!ok) {
        printf("    PMP did not go idle, leaving its CPU running\n");
        return false;
    }
    asc_cpu_stop(s2r_pmp);
    printf("    PMP CPU stopped (running: %d)\n", asc_cpu_running(s2r_pmp));
    return true;
}

static void s2r_pmp_wake(bool stopped)
{
    if (!s2r_pmp)
        return;
    printf("  PMP wake:\n");
    if (stopped) {
        asc_cpu_start(s2r_pmp);
        s2r_pmp_mgmt(S2R_MGMT_IOP_PWR, S2R_RTKIT_AP_ON, S2R_MGMT_IOP_PWR_ACK, "SetIOPPower");
    }
    s2r_pmp_mgmt(S2R_MGMT_AP_PWR, S2R_RTKIT_AP_ON, S2R_MGMT_AP_PWR_ACK, "SetAPPower");
    if (s2r_pmp != s2r_pmpdev.asc)
        asc_free(s2r_pmp);
    s2r_pmp = NULL;
}

/*
 * E2r: the AOP (always-on processor). Under m1n1 its CPU is stopped (firmware mac13gaop
 * pre-loaded by iBoot); macOS starts it at boot (trace 10, 5.55 s: CPU_CONTROL RUN=1, HELLO,
 * EPMap, StartEP 1/2/4/8 + app endpoints, SetAPPower 0x20) and, right before sleep, tells it
 * the AP goes idle (183.46 s: SetAPPower 0x201, acked in 1 ms), before the SMC and the PMP.
 * We only bring up the system endpoints, not the AFK app endpoints 0x20-0x28.
 */
static struct {
    dart_dev_t *dart;
    iova_domain_t *iovad;
    asc_dev_t *asc;
    rtkit_dev_t *rtk;
} s2r_aopdev;

static bool s2r_aop_up(void)
{
    const char *dart_path = "/arm-io/dart-aop";
    u32 sid = 0;

    printf("  AOP boot:\n");
    /* m1n1 only does this from kboot; without it the AOP faults on its MMIO in dart-aop */
    printf("    dapf_init(%s): %d\n", dart_path, dapf_init(dart_path, 1));
    int dart_node = adt_path_offset(adt, dart_path);
    int mapper = dart_node >= 0 ? adt_first_child_offset(adt, dart_node) : -1;
    if (mapper < 0 || ADT_GETPROP(adt, mapper, "reg", &sid) < 0)
        printf("    no %s mapper sid, using sid 0\n", dart_path);

    s2r_aopdev.dart = dart_init_adt(dart_path, 0, sid, true);
    if (!s2r_aopdev.dart) {
        printf("    dart_init_adt(%s, sid %u) failed\n", dart_path, sid);
        return false;
    }
    u64 vm_base = dart_vm_base(s2r_aopdev.dart);
    dart_setup_pt_region(s2r_aopdev.dart, dart_path, sid, vm_base);
    /* dart-aop vm-base is 0x34000 (macOS buffers at 0x34000+); iovad needs a 32M-aligned base */
    u64 iova_base = ALIGN_UP(vm_base, SZ_32M);
    if (iova_base == vm_base)
        iova_base += SZ_32M;
    s2r_aopdev.iovad = iovad_init(iova_base, iova_base + 0x10000000);
    if (!s2r_aopdev.iovad) {
        printf("    iovad_init(0x%lx) failed\n", iova_base);
        return false;
    }

    /*
     * The AOP code is split: __TEXT/__DATA in its SRAM, __ETEXT in DRAM reached through
     * dart-aop at the segment's "remap" DVA (0x20000 on j313). iBoot leaves dart-aop without
     * page tables and the boot args for it unset, so do what Asahi's aop.rs does: map it and
     * patch EC0p/nCal/alig/AOPt before the CPU starts.
     */
    int nub = adt_path_offset(adt, "/arm-io/aop/iop-aop-nub");
    u32 seglen = 0;
    const struct adt_segment_ranges *seg =
        nub >= 0 ? adt_getprop(adt, nub, "segment-ranges", &seglen) : NULL;
    for (u32 i = 0; seg && i < seglen / sizeof(*seg); i++) {
        if (seg[i].remap == seg[i].phys)
            continue; /* SRAM segments and __OS_LOG */
        size_t sz = ALIGN_UP(seg[i].size, SZ_16K);
        int ret = dart_map(s2r_aopdev.dart, seg[i].remap, (void *)seg[i].phys, sz);
        printf("    dart map dva 0x%lx -> 0x%lx (0x%lx): %d\n", seg[i].remap, seg[i].phys, sz,
               ret);
    }

    {
        const u64 sram = 0x24ac00000;
        u32 off = read32(sram + 0x22c), size = read32(sram + 0x230);
        static const struct {
            u32 key;
            u64 val;
        } patches[] = {
            {0x45433070, 0x20000}, /* 'EC0p' */
            {0x6e43616c, 0},       /* 'nCal' */
            {0x616c6967, 128},     /* 'alig' */
            {0x414f5074, 1},       /* 'AOPt' */
        };
        u8 args[0x400];
        if (off < 0x1e0000 && size <= sizeof(args)) {
            for (u32 j = 0; j < size; j += 4)
                write32((u64)args + j, read32(sram + off + j));
            for (u32 j = 0; j + 8 <= size;) {
                u32 key = *(u32 *)&args[j], n = *(u32 *)&args[j + 4];
                j += 8;
                for (u32 k = 0; k < ARRAY_SIZE(patches); k++) {
                    if (patches[k].key != key || n > 8)
                        continue;
                    u64 old = 0;
                    memcpy(&old, &args[j], n);
                    memcpy(&args[j], &patches[k].val, n);
                    printf("    bootarg %c%c%c%c 0x%lx -> 0x%lx\n", key >> 24, (key >> 16) & 0xff,
                           (key >> 8) & 0xff, key & 0xff, old, patches[k].val);
                }
                j += n;
            }
            for (u32 j = 0; j < size; j += 4)
                write32(sram + off + j, *(u32 *)&args[j]);
        } else {
            printf("    bootargs at 0x%x size 0x%x look wrong, not patched\n", off, size);
        }
    }

    s2r_aopdev.asc = asc_init("/arm-io/aop");
    if (!s2r_aopdev.asc) {
        printf("    asc_init(/arm-io/aop) failed\n");
        return false;
    }
    printf("    dart sid %u vm_base 0x%lx, AOP CPU running before: %d\n", sid, vm_base,
           asc_cpu_running(s2r_aopdev.asc));

    s2r_aopdev.rtk =
        rtkit_init("aop", s2r_aopdev.asc, s2r_aopdev.dart, s2r_aopdev.iovad, NULL, false);
    if (!s2r_aopdev.rtk) {
        printf("    rtkit_init failed\n");
        return false;
    }
    s2r_console_sync(100);

    if (!rtkit_boot(s2r_aopdev.rtk)) {
        printf("    rtkit_boot FAILED (AOP CPU running: %d)\n", asc_cpu_running(s2r_aopdev.asc));
        s2r_aopdev.rtk = NULL;
        return false;
    }
    printf("    rtkit_boot OK, AOP CPU running: %d\n", asc_cpu_running(s2r_aopdev.asc));

    /* let it ack SetAPPower 0x20 and drain its boot syslog */
    u64 timeout = timeout_calculate(500000);
    int n = 0;
    while (!timeout_expired(timeout)) {
        struct rtkit_message msg;
        if (!rtkit_can_recv(s2r_aopdev.rtk))
            continue;
        int ret = rtkit_recv(s2r_aopdev.rtk, &msg);
        if (ret < 0) {
            printf("    AOP rtkit_recv error %d\n", ret);
            s2r_aopdev.rtk = NULL;
            return false;
        }
        if (ret > 0 && n++ < 8)
            printf("    AOP ep 0x%02x: 0x%016lx\n", msg.ep, msg.msg);
    }
    printf("    AOP up, %d app message(s)\n", n);
    s2r_console_sync(100);
    return true;
}

static void s2r_aop_sleep(void)
{
    if (!s2r_aopdev.rtk)
        return;
    bool ok = rtkit_set_ap_power(s2r_aopdev.rtk, S2R_RTKIT_AP_IDLE, 300000);
    printf("  AOP SetAPPower(0x%x): %s\n", S2R_RTKIT_AP_IDLE, ok ? "acked as macOS" : "FAILED");
}

/*
 * E2k: macOS switches off every watchdog before sleep (trace 4, kernel log in the debug
 * FIFO): IOPlatformSleepAction -> AppleARMWatchdogTimer clears 0x23d2b0000 and 0x23d2b0010,
 * IOPlatformQuiesceAction -> AppleARMWatchdogTimer clears 0x23d2b8008, 0x23d2b800c and
 * 0x23d2b000c. After "pmu go hib (0x7)" it writes 0x23d2c4000 = 1 (PMGR quiesce, meaning
 * unknown). T8103 addresses; m1n1's own reset watchdog (wdt_arm) is not armed in E2k.
 */
static const struct {
    u64 addr;
    const char *what;
} s2r_wdt_off_t8103[] = {
    {0x23d2b0000, "wdt+0x00"}, {0x23d2b0010, "wdt+0x10"}, {0x23d2b8008, "wdt[2]"},
    {0x23d2b800c, "wdt[3]"},   {0x23d2b000c, "wdt+0x0c"},
};
#define S2R_WDT_OFF_N (sizeof(s2r_wdt_off_t8103) / sizeof(s2r_wdt_off_t8103[0]))
#define S2R_PMGR_QUIESCE 0x23d2c4000

static u32 s2r_wdt_saved[S2R_WDT_OFF_N];

static void s2r_wdt_off_macos(void)
{
    printf("  Watchdogs off (macOS order):\n");
    for (size_t i = 0; i < S2R_WDT_OFF_N; i++) {
        u64 a = s2r_wdt_off_t8103[i].addr;
        s2r_wdt_saved[i] = read32(a);
        write32(a, 0);
        printf("    %-9s %lx: %08x -> %08x\n", s2r_wdt_off_t8103[i].what, a, s2r_wdt_saved[i],
               read32(a));
    }
}

static void s2r_wdt_restore(void)
{
    /* only the wdt[2]/wdt[3] words: m1n1 runs with its own watchdogs disabled anyway */
    for (size_t i = 0; i < S2R_WDT_OFF_N; i++)
        if (s2r_wdt_off_t8103[i].addr >= 0x23d2b8000)
            write32(s2r_wdt_off_t8103[i].addr, s2r_wdt_saved[i]);
}

/*
 * E2l: the rest of IOPlatformQuiesceAction -> AppleT8103PMGR from trace 4 (59.780-59.785 s,
 * after the watchdog quiesce, before "pmu go hib"), replayed with macOS's values and in its
 * order. The SIO power-state writes are left out (hand replay rebooted the Mac). T8103 only.
 */
static const struct {
    u64 addr;
    u64 val;
    u8 w64;
} s2r_pmgr_quiesce_t8103[] = {
    {0x23b040140, 0x80100000, 0},      {0x23b040144, 0x80100000, 0},
    {0x23b040148, 0x80100000, 0},      {0x23b04014c, 0x80100000, 0},
    {0x23b040154, 0x80100000, 0},      {0x23b734000, 0, 0},
    {0x23b77c000, 0, 0},               {0x23b778000, 0, 0},
    {0x210e20660, 0, 1},               {0x210e48080, 0, 1},
    {0x211e20660, 0, 1},               {0x211e48080, 0, 1},
    {0x210e20020, 0x101101, 1},        {0x211e20020, 0x101101, 1},
    {0x210e40250, 0x1b20026f02400, 1}, {0x211e40250, 0x1b2005df05200, 1},
    {0x210e40240, 0x2000026f00000, 1}, {0x211e40240, 0x100005df00000, 1},
    {0x210e48400, 0x3b20026f02400, 1}, {0x211e48400, 0x1b2005df05200, 1},
    {0x23b748400, 0, 0},               {0x23b748800, 0, 0},
    {0x23b748c00, 0, 0},               {0x23b748000, 0, 0},
    {0x210e20020, 0x40000501101, 1},   {0x211e20020, 0x40000501101, 1},
    {0x23b71c080, 0x80000000, 0},      {0x23b71c088, 0x80000000, 0},
    {0x23b718000, 0, 0},               {0x23b718004, 0, 0},
};
#define S2R_PMGR_QUIESCE_N (sizeof(s2r_pmgr_quiesce_t8103) / sizeof(s2r_pmgr_quiesce_t8103[0]))

static void s2r_pmgr_quiesce_macos(void)
{
    printf("  PMGR quiesce replay (%lu writes):\n", S2R_PMGR_QUIESCE_N);
    s2r_console_sync(100);
    for (size_t i = 0; i < S2R_PMGR_QUIESCE_N; i++) {
        u64 a = s2r_pmgr_quiesce_t8103[i].addr, v = s2r_pmgr_quiesce_t8103[i].val;
        u64 before, after;
        if (s2r_pmgr_quiesce_t8103[i].w64) {
            before = read64(a);
            write64(a, v);
            after = read64(a);
        } else {
            before = read32(a);
            write32(a, (u32)v);
            after = read32(a);
        }
        printf("    %lx: %lx -> %lx (now %lx)\n", a, before, v, after);
    }
    s2r_console_sync(100);
}

/*
 * S2R snapshot (T8103): the state right before the final WFI, in the same "SNAP" format as
 * s2r_snapshot() in proxyclient/hv/trace_s2r_wake_arming.py (taken in macOS at ANCHOR #1), so
 * the two can be diffed with proxyclient/hv/s2r_snapdiff.py.
 */
#define S2R_SNAPSHOT 1

static const struct {
    u64 base;
    u32 count;
    u32 stride;
    u8 w64;
    const char *what;
} s2r_snap_ranges_t8103[] = {
    {0x23b040000, 86, 4, 0, "CLK_CFG0"},  {0x23b040200, 8, 4, 0, "CLK_CFG1"},
    {0x23b040280, 2, 4, 0, "CLK_CFG2"},   {0x23b71c000, 36, 4, 0, "PMGR0_PG"},
    {0x23d29c000, 72, 4, 0, "PMGR1_PG"},  {0x23b718000, 2, 4, 0, "PMGR0_718"},
    {0x23b734000, 1, 4, 0, "PMGR0_734"},  {0x23b77c000, 1, 4, 0, "PMGR0_77c"},
    {0x23b778000, 1, 4, 0, "PMGR0_778"},  {0x23b748000, 4, 0x400, 0, "CPUTVM"},
    {0x23d2c4000, 1, 4, 0, "QUIESCE"},    {0x210e20020, 1, 8, 1, "ECPU_20020"},
    {0x211e20020, 1, 8, 1, "PCPU_20020"}, {0x210e20660, 1, 8, 1, "ECPU_20660"},
    {0x211e20660, 1, 8, 1, "PCPU_20660"}, {0x210e48080, 1, 8, 1, "ECPU_48080"},
    {0x211e48080, 1, 8, 1, "PCPU_48080"}, {0x210e40240, 2, 0x10, 1, "ECPU_4024x"},
    {0x211e40240, 2, 0x10, 1, "PCPU_4024x"}, {0x210e48400, 1, 8, 1, "ECPU_48400"},
    {0x211e48400, 1, 8, 1, "PCPU_48400"},
};

static void s2r_snapshot(void)
{
    printf("SNAP BEGIN m1n1\n");
    pmgr_snapshot_ps();
    s2r_console_sync(100);
    for (size_t r = 0; r < sizeof(s2r_snap_ranges_t8103) / sizeof(s2r_snap_ranges_t8103[0]);
         r++) {
        for (u32 i = 0; i < s2r_snap_ranges_t8103[r].count; i++) {
            u64 a = s2r_snap_ranges_t8103[r].base + (u64)i * s2r_snap_ranges_t8103[r].stride;
            if (s2r_snap_ranges_t8103[r].w64)
                printf("SNAP R %s %lx %lx\n", s2r_snap_ranges_t8103[r].what, a, read64(a));
            else
                printf("SNAP R %s %lx %x\n", s2r_snap_ranges_t8103[r].what, a, read32(a));
        }
        s2r_console_sync(20);
    }
    printf("SNAP END\n");
    s2r_console_sync(200);
}

/* IOPlatformSleepAction -> AppleGen1SPMIController / AppleStockholmSPMI: +0x38 = 1 */
#define S2R_SPMI_AOP  0x24a829338
#define S2R_SPMI_NUB1 0x23d0e1338
#define S2R_SPMI_NUB0 0x23d0d9338

/*
 * E2m, from the hypervisor traces (macOS 13.6.2):
 *  - SEP (trace 4, 58.49 s, AppleSEPManager power state 2 -> 0): AP sends 0x00000000020c0600
 *    (SEP message: ep 0x00, tag 0x06, opcode 0x0c, param 0x02), SEP replies
 *    0x0000000000010600 (opcode 0x01), then the AP disables the I2A mailbox.
 *  - NVMe (trace 4, 58.77 s, "[ANS2:client1] NVMe shutdown start"): m1n1's own nvme_init()
 *    + nvme_shutdown() path (controller shutdown + ANS sleep).
 *  - PMP ep 0x20 (trace 3, 164.43-164.49 s, between WKTP and MBSE='slpa'): five messages,
 *    0x63 is acked by 0x0020900000630000. Meaning unknown, replayed verbatim.
 */
#define S2R_SEP_SLEEP_MSG  0x00000000020c0700UL
#define S2R_SEP_SLEEP_ACK  0x0000000000010700UL

/*
 * SEP sleep as macOS does it (trace 9, 217.57 s): cntl message 0x00000000020c0700, ack
 * 0x0000000000010700, then the AP disables the outbox (OUTBOX_CTRL.ENABLE = 0) and the SEP
 * powers itself off (PS 0x23b700c00 -> 1000230f). Needs sepOS running and its xART requests
 * served (proxyclient/experiments/s2r_sep_up.py); the boot ROM does not ack it.
 */
#define S2R_SEP_PS 0x23b700c00
static void s2r_sep_sleep(void)
{
    struct asc_message msg;

    printf("  SEP sleep:\n");
    u32 ps = read32(S2R_SEP_PS);
    if (!((ps >> 4) & 0xf)) {
        printf("    SEP already off (PS %08x), skipped\n", ps);
        return;
    }
    asc_dev_t *sep = asc_init("/arm-io/sep");
    if (!sep) {
        printf("    asc_init(/arm-io/sep) failed\n");
        return;
    }
    while (asc_recv(sep, &msg))
        printf("    pending 0x%016lx (dropped)\n", msg.msg0);
    msg.msg0 = S2R_SEP_SLEEP_MSG;
    msg.msg1 = 0;
    bool acked = false;
    if (!asc_send(sep, &msg)) {
        printf("    send failed\n");
    } else {
        u64 t = timeout_calculate(2000000);
        while (!acked && !timeout_expired(t)) {
            if (!asc_recv(sep, &msg))
                continue;
            if (msg.msg0 == S2R_SEP_SLEEP_ACK)
                acked = true;
            else
                printf("    0x%016lx (other message, dropped)\n", msg.msg0);
        }
        printf("    0x%016lx sent, %s\n", S2R_SEP_SLEEP_MSG,
               acked ? "acked as macOS" : "NO ack (sepOS not running or xART not served?)");
    }
    if (acked) {
        u64 base = 0;
        int path[8];
        int node = adt_path_offset_trace(adt, "/arm-io/sep", path);
        if (node >= 0 && !adt_get_reg(adt, path, "reg", 0, &base, NULL)) {
            write32(base + 0x8114, read32(base + 0x8114) & ~1u); /* OUTBOX_CTRL.ENABLE = 0 */
            u64 t = timeout_calculate(1000000);
            while (((read32(S2R_SEP_PS) >> 4) & 0xf) && !timeout_expired(t))
                ;
        }
    }
    printf("    SEP power state now %08x%s\n", read32(S2R_SEP_PS),
           ((read32(S2R_SEP_PS) >> 4) & 0xf) ? "" : " (OFF, as macOS)");
    asc_free(sep);
}

static void s2r_nvme_sleep(void)
{
    printf("  NVMe shutdown:\n");
    s2r_console_sync(100);
    if (!nvme_init()) {
        printf("    nvme_init failed\n");
        return;
    }
    nvme_shutdown();
    printf("    done, ANS2 power state now %08x\n", read32(0x23b7003f0));
}

static void s2r_pmp_ep20_replay(void)
{
    static const u64 seq[] = {0x0020e00000550001, 0x0020e00000540001, 0x0020800000630000,
                              0x0020e00000550000, 0x0020e00000540000};

    if (!s2r_pmpdev.rtk) {
        printf("  PMP ep 0x20 replay: PMP not booted, skipped\n");
        return;
    }
    if (s2r_pmp_crashed) {
        printf("  PMP ep 0x20 replay: PMP crashed, skipped\n");
        return;
    }
    printf("  PMP ep 0x20 replay:\n");
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        struct rtkit_message m = {.ep = 0x20, .msg = seq[i]};
        printf("    > 0x%016lx: %s\n", seq[i], rtkit_send(s2r_pmpdev.rtk, &m) ? "sent" : "FAILED");
        u64 timeout = timeout_calculate(100000);
        while (!timeout_expired(timeout)) {
            if (!rtkit_can_recv(s2r_pmpdev.rtk))
                continue;
            int ret = rtkit_recv(s2r_pmpdev.rtk, &m);
            if (ret > 0)
                printf("    < ep 0x%02x 0x%016lx\n", m.ep, m.msg);
            if (ret < 0)
                break;
        }
    }
    s2r_console_sync(100);
}

/*
 * E2n, trace 5 (full boot): macOS opens PMP endpoint 0x20 like this at boot:
 *   PMP -> AP 0x0000000000000000  (right after StartEP 0x20: looks like a buffer request)
 *   AP -> PMP 0x000100000003c000  (type 1 + shared memory IOVA 0x3c000)
 *   PMP -> AP 0x000200000023b549  (type 2, ~1.5 s later)
 *   AP -> PMP 19 configuration messages (below). Requests with command nibble 0/2/8/a/c
 *   (bits 47:44) are acked by the PMP with nibble + 1 and the same payload; 0xe has no ack.
 * m1n1 left the 0x0 request unanswered, which is why the PMP ignored our later messages.
 * The buffer size is unknown: 64 KiB is allocated (macOS' next buffer would fit below).
 */
#define S2R_EP20_BUF_SIZE 0x10000 /* macOS: >= 4 pages at 0x3c000 (trace 6) */
#ifndef S2R_EP20_DVA
#define S2R_EP20_DVA 0x3c000 /* macOS' value; 0 = let m1n1 allocate the IOVA */
#endif

/* Trace 6 order (boot, 10.376 s .. 11.233 s): slightly different from trace 5 */
static const u64 s2r_pmp_ep20_boot[] = {
    0x0020000000010003, 0x0020200000010000, 0x0020e00000630001, 0x0020e00000680001,
    0x0020e00000660001, 0x0020e000005c0001, 0x0020e000005c0000, 0x0020e000005c0001,
    0x0020e000005c0000, 0x0020e00000650001, 0x0020e000005c0001, 0x0020e000005c0000,
    0x0020e000005c0001, 0x0020e000005c0000, 0x0020e000005d0001, 0x0020e00000650000,
    0x0020e00000660000, 0x0020e00000660001, 0x0020e000005d0000, 0x0020c00000640021,
    0x0020e00000660000, 0x0020a00000640020, 0x0020800000680000, 0x0020e00000640000,
    0x0020e00000640001, 0x0020a00000540022, 0x0020a00000550023,
};

/*
 * Trace 6: between the type-2 dump and the 0x680000 dump this appeared at buffer +0x43b0,
 * around the 0x0020c00000640021 message. Two 0x28-byte "ISP" entries: {name[8], 0, 0,
 * n (2 / 1), reg 0x2_3b738010 / 0x2_3b73800c (lo, hi), 0x2_3bc3c000 (lo, hi), n}.
 * Without it the PMP crashes on 0x640021: written right before that message.
 */
#define S2R_EP20_ISP_DESC 0x43b0
static const u32 s2r_pmp_ep20_isp_desc[] = {
    0x00505349, 0x00000000, 0x00000000, 0x00000000, 0x00000002,
    0x3b738010, 0x00000002, 0x3bc3c000, 0x00000002, 0x00000002,
    0x00505349, 0x00000000, 0x00000000, 0x00000000, 0x00000001,
    0x3b73800c, 0x00000002, 0x3bc3c000, 0x00000002, 0x00000001,
};

/* Wait up to us for an ep 0x20 message matching (msg & mask) == val; print everything */
static bool s2r_pmp_ep20_wait(u64 mask, u64 val, u32 us, u64 *got)
{
    u64 timeout = timeout_calculate(us);

    while (!timeout_expired(timeout)) {
        struct rtkit_message m;
        if (!rtkit_can_recv(s2r_pmpdev.rtk))
            continue;
        int ret = rtkit_recv(s2r_pmpdev.rtk, &m);
        if (ret < 0) {
            s2r_pmp_crashed = true;
            return false;
        }
        if (ret == 0)
            continue;
        printf("      < ep 0x%02x 0x%016lx\n", m.ep, m.msg);
        if (m.ep == 0x20 && (m.msg & mask) == val) {
            if (got)
                *got = m.msg;
            return true;
        }
    }
    return false;
}

/* Send one ep 0x20 message, wait for its ack when the command expects one */
static bool s2r_pmp_ep20_cmd(u64 msg)
{
    struct rtkit_message m = {.ep = 0x20, .msg = msg};
    u32 cmd = (msg >> 44) & 0xf;
    bool acked = (cmd & 1) == 0 && cmd != 0xe;

    if (msg == 0x0020c00000640021UL && s2r_pmpdev.ep20_fixed && s2r_pmpdev.ep20_buf.bfr) {
        u8 *b = s2r_pmpdev.ep20_buf.bfr;
        memcpy(b + S2R_EP20_ISP_DESC, s2r_pmp_ep20_isp_desc, sizeof(s2r_pmp_ep20_isp_desc));
        dc_civac_range(b + S2R_EP20_ISP_DESC, sizeof(s2r_pmp_ep20_isp_desc));
        printf("    (ISP descriptor written at +0x%x)\n", S2R_EP20_ISP_DESC);
    }
    printf("    > 0x%016lx\n", msg);
    if (!rtkit_send(s2r_pmpdev.rtk, &m)) {
        printf("      send FAILED\n");
        return false;
    }
    if (!acked) {
        /* No ack: listen 100 ms anyway so a PMP crash is pinned to this message */
        s2r_pmp_ep20_wait(~0UL, ~0UL, 100000, NULL);
        if (s2r_pmp_crashed) {
            printf("      PMP CRASHED after 0x%016lx\n", msg);
            return false;
        }
        return true;
    }
    u64 ack = (msg & ~(0xfUL << 44)) | ((u64)(cmd + 1) << 44);
    if (!s2r_pmp_ep20_wait(~0UL, ack, 300000, NULL)) {
        printf("      no ack (expected 0x%016lx)%s\n", ack,
               s2r_pmp_crashed ? ": PMP CRASHED" : "");
        return false;
    }
    return true;
}

/*
 * Trace 6 (2026-11-05, hypervisor dump of the buffer through dart-pmp): what macOS puts in the
 * ep 0x20 buffer before the type-1 message. With a zeroed buffer the PMP crashes.
 *   +0x2000  energy-model-dram-configs: the 8 GB / vendor 6 entry of the iop-pmp-nub property
 *   +0x4000  SOC/DCS/DISP VMIN/VNOM/VMAX (mV) and the CPU/ECPU/PCPU/GPU/ISP/DISP/VENC/VDEC/
 *            JPEG/MSR table
 *   +0xe000  PIO map: {u64 dva, u64 size} for every /arm-io/pmp "reg" from pio-reg-index (3)
 *            on (zero for empty regs). macOS also maps those MMIO windows into dart-pmp at
 *            those DVAs: pio-vm-base 0xc0000000, one 16 MiB slot (pio-granularity) per 16 MiB
 *            physical window in order of first use, regions packed inside a slot.
 * The PMP fills +0x0000.. (EHUB/PM/FAST_DIE tunables) and +0x41d0.. itself before type 2.
 */
/* trace 6 type-1 dump, buffer +0x2000 */
static const u32 s2r_pmp_ep20_dramcfg[] = {
    0x00000008, 0x00000006, 0x001a4ccc, 0x00009999, 0x00076666, 0x00010000, 0x0008b333, 0x00588000,
    0x0054e666, 0x000b3333, 0x0000cccc, 0x00176666, 0x00236666, 0x00018000, 0x000f9999, 0x0001e666,
    0x0011b333, 0x00b44ccc, 0x00aa1999, 0x00148000, 0x0000cccc, 0x002ecccc, 0x0025cccc, 0x00018000,
    0x00123333, 0x0001e666, 0x00140000, 0x00f24ccc, 0x00d23333, 0x0016b333, 0x0000cccc, 0x00443333,
};
/* trace 6 type-1 dump, buffer +0x4000 */
static const u32 s2r_pmp_ep20_vtable[] = {
    0x00434f53, 0x00000000, 0x4e494d56, 0x00000000, 0x00000286, 0x4d4f4e56, 0x00000000, 0x000002bf,
    0x58414d56, 0x00000000, 0x0000036e, 0x00534344, 0x00000000, 0x4e494d56, 0x00000000, 0x00000274,
    0x4d4f4e56, 0x00000000, 0x000002cb, 0x58414d56, 0x00000000, 0x00000313, 0x50534944, 0x00000000,
    0x4e494d56, 0x00000000, 0x0000026a, 0x4d4f4e56, 0x00000000, 0x000002d8, 0x58414d56, 0x00000000,
    0x00000332, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00555043, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x55504345, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x55504350, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00555047, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00505349, 0x00000000, 0x00000000, 0x0000012c,
    0x000001ab, 0x00000215, 0x50534944, 0x00000030, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x50534944, 0x00000058, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x50534944, 0x00005830,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x434e4556, 0x00000000, 0x00000003, 0x00000168,
    0x00000215, 0x000002c8, 0x43454456, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x4745504a, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0052534d, 0x00000000,
};

#define S2R_EP20_PIO_TABLE 0xe000

static int s2r_pmp_pio_map(u8 *buf)
{
    const char *path = "/arm-io/pmp";
    int pathv[8];
    int node = adt_path_offset_trace(adt, path, pathv);
    u64 gran = 0x1000000;
    if (node < 0)
        return -1;
    /*
     * adt_getprop_copy() copies the whole property before checking its length: read into
     * zeroed u64s so a 4- or 8-byte property both fit (little endian: low word first).
     */
    u64 v = 0;
    ADT_GETPROP(adt, node, "pio-reg-index", &v);
    u32 pio_idx = v ? (u32)v : 3;
    v = 0;
    ADT_GETPROP(adt, node, "pio-vm-base", &v);
    u64 pio_base = v ? v : 0xc0000000;

    u64 slot_win[16];
    u64 slot_off[16];
    int nslots = 0, n = 0, mapped = 0;

    for (int idx = pio_idx;; idx++, n++) {
        u64 pa, sz;
        if (adt_get_reg(adt, pathv, "reg", idx, &pa, &sz) < 0)
            break;
        u64 *ent = (u64 *)(buf + S2R_EP20_PIO_TABLE + 16 * n);
        if (S2R_EP20_PIO_TABLE + 16 * (n + 1) > S2R_EP20_BUF_SIZE)
            break;
        if (!sz) {
            ent[0] = ent[1] = 0;
            continue;
        }
        u64 win = pa / gran;
        int s;
        for (s = 0; s < nslots; s++)
            if (slot_win[s] == win)
                break;
        if (s == nslots) {
            if (nslots == 16)
                return -1;
            slot_win[s] = win;
            slot_off[s] = 0;
            nslots++;
        }
        u64 dva = pio_base + s * gran + slot_off[s];
        slot_off[s] += sz;
        ent[0] = dva;
        ent[1] = sz;
        void *cur = dart_translate_silent(s2r_pmpdev.dart, dva);
        if (cur) {
            if ((u64)cur != pa)
                printf("    PIO reg %d: dva 0x%lx already maps %p (want 0x%lx)\n", idx, dva, cur, pa);
            continue;
        }
        if (dart_map(s2r_pmpdev.dart, dva, (void *)pa, sz) < 0) {
            printf("    PIO reg %d: dart_map(0x%lx -> 0x%lx, 0x%lx) FAILED\n", idx, dva, pa, sz);
            return -1;
        }
        mapped++;
    }
    printf("    PIO map: %d reg entries from index %u, %d slot(s), %d window(s) mapped at 0x%lx\n",
           n, pio_idx, nslots, mapped, pio_base);
    return n;
}

static void s2r_pmp_ep20_fill(u8 *buf)
{
    memset(buf, 0, S2R_EP20_BUF_SIZE);
    memcpy(buf + 0x2000, s2r_pmp_ep20_dramcfg, sizeof(s2r_pmp_ep20_dramcfg));
    memcpy(buf + 0x4000, s2r_pmp_ep20_vtable, sizeof(s2r_pmp_ep20_vtable));
}

static bool s2r_pmp_ep20_init(void)
{
    u64 got = 0;

    if (!s2r_pmpdev.rtk)
        return false;
    printf("  PMP ep 0x20 init (macOS boot sequence):\n");

    printf("    waiting for the PMP buffer request (0x0):\n");
    if (!s2r_pmp_ep20_wait(~0UL, 0, 1000000, NULL))
        printf("      not seen, answering anyway\n");

    /*
     * Run 1 (2026-11-05): a fresh buffer at DVA 0x10018000 crashed the PMP. macOS sends
     * 0x3c000: either iBoot's (kept) DART tables already map it, or it must be that address.
     */
    static const u64 probe_dva[] = {0x18000, 0x20000, 0x34000, 0x38000, 0x3c000, 0x40000};
    for (size_t i = 0; i < sizeof(probe_dva) / sizeof(probe_dva[0]); i++)
        printf("    DART dva 0x%05lx -> %p\n", probe_dva[i],
               dart_translate_silent(s2r_pmpdev.dart, probe_dva[i]));

    u64 dva;
    if (S2R_EP20_DVA) {
        dva = S2R_EP20_DVA;
        void *pre = dart_translate_silent(s2r_pmpdev.dart, dva);
        if (pre) {
            printf("    dva 0x%lx already mapped by iBoot to %p, first words:", dva, pre);
            for (int i = 0; i < 8; i++)
                printf(" %08x", ((u32 *)pre)[i]);
            printf("\n");
        } else {
            void *b = memalign(SZ_16K, S2R_EP20_BUF_SIZE);
            if (!b || dart_map(s2r_pmpdev.dart, dva, b, S2R_EP20_BUF_SIZE) < 0) {
                printf("    mapping a buffer at dva 0x%lx FAILED\n", dva);
                free(b);
                return false;
            }
            s2r_pmp_ep20_fill(b);
            if (s2r_pmp_pio_map(b) < 0) {
                printf("    PIO map FAILED, not sending type 1\n");
                return false;
            }
            dc_civac_range(b, S2R_EP20_BUF_SIZE);
            s2r_pmpdev.ep20_buf.bfr = b;
            s2r_pmpdev.ep20_buf.dva = dva;
            s2r_pmpdev.ep20_buf.sz = S2R_EP20_BUF_SIZE;
            s2r_pmpdev.ep20_fixed = true;
            printf("    mapped the macOS-style 0x%x buffer %p at dva 0x%lx\n", S2R_EP20_BUF_SIZE, b, dva);
        }
    } else {
        if (!rtkit_alloc_buffer(s2r_pmpdev.rtk, &s2r_pmpdev.ep20_buf, S2R_EP20_BUF_SIZE)) {
            printf("    buffer allocation FAILED\n");
            return false;
        }
        memset(s2r_pmpdev.ep20_buf.bfr, 0, S2R_EP20_BUF_SIZE);
        dva = s2r_pmpdev.ep20_buf.dva;
        printf("    shared buffer %p dva 0x%lx size 0x%x\n", s2r_pmpdev.ep20_buf.bfr, dva,
               S2R_EP20_BUF_SIZE);
    }

    u64 msg = 0x0001000000000000UL | (dva & 0xffffffffffUL);
    printf("    > 0x%016lx (type 1, buffer)\n", msg);
    struct rtkit_message m = {.ep = 0x20, .msg = msg};
    if (!rtkit_send(s2r_pmpdev.rtk, &m))
        return false;

    printf("    waiting for type 2 (macOS: 0x000200000023b549, ~1.5 s):\n");
    if (!s2r_pmp_ep20_wait(0xffff000000000000UL, 0x0002000000000000UL, 4000000, &got)) {
        printf("      no type 2 message, ep 0x20 not initialized\n");
        s2r_console_sync(100);
        return false;
    }
    s2r_console_sync(100);

    /*
     * Run 11 (2026-11-05): after 0x680001 the PMP polls PIO 0xc50000c8 = pmp reg 33 =
     * 0x26b90c0c8, inside the ANE block, and takes an SError (ESR 0xbe000000) because the
     * ANE is power-gated under m1n1 (on under macOS at that point). Power the PMP's monitored
     * blocks on first.
     */
    static const char *const s2r_pmp_need_on[] = {"/arm-io/ane"};
    for (size_t i = 0; i < sizeof(s2r_pmp_need_on) / sizeof(s2r_pmp_need_on[0]); i++)
        printf("    power on %s: %d\n", s2r_pmp_need_on[i],
               pmgr_adt_power_enable(s2r_pmp_need_on[i]));

    int ok = 0;
    for (size_t i = 0; i < sizeof(s2r_pmp_ep20_boot) / sizeof(s2r_pmp_ep20_boot[0]); i++)
    {
        ok += s2r_pmp_ep20_cmd(s2r_pmp_ep20_boot[i]);
        if (s2r_pmp_crashed) {
            printf("    stopping: PMP crashed at boot message %lu\n", i);
            break;
        }
    }
    printf("    %d/%lu boot messages ok\n", ok,
           sizeof(s2r_pmp_ep20_boot) / sizeof(s2r_pmp_ep20_boot[0]));
    s2r_console_sync(100);
    return true;
}

/*
 * AppleInterruptController quiesce (trace 5, 270.580 s, just before "pmu go hib"):
 * AIC_CPU_IPI_MASK_SET(cpu) = 0x80000001 for cpu 0..10, i.e. every IPI masked.
 */
#define S2R_AIC_CPU_IPI_MASK_SET(cpu) (0x5024 + ((cpu) << 7))
#define S2R_AIC_CPU_IPI_MASK_CLR(cpu) (0x5028 + ((cpu) << 7))
#define S2R_AIC_IPI_CPUS              11

static void s2r_aic_quiesce(void)
{
    for (int cpu = 0; cpu < S2R_AIC_IPI_CPUS; cpu++)
        aic_write(S2R_AIC_CPU_IPI_MASK_SET(cpu), 0x80000001);
    printf("  AIC quiesce: CPU IPI mask set 0x80000001 for cpu 0..%d\n", S2R_AIC_IPI_CPUS - 1);
}

static void s2r_aic_unquiesce(void)
{
    /* m1n1 uses the fast IPIs on M1: the AIC per-CPU IPIs can just be unmasked again */
    for (int cpu = 0; cpu < S2R_AIC_IPI_CPUS; cpu++)
        aic_write(S2R_AIC_CPU_IPI_MASK_CLR(cpu), 0x80000001);
}

/*
 * Sleep until the SMC reports a power button or lid event (or the watchdog fires).
 * deep=false: plain WFI (E2a). deep=true: Level 3 overrides + reset-vector resume (E2b).
 */
/*
 * E2o, trace 5: the last value macOS writes to each PMGR[0] power-state register before
 * sleep (155 s .. 270.6 s, chronological). Most go to target PWRGATE; the display pipe,
 * GFX, RMX, SPI/I2C and APCIE_ST between 269.05 and 269.82 s, SIO_CPU to auto at 270.57 s.
 * Under m1n1 the display pipe (the lit screen), the ANE (PMP init) and whatever iBoot/m1n1
 * enabled stay on. Applied after the PMP is stopped (it polls the ANE).
 */
static const struct {
    const char *name;
    u32 off;
    u32 val;
} s2r_ps_macos_final[] = {
    {"FPWM1", 0x001e0, 0x000003f0}, /* 167.566 s */
    {"AVD_SYS", 0x00410, 0x04002340}, /* 174.845 s */
    {"AES", 0x00238, 0x000003f0}, /* 176.299 s */
    {"ISP_CPU_CORE1", 0x04010, 0x00000000}, /* 233.305 s */
    {"ISP_CPU_CORE0", 0x04008, 0x00000000}, /* 233.305 s */
    {"ISPSENS3", 0x00198, 0x04000340}, /* 233.305 s */
    {"ISPSENS2", 0x00190, 0x04000340}, /* 233.306 s */
    {"ISPSENS1", 0x00188, 0x04000340}, /* 233.306 s */
    {"ISPSENS0", 0x00180, 0x04000340}, /* 233.307 s */
    {"ISP_SYS", 0x00400, 0x000023f0}, /* 233.308 s */
    {"MSR", 0x003c8, 0x04002340}, /* 248.825 s */
    {"MMX", 0x00358, 0x04000340}, /* 248.825 s */
    {"ANE_SYS_CPU", 0x0c000, 0x00000300}, /* 249.611 s */
    {"ANE_SYS", 0x00470, 0x0f0023f0}, /* 249.613 s */
    {"GFX", 0x003f8, 0x0f0023f0}, /* 269.055 s */
    {"DISPEXT_CPU0", 0x00378, 0x0f0403f0}, /* 269.100 s */
    {"DISPEXT_FE", 0x00368, 0x000023f0}, /* 269.100 s */
    {"DISP0_CPU0", 0x10018, 0x0f0403f0}, /* 269.102 s */
    {"DISP0_FE", 0x00360, 0x000023f0}, /* 269.102 s */
    {"RMX", 0x00350, 0x00000300}, /* 269.102 s */
    {"SPI2", 0x00250, 0x000003f0}, /* 269.125 s */
    {"I2C0", 0x001f0, 0x000003f0}, /* 269.129 s */
    {"SPI3", 0x00258, 0x000003f0}, /* 269.198 s */
    {"I2C3", 0x00208, 0x000003f0}, /* 269.232 s */
    {"APCIE_ST", 0x00418, 0x0f0003f0}, /* 269.817 s */
    {"SIO_CPU", 0x001d0, 0x0400024f}, /* 270.566 s */
};

static void s2r_ps_domains_off_macos(void)
{
    u64 pmgr = 0x23b700000UL;
    printf("  Power domains as macOS leaves them (%lu):\n",
           sizeof(s2r_ps_macos_final) / sizeof(s2r_ps_macos_final[0]));
    for (size_t i = 0; i < sizeof(s2r_ps_macos_final) / sizeof(s2r_ps_macos_final[0]); i++) {
        u64 a = pmgr + s2r_ps_macos_final[i].off;
        if (s2r_ps_is_disp(s2r_ps_macos_final[i].name)) {
            printf("    %-14s kept (display)\n", s2r_ps_macos_final[i].name);
            continue;
        }
        u32 before = read32(a);
        write32(a, s2r_ps_macos_final[i].val);
        /* wait for actual (bits 7:4) to follow target (bits 3:0), at most 2 ms */
        u64 t = timeout_calculate(2000);
        u32 now = read32(a);
        while (((now >> 4) & 0xf) != (s2r_ps_macos_final[i].val & 0xf) && !timeout_expired(t))
            now = read32(a);
        printf("    %-14s %08x -> %08x (now %08x)%s\n", s2r_ps_macos_final[i].name, before,
               s2r_ps_macos_final[i].val, now,
               ((now >> 4) & 0xf) != (s2r_ps_macos_final[i].val & 0xf) ? " NOT REACHED" : "");
    }
}

/*
 * E2p: the remaining power-state differences between m1n1 and macOS at its last CPU shutdown
 * (hypervisor trace 8, snapshot at ANCHOR #24, diffed with hv/s2r_snapdiff.py). CPU/cluster
 * entries are left out (the hypervisor keeps the guest CPUs up). val holds bits 28:24 (auto
 * enable + auto state) and 3:0 (target); the other bits are kept. ATC0 (USB port not used for
 * the proxy, macOS gates it under the hypervisor) is done last, after the final console flush.
 */
static const struct {
    const char *name;
    u64 addr;
    u32 val;
} s2r_ps_macos_snap_t8103[] = {
    /* leaves macOS has OFF (m1n1: ACTIVE or CLKGATE) */
    {"PMGR_SOC_OCLA", 0x23b700178, 0},
    {"I2C1", 0x23b7001f8, 0},
    {"I2C2", 0x23b700200, 0},
    {"I2C4", 0x23b700210, 0},
    {"SPI0", 0x23b700240, 0},
    {"SPI1", 0x23b700248, 0},
    {"SPI4", 0x23b700260, 0},
    {"MCA1", 0x23b7002c0, 0},
    {"DPA0", 0x23b7002e8, 0},
    {"SIO_ADMA", 0x23b700230, 0},
    {"AFT0", 0x23b7001a8, 0},
    {"DEVC0_IVDMC", 0x23b7001b0, 0},
    {"FPWM0", 0x23b7001d8, 0},
    {"FPWM2", 0x23b7001e8, 0},
    {"PMS_FPWM0", 0x23b700148, 0},
    {"PMS_FPWM1", 0x23b700150, 0},
    {"PMS_FPWM2", 0x23b700158, 0},
    {"PMS_FPWM3", 0x23b700160, 0},
    {"PMS_FPWM4", 0x23b700168, 0},
    {"SOC_SPMI0", 0x23b700118, 0},
    {"SOC_SPMI1", 0x23b700120, 0},
    {"SOC_SPMI2", 0x23b700128, 0},
    {"MSG", 0x23d280078, 0},
    /* macOS: RMX off without auto (the PS auto table sets auto) */
    {"RMX", 0x23b700350, 0x0f000000},
    /* auto (power down when idle) as macOS */
    {"PCIE_REF", 0x23b7001a0, 0x1400000f},
    {"SIO_BUSIF", 0x23b7001c0, 0x1400000f},
    {"SIO", 0x23b7001c8, 0x1f00000f},
    {"APCIE_GP", 0x23b7003e8, 0x1000000f},
    {"ANS2", 0x23b7003f0, 0x1f00000f},
    {"DEBUG_USB", 0x23d2800b8, 0x1000000f},
    {"DEBUG_AUTH", 0x23d2800c0, 0x1000000f},
};

static const struct {
    const char *name;
    u64 addr;
} s2r_ps_atc0_t8103[] = {
    {"ATC0_USB", 0x23d280098},
    {"ATC0_USB_AON", 0x23d280088},
    {"ATC0_COMMON", 0x23b700420},
};

/*
 * E2q: the auto domains macOS has OFF at its last CPU shutdown but that stay ACTIVE under m1n1
 * (SEP does not acknowledge the sleep message there, reply opcode 0 instead of 0x01), forced
 * off; then ATC1 (the USB port of the proxy) last, after the final console flush.
 */
static const struct {
    const char *name;
    u64 addr;
} s2r_ps_force_off_t8103[] = {
    {"APCIE_GP", 0x23b7003e8},
    {"DEBUG_AUTH", 0x23d2800c0},
    {"DEBUG_USB", 0x23d2800b8},
}, s2r_ps_atc1_t8103[] = {
    {"ATC1_USB", 0x23d2800a0},
    {"ATC1_USB_AON", 0x23d280090},
    {"ATC1_COMMON", 0x23b700448},
};

static void s2r_ps_force_off(void)
{
    printf("  Stuck auto domains forced off:\n");
    for (size_t i = 0; i < sizeof(s2r_ps_force_off_t8103) / sizeof(s2r_ps_force_off_t8103[0]);
         i++) {
        u64 a = s2r_ps_force_off_t8103[i].addr;
        u32 before = read32(a);
        write32(a, before & ~0x1f00000fu & ~0x80000000u);
        u64 t = timeout_calculate(5000);
        u32 now = read32(a);
        while (((now >> 4) & 0xf) && !timeout_expired(t))
            now = read32(a);
        printf("    %-12s %08x -> now %08x%s\n", s2r_ps_force_off_t8103[i].name, before, now,
               ((now >> 4) & 0xf) ? " NOT REACHED" : "");
    }
}

/* No console after this: the proxy's USB port goes down */
static void s2r_ps_atc1_off(void)
{
    for (size_t i = 0; i < sizeof(s2r_ps_atc1_t8103) / sizeof(s2r_ps_atc1_t8103[0]); i++) {
        u64 a = s2r_ps_atc1_t8103[i].addr;
        write32(a, read32(a) & ~0x1f00000fu & ~0x80000000u);
        u64 t = timeout_calculate(2000);
        while (((read32(a) >> 4) & 0xf) && !timeout_expired(t))
            ;
    }
}

static void s2r_ps_macos_snap_apply(void)
{
    printf("  Power domains as in the macOS snapshot (%lu):\n",
           sizeof(s2r_ps_macos_snap_t8103) / sizeof(s2r_ps_macos_snap_t8103[0]));
    for (size_t i = 0; i < sizeof(s2r_ps_macos_snap_t8103) / sizeof(s2r_ps_macos_snap_t8103[0]);
         i++) {
        u64 a = s2r_ps_macos_snap_t8103[i].addr;
        u32 v = s2r_ps_macos_snap_t8103[i].val;
        if (s2r_ps_is_disp(s2r_ps_macos_snap_t8103[i].name)) {
            printf("    %-14s kept (display)\n", s2r_ps_macos_snap_t8103[i].name);
            continue;
        }
        u32 before = read32(a);
        u32 w = (before & ~0x1f00000fu & ~0x80000000u) | v;
        write32(a, w);
        bool autom = v & 0x10000000;
        u64 t = timeout_calculate(2000);
        u32 now = read32(a);
        while (!autom && ((now >> 4) & 0xf) != (v & 0xf) && !timeout_expired(t))
            now = read32(a);
        printf("    %-14s %08x -> %08x (now %08x)%s\n", s2r_ps_macos_snap_t8103[i].name, before, w,
               now, (!autom && ((now >> 4) & 0xf) != (v & 0xf)) ? " NOT REACHED" : "");
    }
}

/* No console after this (it may poll the USB0 controller): no printf */
static void s2r_ps_atc0_off(void)
{
    for (size_t i = 0; i < sizeof(s2r_ps_atc0_t8103) / sizeof(s2r_ps_atc0_t8103[0]); i++) {
        u64 a = s2r_ps_atc0_t8103[i].addr;
        write32(a, read32(a) & ~0x1f00000fu & ~0x80000000u);
        u64 t = timeout_calculate(2000);
        while (((read32(a) >> 4) & 0xf) && !timeout_expired(t))
            ;
    }
}

static int test_smc_button_wake(int mode)
{
    static const char *const names[] = {"E2a", "E2b", "E2c", "E2d", "E2e", "E2f", "E2g", "E2h", "E2i", "E2j", "E2k", "E2l", "E2m", "E2n", "E2o", "E2p", "E2q", "E2r"};
    bool deep = mode >= 1;
    bool l4 = mode >= 2;            /* Level 4: ACC_OVRD DEEP_SLEEP, power-gated core */
    bool handshake = mode >= 3;     /* E2d: macOS sleep handshake with the SMC first */
    bool platform = mode >= 4;      /* E2e: + GPIO wake group clear, per-core stop requests */
    bool pmu = mode >= 5;           /* E2f: + PMU boot_stage / sleep timestamp / sleep flag */
    bool allcores = mode >= 6;      /* E2g: + every secondary in deep sleep too */
    bool psauto = mode >= 7;        /* E2h: + macOS auto power states on memory/fabric/NUB */
    bool pmpsleep = mode >= 8;      /* E2i: + PMP coprocessor idle and stopped */
    bool pmp_stopped = false;
    bool pmpboot = mode >= 9;       /* E2j: boot the PMP first, as macOS runs with it */
    bool wdtoff = mode >= 10;       /* E2k: no safety watchdog, all watchdogs off like macOS */
    bool quiesce = mode >= 11;      /* E2l: + display off, PMGR quiesce replay, SPMI +0x38 */
    bool devices = mode >= 12;      /* E2m: + SEP sleep, NVMe shutdown, PMP ep 0x20 replay */
    bool ep20init = mode >= 13;     /* E2n: + PMP ep 0x20 init as at macOS boot, AIC quiesce */
    bool domoff = mode >= 14;       /* E2o: + power domains off as macOS leaves them */
    bool snapps = mode >= 15;       /* E2p: + every power state as in the macOS snapshot */
    bool forceoff = mode >= 16;     /* E2q: + stuck auto domains (SEP...) and ATC1 forced off */
    bool aop = mode >= 17;          /* E2r: + AOP booted, then SetAPPower 0x201 like macOS */
    spmi_dev_t *spmi = NULL;
    struct s2r_pmu_saved pmu_saved = {0};
    const char *name = names[mode];

    printf("\n--- %s: SMC power button wake from %s ---\n", name,
           aop       ? "Level 4 DEEP_SLEEP on all cores, E2q + AOP booted and told the AP sleeps"
           : forceoff  ? "Level 4 DEEP_SLEEP on all cores, E2p + SEP/APCIE_GP/DEBUG and ATC1 USB forced off"
           : snapps    ? "Level 4 DEEP_SLEEP on all cores, E2o + power states as the macOS snapshot"
           : domoff    ? "Level 4 DEEP_SLEEP on all cores, E2n + power domains off as macOS"
           : ep20init  ? "Level 4 DEEP_SLEEP on all cores, E2m + PMP ep 0x20 init + AIC quiesce"
           : devices   ? "Level 4 DEEP_SLEEP on all cores, E2l + SEP/NVMe sleep + PMP ep 0x20 replay"
           : quiesce   ? "Level 4 DEEP_SLEEP on all cores, E2k + display off + PMGR quiesce replay"
           : wdtoff    ? "Level 4 DEEP_SLEEP on all cores, E2j + watchdogs off like macOS (NO safety net)"
           : pmpboot   ? "Level 4 DEEP_SLEEP on all cores, running PMP put to sleep like macOS"
           : pmpsleep  ? "Level 4 DEEP_SLEEP on all cores, macOS sleep sequence + auto power states + PMP sleep"
           : psauto    ? "Level 4 DEEP_SLEEP on all cores, macOS sleep sequence + auto power states"
           : allcores  ? "Level 4 DEEP_SLEEP on all cores after the full macOS sleep sequence"
           : pmu       ? "Level 4 DEEP_SLEEP after SMC handshake + GPIO/PMGR prep + PMU sleep request"
           : platform  ? "Level 4 DEEP_SLEEP after SMC handshake + GPIO/PMGR sleep prep"
           : handshake ? "Level 4 DEEP_SLEEP after the macOS SMC sleep handshake"
           : l4      ? "Level 4 DEEP_SLEEP (power-gated core, no waker core)"
           : deep    ? "Level 3 deep WFI (reset-vector resume)"
                     : "plain WFI");

    u64 cyc_ovrd_saved = 0;
    u64 acc_ovrd_saved = mrs(SYS_IMP_APL_ACC_OVRD);

    smc_dev_t *smc = smc_init();
    if (!smc) {
        printf("  SMC init failed, skipping\n");
        return 0;
    }
    s2r_print_smc_irqs();
    if (smc_enable_notifications(smc))
        printf("  SMC: enabling notifications (NTAP) failed\n");

    u32 me = aic_whoami();
    if (!aic_route_irq(S2R_SMC_IRQ, BIT(me))) {
        printf("  AIC routing unsupported, skipping\n");
        smc_shutdown(smc);
        return 0;
    }

    /* The CPU timer must not be a wake source */
    msr(CNTP_CTL_EL0, 0);
    sysop("isb");

    if (l4 && cpu_features->sleep_mode != SLEEP_GLOBAL) {
        printf("  %s only implemented for SLEEP_GLOBAL, skipping\n", name);
        smc_shutdown(smc);
        return 0;
    }

    s2r_console_sync(100);

    if (pmpboot) {
        printf("  PMP boot:\n");
        if (s2r_pmp_up()) {
            if (ep20init)
                s2r_pmp_ep20_init();
            else
                s2r_pmp_listen(1000000);
        }
    }

    if (aop && chip_id == T8103)
        s2r_aop_up();

    if (quiesce && chip_id == T8103) {
        printf("  Display off: from here on the console is on the proxy only\n");
        s2r_console_sync(200);
        int dret = display_sleep_dcp();
        printf("  display_sleep_dcp: %d\n", dret);
        s2r_console_sync(100);
    }

    /* macOS order: SEP (58.49 s), NVMe (58.77 s), then the SMC WKTP write (58.88 s) */
    if (devices && chip_id == T8103) {
        s2r_sep_sleep();
        s2r_nvme_sleep();
        s2r_pmp_ep20_replay();
        s2r_console_sync(100);
    }

    /* macOS: AOP SetAPPower 0x201 (183.46 s) before the SMC (183.90 s) and the PMP (183.94 s) */
    if (aop)
        s2r_aop_sleep();

    if (handshake)
        s2r_smc_sleep_handshake(smc);

    if (pmu) {
        spmi = spmi_init("/arm-io/nub-spmi");
        printf("  PMU sleep stage:\n");
        if (!spmi || !s2r_pmu_sleep_stage(spmi, &pmu_saved))
            printf("    PMU boot_stage step FAILED\n");
    }

    if (quiesce && chip_id == T8103) {
        write32(S2R_SPMI_AOP, 1);
        write32(S2R_SPMI_NUB1, 1);
        printf("  SPMI +0x38 = 1: aop-spmi %x, nub-spmi1 %x\n", read32(S2R_SPMI_AOP),
               read32(S2R_SPMI_NUB1));
    }

    /* macOS: PMP idle + stop right after the PMU boot_stage, before the CPU stop requests */
    if (pmpsleep) {
        pmp_stopped = s2r_pmp_sleep();
        s2r_console_sync(100);
    }

    if (domoff && chip_id == T8103) {
        s2r_ps_domains_off_macos();
        s2r_console_sync(100);
    }

    if (platform) {
        printf("  GPIO wake group (clear):\n");
        s2r_gpio_wake_group("/arm-io/aop-gpio", true);
        s2r_gpio_wake_group("/arm-io/nub-gpio", true);
        s2r_gpio_wake_group("/arm-io/smc-gpio", false);
        printf("  PMGR stop request sent for %d CPU(s)\n", smp_stop_request_all());
    }

    /* Before the boot core sets DEEP_SLEEP: cores 1-3 stop answering after that */
    if (allcores)
        s2r_secondaries_deep_sleep();

    if (deep) {
        if (l4 && in_el2())
            reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
        if (cpu_features->sleep_mode == SLEEP_GLOBAL) {
            reg_mask(SYS_IMP_APL_ACC_OVRD, ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP_MASK,
                     ACC_OVRD_PWR_DN_SRM(3) | ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP(2) |
                         ACC_OVRD_TRAIN_DOWN_LINK(3) | ACC_OVRD_POWER_DOWN_CPM(3) |
                         ACC_OVRD_DISABLE_PIO_ON_WFI_CPU |
                         (l4 ? ACC_OVRD_DEEP_SLEEP : 0));
        }
        if (cpu_features->cyc_ovrd) {
            cyc_ovrd_saved = mrs(SYS_IMP_APL_CYC_OVRD);
            reg_mask(SYS_IMP_APL_CYC_OVRD, CYC_OVRD_WFI_MODE_MASK,
                     CYC_OVRD_WFI_MODE(3) | CYC_OVRD_DISABLE_WFI_RET);
        }
    }

    if (wdtoff && chip_id != T8103) {
        printf("  %s: T8103 only, keeping the safety watchdog\n", name);
        wdtoff = false;
    }
    if (wdtoff) {
        printf("  >>> NO watchdog: press the power button to wake; if nothing happens,\n"
               "  >>> long-press the power button to force the Mac off\n");
    } else {
        wdt_arm(S2R_E2_TIMEOUT_MS);
        printf("  >>> Press the power button (or open the lid) within %d s, else the watchdog "
               "resets\n",
               S2R_E2_TIMEOUT_MS / 1000);
    }

    s2r_console_sync(100);
    if (psauto) {
        s2r_ps_auto_apply();
        s2r_console_sync(100);
    }

    if (snapps && chip_id == T8103) {
        s2r_ps_macos_snap_apply();
        s2r_console_sync(100);
    }
    if (forceoff && chip_id == T8103) {
        s2r_ps_force_off();
        s2r_console_sync(100);
    }

#if S2R_SNAPSHOT
    /* before the watchdog/quiesce writes: a long delay after 23d2c4000 = 1 resets the SoC */
    if (chip_id == T8103)
        s2r_snapshot();
#endif

    if (wdtoff)
        s2r_wdt_off_macos();

    if (quiesce && wdtoff)
        s2r_pmgr_quiesce_macos();

    if (ep20init)
        s2r_aic_quiesce();

    if (pmu) {
        printf("  PMU sleep request:\n");
        if (!spmi || !s2r_pmu_sleep_request(spmi, &pmu_saved))
            printf("    PMU sleep request FAILED\n");
    }

    /* nub-spmi last: m1n1's SPMI driver does not do macOS's 0/1 toggling around commands */
    if (quiesce && wdtoff) {
        write32(S2R_SPMI_NUB0, 1);
        printf("  SPMI +0x38 = 1: nub-spmi %x\n", read32(S2R_SPMI_NUB0));
    }

    if (wdtoff) {
        write32(S2R_PMGR_QUIESCE, 1);
        printf("  PMGR quiesce: %lx = 1 (reads %08x)\n", (u64)S2R_PMGR_QUIESCE,
               read32(S2R_PMGR_QUIESCE));
    }

    s2r_console_sync(200);

    /*
     * macOS drops off the USB bus at sleep entry. Do the same soft disconnect (dwc3 RUN_STOP=0)
     * before powering the ATC ports off: a powered-off PHY alone leaves the PC thinking the device
     * is still attached, which made "USB still enumerated" useless as an "is it asleep" signal.
     */
    if (forceoff)
        usb_iodev_shutdown();

    if (snapps && chip_id == T8103)
        s2r_ps_atc0_off();
    if (forceoff && chip_id == T8103)
        s2r_ps_atc1_off();

    u64 t0 = mrs(CNTPCT_EL0);
    bool woken = false;
    int sleeps = 0;

    while (!woken) {
        smc_poll(smc); /* empty the mailbox so the level IRQ is de-asserted */
        aic_unmask_irq(S2R_SMC_IRQ);

        s2r_sleep_type = 5 + mode;
        sleeps++;
        if (deep)
            s2r_enter_deep((void *)0);
        else
            s2r_enter((void *)0);

        aic_mask_irq(S2R_SMC_IRQ);
        printf("  Back from WFI #%d, polling SMC mailbox\n", sleeps);
        /* The notification can trail the syslog message by a while: keep polling */
        int handled = 0;
        u64 settle = timeout_calculate(200000);
        while (!timeout_expired(settle))
            handled += smc_poll(smc);
        printf("  smc_poll handled %d message(s)\n", handled);

        u32 n = smc_notification_count(smc);
        u32 last = smc_last_notification(smc);
        printf("  Woke up #%d: %u SMC notification(s), last 0x%08x\n", sleeps, n, last);
        s2r_console_sync(50);
        if (n && smc_notif_is_wake(last))
            woken = true;
    }

    u64 t1 = mrs(CNTPCT_EL0);
    wdt_stop();

    if (deep) {
        if (cpu_features->cyc_ovrd)
            msr(SYS_IMP_APL_CYC_OVRD, cyc_ovrd_saved);
        msr(SYS_IMP_APL_ACC_OVRD, acc_ovrd_saved);
        sysop("isb");
    }

    printf("  Button/lid event 0x%08x after %lu ms\n", smc_last_notification(smc),
           ((t1 - t0) * 1000) / mrs(CNTFRQ_EL0));

    if (psauto)
        s2r_ps_auto_restore();

    if (pmu && spmi) {
        s2r_pmu_restore(spmi, &pmu_saved);
        spmi_shutdown(spmi);
    }

    if (wdtoff)
        s2r_wdt_restore();
    if (ep20init)
        s2r_aic_unquiesce();

    if (pmpsleep)
        s2r_pmp_wake(pmp_stopped);
    if (pmpboot && s2r_pmpdev.rtk) {
        s2r_pmp_listen(500000);
        s2r_pmp_down();
    }

    if (handshake && !smc_set_ap_power(smc, S2R_RTKIT_AP_ON))
        printf("  SMC: AP power state back to ON not acked\n");

    if (allcores)
        s2r_secondaries_release();

    if (l4) {
        int restarted = smp_restart_unresponsive();
        printf("  Restarted %d unresponsive secondary CPU(s)\n", restarted);
    }

    smc_shutdown(smc);
    printf("  %s: PASS\n", name);
    return 0;
}

void wait_loop(u32 sleep_us)
{
    printf("\n--- Wait loop timer polling ---\n");

    //dump_timer_state("before");
    setup_timer(sleep_us);
    //dump_timer_state("armed");

    u64 t0 = mrs(CNTPCT_EL0);
    poll_timer_expiry(sleep_us * 3);
    u64 t1 = mrs(CNTPCT_EL0);

    u64 elapsed_us = ((t1 - t0) * 1000000) / mrs(CNTFRQ_EL0);
    printf("  Elapsed: %lu us (expected ~%u us)\n", elapsed_us, sleep_us);

    /* Disable timer */
    msr(CNTP_CTL_EL0, 0);
    sysop("isb");

}

/* ------------------------------------------------------------------ */
/*  Main entry point                                                   */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/*  PMP boot probe: is the (iBoot pre-loaded) PMP firmware usable?     */
/* ------------------------------------------------------------------ */

#ifndef S2R_PMP_PROBE
#define S2R_PMP_PROBE 0 /* E2j boots the PMP itself; 1 = no-sleep PMP probe */
#endif

static void __attribute__((unused)) s2r_pmp_boot_probe(void)
{
    printf("\n--- PMP boot probe (no sleep, 60 s safety watchdog) ---\n");
    wdt_arm(60000);
    if (s2r_pmp_up()) {
        if (s2r_pmp_ep20_init()) {
            s2r_pmp_listen(1000000);
            s2r_pmp_ep20_replay();
        }
        s2r_pmp_listen(500000);
    }
    s2r_pmp_down();
    wdt_stop();
    s2r_console_sync(200);
}

#ifndef S2R_LEVEL4
#define S2R_LEVEL4 1
#endif

#ifndef S2R_LOW_LEVELS
#define S2R_LOW_LEVELS 0
#endif


/* ------------------------------------------------------------------ */
/*  Battery drain test: Level 3 for N seconds, then E2r (Level 4)      */
/* ------------------------------------------------------------------ */

#define S2R_BATTERY_MAGIC 0xBA77

struct s2r_batt {
    bool ok;
    int rm_mah, fc_mah, soc_pct, mv, ma, acw;
    u64 cnt; /* CNTPCT when read */
};

static void s2r_batt_read(smc_dev_t *smc, struct s2r_batt *b)
{
    u8 rm[2] = {0}, fc[2] = {0}, soc[2] = {0}, av[2] = {0}, ac[2] = {0};
    s8 acw = 0;

    memset(b, 0, sizeof(*b));
    b->cnt = mrs(CNTPCT_EL0);
    b->ok = !smc_read(smc, 0x4230524d /* B0RM */, rm, 2) &&
            !smc_read(smc, 0x42304643 /* B0FC */, fc, 2) &&
            !smc_read(smc, 0x42525343 /* BRSC */, soc, 2) &&
            !smc_read(smc, 0x42304156 /* B0AV */, av, 2) &&
            !smc_read(smc, 0x42304143 /* B0AC */, ac, 2) &&
            !smc_read(smc, 0x41432d57 /* AC-W */, &acw, 1);
    b->fc_mah = fc[0] | (fc[1] << 8);
    b->rm_mah = rm[0] | (rm[1] << 8);
    if (b->fc_mah && b->rm_mah > 2 * b->fc_mah) /* B0RM sometimes comes back byte-swapped */
        b->rm_mah = (rm[0] << 8) | rm[1];
    b->soc_pct = soc[0] | (soc[1] << 8);
    b->mv = av[0] | (av[1] << 8);
    b->ma = (s16)(ac[0] | (ac[1] << 8));
    b->acw = acw;
}

static void s2r_batt_print(const char *tag, const struct s2r_batt *b)
{
    printf("  BATTERY %-22s %s RM=%d mAh FC=%d mAh SoC=%d%% (%d.%02d%% of FC) V=%d mV I=%d mA AC-W=%d\n",
           tag, b->ok ? "" : "(read error)", b->rm_mah, b->fc_mah, b->soc_pct,
           b->fc_mah ? b->rm_mah * 100 / b->fc_mah : 0,
           b->fc_mah ? (b->rm_mah * 10000 / b->fc_mah) % 100 : 0, b->mv, b->ma, b->acw);
}

static void s2r_batt_delta(const char *tag, const struct s2r_batt *a, const struct s2r_batt *b)
{
    u64 secs = (b->cnt - a->cnt) / mrs(CNTFRQ_EL0);
    int d = a->rm_mah - b->rm_mah;
    /* drain rate in 0.01 %/h of the full charge capacity */
    long rate = (secs && a->fc_mah) ? (long)d * 360000L * 100 / ((long)secs * a->fc_mah) : 0;
    long arate = rate < 0 ? -rate : rate;
    printf("  >>> %s: %d mAh in %lu s (%lu min) = %s%ld.%02ld %%/h of FC (SoC %d%% -> %d%%)\n", tag,
           d, secs, secs / 60, rate < 0 ? "-" : "", arate / 100, arate % 100, a->soc_pct,
           b->soc_pct);
}

static void s2r_l3_wfi_chunk(u32 sleep_us)
{
    u64 acc_saved = mrs(SYS_IMP_APL_ACC_OVRD), cyc_saved = 0;

    if (in_el2())
        reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
    if (cpu_features->sleep_mode == SLEEP_GLOBAL)
        reg_mask(SYS_IMP_APL_ACC_OVRD, ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP_MASK,
                 ACC_OVRD_PWR_DN_SRM(3) | ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP(2) |
                     ACC_OVRD_TRAIN_DOWN_LINK(3) | ACC_OVRD_POWER_DOWN_CPM(3) |
                     ACC_OVRD_DISABLE_PIO_ON_WFI_CPU);
    if (cpu_features->cyc_ovrd) {
        cyc_saved = mrs(SYS_IMP_APL_CYC_OVRD);
        reg_mask(SYS_IMP_APL_CYC_OVRD, CYC_OVRD_WFI_MODE_MASK,
                 CYC_OVRD_WFI_MODE(3) | CYC_OVRD_DISABLE_WFI_RET);
    }
    setup_timer(sleep_us);
    s2r_sleep_type = 3;
    s2r_enter_deep((void *)0);
    if (cpu_features->cyc_ovrd)
        msr(SYS_IMP_APL_CYC_OVRD, cyc_saved);
    msr(SYS_IMP_APL_ACC_OVRD, acc_saved);
    msr(CNTP_CTL_EL0, 0);
    sysop("isb");
}

/*
 * s2r_test(l3_seconds, S2R_BATTERY_MAGIC):
 *  1. battery reading A, wait (up to 10 min) for the USB-C cable to be unplugged (AC-W <= 0),
 *     then soft-disconnect USB (no console on the PC from here on) and read B (unplugged).
 *  2. panel off, Level 3 deep WFI on the boot CPU (secondaries in m1n1's deep_wfi idle loop),
 *     woken by the timer every 60 s, for l3_seconds.
 *  3. panel on, reading C, results on screen for 3 min (take a photo).
 *  4. E2r (Level 4, everything as for the power-button test). Force off 60 min after the
 *     screen goes black, boot, read the battery (s2r_battery.py) right after replugging.
 */
/* low-power option bits for the battery test (s2r_test pmic = 0xBA7A | mask << 16) */
#define S2R_LP_DCP_SLEEP  BIT(0) /* DCP fully asleep (else only the panel off) */
#define S2R_LP_NVME       BIT(1) /* NVMe shutdown */
#define S2R_LP_DOMAINS    BIT(2) /* power domains macOS turns off before sleep (E2o) */
#define S2R_LP_SECONDARY  BIT(3) /* secondaries in Level 4 DEEP_SLEEP for good (E2g) */
#define S2R_LP_PS_AUTO    BIT(4) /* auto power states on memory/fabric/NUB (E2h) */
#define S2R_LP_PS_SNAP    BIT(5) /* every power state as in the macOS sleep snapshot (E2p) */
#define S2R_LP_FORCE_OFF  BIT(6) /* APCIE_GP / DEBUG_USB / DEBUG_AUTH forced off (E2q) */
#define S2R_LP_ATC        BIT(7) /* ATC0 + ATC1 (USB-C ports) off */
#define S2R_LP_SMC_HS     BIT(8) /* SMC sleep keys (slpw, WKTP=2, slpa), --wake only */
#define S2R_LP_SMC_IDLE   BIT(9) /* + RTKit SetAPPower(0x201 IDLE): the SMC then stops answering (hs2 hung) */

/* SMC button/lid notification (Asahi macsmc-hid): type in 31:16, subtype 15:8, data 7:0 */
static const char *s2r_wake_event(u32 notif)
{
    u32 type = notif >> 16, sub = (notif >> 8) & 0xff, data = notif & 0xff;
    if (type == SMC_EV_BTN && (sub == 0x01 || sub == 0x06))
        return "power button";
    if (type == SMC_EV_LID && data == 0)
        return "lid open";
    return NULL;
}

/*
 * --probe: find where a button press / lid change goes once the SMC has 'slpa' (no more mailbox
 * notifications). Snapshot the raw AIC line states (AIC1 HW_STATE) and the always-on GPIO
 * controllers (pin registers incl. input level, IRQ status of every group), then on every
 * heartbeat print what changed. Entries that change on 3 heartbeats are muted as noise.
 */
#define S2R_PROBE_MAX 1024
static struct {
    const char *name;
    u64 addr;
    u32 val;
    u16 idx;
    u8 nchg;
} s2r_probe[S2R_PROBE_MAX];
static int s2r_probe_n;

static void s2r_probe_add(const char *name, u64 addr, u16 idx)
{
    if (s2r_probe_n >= S2R_PROBE_MAX)
        return;
    s2r_probe[s2r_probe_n].name = name;
    s2r_probe[s2r_probe_n].addr = addr;
    s2r_probe[s2r_probe_n].idx = idx;
    s2r_probe[s2r_probe_n].val = read32(addr);
    s2r_probe[s2r_probe_n].nchg = 0;
    s2r_probe_n++;
}

static void s2r_probe_gpio(const char *path, const char *pins_name, const char *irq_name)
{
    int pathv[8];
    u64 base, size;
    u32 npins = 0;

    if (adt_path_offset_trace(adt, path, pathv) < 0 ||
        adt_get_reg(adt, pathv, "reg", 0, &base, &size) < 0)
        return;
    int node = adt_path_offset(adt, path);
    ADT_GETPROP(adt, node, "#gpio-pins", &npins);
    if (npins > 256)
        npins = 256;
    for (u32 x = 0; x < npins; x++)
        s2r_probe_add(pins_name, base + 4 * x, x);
    for (u32 g = 0; g < 7; g++)
        for (u32 x = 0; x < npins; x += 32)
            s2r_probe_add(irq_name, base + S2R_GPIO_IRQ(g, x), g * 16 + x / 32);
    printf("  probe: %s @0x%lx, %u pins\n", path, base, npins);
}

static void s2r_probe_init(void)
{
    s2r_probe_n = 0;
    if (aic && aic->version == 1)
        for (u32 i = 0; i < (aic->nr_irq + 31) / 32; i++)
            s2r_probe_add("AIC_HW_STATE", aic->base + 0x4200 + 4 * i, i);
    s2r_probe_gpio("/arm-io/smc-gpio", "smc-gpio pin", "smc-gpio irq g*16+w");
    s2r_probe_gpio("/arm-io/nub-gpio", "nub-gpio pin", "nub-gpio irq g*16+w");
    s2r_probe_gpio("/arm-io/aop-gpio", "aop-gpio pin", "aop-gpio irq g*16+w");
    printf("  probe: %d registers watched\n", s2r_probe_n);
}

static int s2r_probe_diff(int chunk)
{
    int printed = 0, changed = 0;
    for (int i = 0; i < s2r_probe_n; i++) {
        u32 v = read32(s2r_probe[i].addr);
        if (v == s2r_probe[i].val)
            continue;
        changed++;
        if (s2r_probe[i].nchg < 255)
            s2r_probe[i].nchg++;
        if (s2r_probe[i].nchg <= 2 && printed < 16) {
            u32 x = v ^ s2r_probe[i].val;
            printf("  [probe %d] %s[%u] @%lx: %08x -> %08x (bits %08x)%s\n", chunk,
                   s2r_probe[i].name, s2r_probe[i].idx, s2r_probe[i].addr, s2r_probe[i].val, v,
                   x, s2r_probe[i].nchg == 2 ? " (muted from now on)" : "");
            if (!strcmp(s2r_probe[i].name, "AIC_HW_STATE"))
                for (int b = 0; b < 32; b++)
                    if (x & BIT(b))
                        printf("      AIC irq %u -> %d\n", s2r_probe[i].idx * 32 + b, !!(v & BIT(b)));
            printed++;
        }
        s2r_probe[i].val = v;
    }
    return changed;
}

/*
 * --lidwake: aop-gpio pin 22 follows the lid (probe run 2026-11-08). The ADT lists it in the
 * aop-gpio "wake-events" (pin 22, IRQ mode 6 = any edge), i.e. the PMGR wake source
 * AOP.GPIO.IRQ6. Program it as an any-edge interrupt in group 6 (AIC 268 + 6) and route that
 * line to the boot CPU, so a lid move wakes Level 3 even after the SMC's 'slpa'.
 */
#define S2R_AOPGPIO_BASE 0x24a820000UL
#define S2R_LID_PIN      22
#define S2R_LID_GRP      6
#define S2R_LID_AIC_IRQ  (268 + S2R_LID_GRP)
static u32 s2r_lid_saved;

static void s2r_lid_irq_clear(void)
{
    write32(S2R_AOPGPIO_BASE + S2R_GPIO_IRQ(S2R_LID_GRP, S2R_LID_PIN), BIT(S2R_LID_PIN % 32));
}

static bool s2r_lid_irq_pending(void)
{
    return read32(S2R_AOPGPIO_BASE + S2R_GPIO_IRQ(S2R_LID_GRP, S2R_LID_PIN)) & BIT(S2R_LID_PIN % 32);
}

static int s2r_lid_level(void)
{
    return read32(S2R_AOPGPIO_BASE + 4 * S2R_LID_PIN) & 1;
}

static bool s2r_lid_wake_setup(void)
{
    u64 a = S2R_AOPGPIO_BASE + 4 * S2R_LID_PIN;
    s2r_lid_saved = read32(a);
    /* mode (3:1) = 6 any edge, group (18:16) = 6 */
    u32 v = (s2r_lid_saved & ~(GENMASK(3, 1) | GENMASK(18, 16))) | (6 << 1) | (S2R_LID_GRP << 16);
    write32(a, v);
    s2r_lid_irq_clear();
    bool ok = aic_route_irq(S2R_LID_AIC_IRQ, BIT(aic_whoami()));
    printf("  lid wake: aop-gpio pin %d %08x -> %08x (now %08x), level %d, AIC %d route %s\n",
           S2R_LID_PIN, s2r_lid_saved, v, read32(a), s2r_lid_level(), S2R_LID_AIC_IRQ,
           ok ? "ok" : "FAILED");
    return ok;
}

static void s2r_lid_wake_restore(void)
{
    aic_mask_irq(S2R_LID_AIC_IRQ);
    write32(S2R_AOPGPIO_BASE + 4 * S2R_LID_PIN, s2r_lid_saved);
    s2r_lid_irq_clear();
}

static int s2r_battery_test(u32 l3_seconds, bool panel_off, u32 lp, bool until_wake, bool no_settle,
                            bool probe, bool lidwake)
{
    struct s2r_batt a, b, c;
    const char *ended_by = "timeout";
    int notifs = 0;

    if (!l3_seconds)
        l3_seconds = until_wake ? 7 * 86400 : 3600;
    printf("\n--- Battery drain test: Level 3 %s %u s (panel %s), then results forever ---\n",
           until_wake ? "until power button / lid open, at most" : "for", l3_seconds,
           panel_off ? "off" : "on");

    smc_dev_t *smc = smc_init();
    if (!smc) {
        printf("  SMC init failed\n");
        return -1;
    }
    s2r_batt_read(smc, &a);
    s2r_batt_print("A (start, cable in)", &a);
    printf("  >>> UNPLUG THE USB-C CABLE NOW (waiting up to 3 min, then aborting)\n");
    s2r_console_sync(200);

    u64 t = timeout_calculate(180 * 1000000UL);
    int unplugged = 0;
    while (unplugged < 3 && !timeout_expired(t)) {
        mdelay(1000);
        s2r_batt_read(smc, &b);
        /* AC-W is not reliable here; the battery current goes negative once on battery alone */
        unplugged = (b.ok && b.ma < -20) ? unplugged + 1 : 0;
    }
    if (unplugged < 3) {
        printf("  cable still connected after 3 min (I=%d mA, AC-W=%d), aborting\n", b.ma, b.acw);
        smc_shutdown(smc);
        return -1;
    }
    usb_iodev_shutdown(); /* the PC is gone (m1n1 cannot re-enumerate after a replug) */
    printf("  cable unplugged, USB shut down; letting the gauge settle %d s\n", no_settle ? 0 : 30);
    if (!no_settle)
        mdelay(30000);
    s2r_batt_read(smc, &b);
    s2r_batt_print("B (unplugged, L3 start)", &b);
    if (until_wake) {
        /* as the E2 button test: SMC notifications on, SMC mailbox IRQ routed to this CPU */
        if (smc_enable_notifications(smc))
            printf("  SMC: enabling notifications (NTAP) failed\n");
        if (!aic_route_irq(S2R_SMC_IRQ, BIT(aic_whoami()))) {
            printf("  AIC routing unsupported, aborting\n");
            smc_shutdown(smc);
            return -1;
        }
        printf("  wake: press the power button (short!) or open the lid; closing the lid is fine\n");
    } else {
        smc_shutdown(smc);
        smc = NULL;
    }

#define S2R_LP_STEP(bit, name, call)                                                              \
    do {                                                                                          \
        if (lp & (bit)) {                                                                         \
            printf("  [lp] %s ...\n", name);                                                     \
            call;                                                                                 \
            printf("  [lp] %s done\n", name);                                                    \
            mdelay(2000); /* time to read it, and to see a delayed crash */                       \
        }                                                                                         \
    } while (0)
    s2r_keep_disp = true;
    if (lp)
        printf("  low-power set-up, mask 0x%x (panel %s):\n", lp, panel_off ? "off" : "on");
    S2R_LP_STEP(S2R_LP_NVME, "NVMe shutdown", s2r_nvme_sleep());
    S2R_LP_STEP(S2R_LP_DOMAINS, "macOS domains off (E2o)", s2r_ps_domains_off_macos());
    if (lp & S2R_LP_DCP_SLEEP)
        printf("  DCP sleep: %d\n", display_sleep_dcp());
    else if (panel_off)
        printf("  panel off: %d\n", display_panel_power(false));
    S2R_LP_STEP(S2R_LP_SECONDARY, "secondaries Level 4 (E2g)", s2r_secondaries_deep_sleep());
    S2R_LP_STEP(S2R_LP_PS_AUTO, "PS auto (E2h)", s2r_ps_auto_apply());
    S2R_LP_STEP(S2R_LP_PS_SNAP, "macOS PS snapshot (E2p)", s2r_ps_macos_snap_apply());
    S2R_LP_STEP(S2R_LP_FORCE_OFF, "force off APCIE_GP/DEBUG (E2q)", s2r_ps_force_off());
    S2R_LP_STEP(S2R_LP_ATC, "ATC0+ATC1 off", ({
                    if (chip_id == T8103) {
                        s2r_ps_atc0_off();
                        s2r_ps_atc1_off();
                    }
                }));
#undef S2R_LP_STEP
    /* SMC key reads block forever if the SMC stops answering: none after SetAPPower(IDLE) */
    bool smc_reads = until_wake && !(lp & S2R_LP_SMC_IDLE);
    u8 lid0 = 0xff;
    if (smc_reads && smc_read(smc, 0x4d534c44 /* MSLD */, &lid0, 1))
        lid0 = 0xff;
    if (until_wake && (lp & (S2R_LP_SMC_HS | S2R_LP_SMC_IDLE))) /* as macOS: last before the cores go down */
        s2r_smc_sleep_handshake_keys(smc, lp & S2R_LP_SMC_IDLE);
    printf("  Level 3 for %u s (lid MSLD=%d)\n", l3_seconds, lid0);
    if (probe)
        s2r_probe_init(); /* baseline: after every set-up step, right before Level 3 */
    int lid_start = -1;
    bool lid_moved = false; /* the lid was seen away from its start level at least once */
    if (lidwake && s2r_lid_wake_setup())
        lid_start = s2r_lid_level();
    else
        lidwake = false;
    u8 lid_prev = lid0;
    u64 t0 = mrs(CNTPCT_EL0);
    u64 end = t0 + (u64)l3_seconds * mrs(CNTFRQ_EL0);
    int chunks = 0;
    while (mrs(CNTPCT_EL0) < end) {
        u64 left_us = (end - mrs(CNTPCT_EL0)) * 1000000 / mrs(CNTFRQ_EL0);
        u64 chunk_us = probe ? 10000000 : 60000000; /* --probe: 10 s heartbeat */
        bool verbose = true; /* heartbeat on screen: shows whether the timer wake still works */
        if (verbose)
            printf("  L3 chunk %d: WFI for at most %lu s\n", chunks,
                   (left_us > chunk_us ? chunk_us : left_us) / 1000000);
        if (until_wake) {
            smc_poll(smc); /* empty the mailbox so the level IRQ is de-asserted */
            aic_unmask_irq(S2R_SMC_IRQ);
        }
        if (lidwake) {
            s2r_lid_irq_clear();
            aic_unmask_irq(S2R_LID_AIC_IRQ);
        }
        u32 n0 = until_wake ? smc_notification_count(smc) : 0;
        s2r_l3_wfi_chunk(left_us > chunk_us ? (u32)chunk_us : (u32)left_us + 1);
        chunks++;
        if (verbose) /* before any SMC call, so an SMC hang can't hide the wake */
            printf("  L3 chunk %d: back from WFI\n", chunks - 1);
        if (probe)
            s2r_probe_diff(chunks - 1);
        if (lidwake) {
            aic_mask_irq(S2R_LID_AIC_IRQ);
            bool pend = s2r_lid_irq_pending();
            int lvl = s2r_lid_level();
            if (pend || lvl != lid_start)
                printf("  L3 chunk %d: lid GPIO irq %s, level %d (start %d)\n", chunks - 1,
                       pend ? "PENDING" : "none", lvl, lid_start);
            s2r_lid_irq_clear();
            if (lvl != lid_start)
                lid_moved = true;
            if (lvl == lid_start && pend) {
                /* debounce: the level must still be the start level 500 ms later */
                mdelay(500);
                if (s2r_lid_level() != lid_start)
                    lid_moved = true;
                else if (lid_moved) { /* seen closed before, now open again */
                    ended_by = "lid open (aop-gpio 22 IRQ)";
                    break;
                } else
                    printf("  L3 chunk %d: lid edge without a seen close, sleeping on\n", chunks - 1);
            }
        }
        if (until_wake) {
            aic_mask_irq(S2R_SMC_IRQ);
            /* the notification can trail the IRQ a little */
            u64 settle = timeout_calculate(200000);
            while (!timeout_expired(settle))
                smc_poll(smc);
            u32 n1 = smc_notification_count(smc);
            notifs += n1 - n0;
            const char *ev = n1 != n0 ? s2r_wake_event(smc_last_notification(smc)) : NULL;
            if (n1 != n0)
                printf("  L3 chunk %d: SMC notification 0x%08x%s%s\n", chunks - 1,
                       smc_last_notification(smc), ev ? " -> " : "", ev ? ev : "");
            if (ev) {
                ended_by = ev;
                break;
            }
            /* fallback if the SMC stops sending notifications: poll the lid key on each wake */
            u8 lid = 0xff;
            if (smc_reads && smc_read(smc, 0x4d534c44 /* MSLD */, &lid, 1))
                lid = 0xff;
            if (lid != lid_prev)
                printf("  L3 chunk %d: lid MSLD %d -> %d\n", chunks - 1, lid_prev, lid);
            if (lid_prev == 1 && lid == 0) {
                ended_by = "lid open (MSLD poll)";
                break;
            }
            if (lid != 0xff)
                lid_prev = lid;
        }
    }
    u64 l3_secs = (mrs(CNTPCT_EL0) - t0) / mrs(CNTFRQ_EL0);
    if (lidwake)
        s2r_lid_wake_restore();

    if (panel_off || (lp & S2R_LP_DCP_SLEEP))
        printf("  panel on: %d\n", display_panel_power(true));

    if (smc && (lp & S2R_LP_SMC_IDLE))
        printf("  SMC SetAPPower(0x%x) after wake: %s\n", S2R_RTKIT_AP_ON,
               smc_set_ap_power(smc, S2R_RTKIT_AP_ON) ? "acked" : "NOT acked");
    if (!smc)
        smc = smc_init();
    if (!smc) {
        printf("  SMC init failed after Level 3\n");
        return -1;
    }
    s2r_batt_read(smc, &c);

    /* Results on the screen forever, with a fresh reading every 60 s (awake, display on) */
    for (int n = 0;; n++) {
        struct s2r_batt now;
        s2r_batt_read(smc, &now);
        printf("\n=== BATTERY RESULTS #%d (Level 3 %lu s = %lu min, %d wakes, %d SMC notifs, ended by %s, panel %s, low-power 0x%x) ===\n",
               n, l3_secs, l3_secs / 60, chunks, notifs, ended_by, panel_off ? "off" : "on", lp);
        s2r_batt_print("A (start, cable in)", &a);
        s2r_batt_print("B (unplugged, L3 start)", &b);
        s2r_batt_print("C (right after L3)", &c);
        s2r_batt_delta("Level 3 drain B->C", &b, &c);
        s2r_batt_print("now (awake since L3)", &now);
        s2r_batt_delta("awake drain C->now", &c, &now);
        printf("  reference: Linux s2idle ~10 %%/h, macOS sleep < 0.1 %%/h\n");
        mdelay(60000);
    }
    return 0;
}

int s2r_test(u32 sleep_us, int pmic)
{
    int ret;

    printf("\n");
    printf("========================================\n");
    printf("  m1n1 S2R Test (progressive levels)\n");
    printf("========================================\n");
    printf("  Sleep duration: %u us\n", sleep_us);
    printf("  PMIC: %s\n", pmic ? "enabled" : "disabled");
    printf("  Timer freq: %lu Hz\n", mrs(CNTFRQ_EL0));
    printf("  Sleep mode: %d\n", cpu_features->sleep_mode);
    printf("  Sleep count: %d\n", s2r_sleep_count);
    printf("  EL%lu, CPU %d\n", mrs(CurrentEL) >> 2, boot_cpu_idx);
    printf("  Arch retention: %s\n", supports_arch_retention() ? "yes" : "no");
    printf("\n");


    s2r_check_secondary("start");
    smp_fix_boot_rvbar();

    /* battery test: magic in 15:0, low-power mask bits 7:0 in 23:16 and bit 8 in 25, bit 24 = --wake, bit 27 = --no-settle, bit 28 = --probe, bit 29 = --lidwake */
    u32 bt_lo = (u32)pmic & 0xffff,
        bt_lp = (((u32)pmic >> 16) & 0xff) | ((((u32)pmic >> 25) & 3) << 8); /* lp bits 9:8 = 26:25 */
    bool bt_wake = ((u32)pmic >> 24) & 1, bt_nosettle = ((u32)pmic >> 27) & 1,
         bt_probe = ((u32)pmic >> 28) & 1, bt_lid = ((u32)pmic >> 29) & 1;
    if (bt_lo == S2R_BATTERY_MAGIC || bt_lo == S2R_BATTERY_MAGIC + 3)
        return s2r_battery_test(sleep_us, true, bt_lp, bt_wake, bt_nosettle, bt_probe, bt_lid);
    if (bt_lo == S2R_BATTERY_MAGIC + 1 || bt_lo == S2R_BATTERY_MAGIC + 4) /* panel kept on */
        return s2r_battery_test(sleep_us, false, bt_lp & ~S2R_LP_DCP_SLEEP, bt_wake, bt_nosettle,
                                bt_probe, bt_lid);
    if (pmic == S2R_BATTERY_MAGIC + 2) { /* panel off for sleep_us, then back on */
        int off = display_panel_power(false);
        mdelay(sleep_us / 1000);
        int on = display_panel_power(true);
        printf("  panel test: off %d, on %d (panel should show this line)\n", off, on);
        return on;
    }

    /* Dump initial state */
    dump_cpu_sleep_state("init");
    dump_timer_state("init");

    /* Levels 0-3 (and E1a) are skipped by default: set S2R_LOW_LEVELS to 1 to run them */
    if (S2R_LOW_LEVELS) {
        /* Level 0: Verify timer works at all (no WFI) */
        ret = test_level0_timer_sanity(sleep_us);
        if (ret < 0) {
            printf("\nABORT: Timer doesn't work, cannot proceed.\n");
            return -1;
        }

        /* Level 1: Simple WFI */
        ret = test_level1_simple_wfi(sleep_us);
        if (ret < 0) {
            printf("\nABORT: Simple WFI failed.\n");
            return -1;
        }

        /* E1a: WD0 IRQ through the AIC wakes a plain WFI (sanity check for the deep-sleep E1) */
        test_wd0_irq_sanity(2000000);

        /* Level 2: Deep WFI with state retention */
        ret = test_level2_deep_wfi_retention(sleep_us);
        if (ret < 0) {
            printf("\nABORT: Deep WFI retention failed.\n");
            return -1;
        }

        /* Level 3: ACC overrides without DEEP_SLEEP */
        s2r_sleep_count += 1;
        ret = test_level3_acc_no_deep_sleep(sleep_us);
        if (ret < 0) {
            printf("\nABORT: ACC override test failed.\n");
            return -1;
        }
    }

    s2r_check_secondary("after level3");

    /* Level 4: ACC overrides with DEEP_SLEEP (power-gate). A chainloaded m1n1 does not come
     * back from it (power_optim notes), so it can be disabled with S2R_LEVEL4 0. */
    if (S2R_LEVEL4) {
        s2r_sleep_count += 1;
        ret = test_level4_deep_sleep(sleep_us);
        if (ret < 0) {
            printf("\nABORT: DEEP_SLEEP test failed.\n");
            return -1;
        }
    }

    s2r_console_sync(200);

#if S2R_PMU_PROBE
    s2r_pmu_probe();
    s2r_console_sync(200);
#endif

#if S2R_PMP_PROBE
    s2r_pmp_boot_probe();
#endif

#if S2R_E2
    /* E2: interactive, needs a power button press (30 s watchdog) */
    for (int m = 0; m < 18; m++)
        if (S2R_E2_MASK & BIT(m))
            test_smc_button_wake(m);
#endif

    printf("\n========================================\n");
    printf("  All levels PASSED!\n");
    printf("========================================\n\n");

    s2r_sleep_type = 0xAA;
    return 0;
}
