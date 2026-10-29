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
#include "spmi.h"
#include "string.h"
#include "types.h"
#include "fb.h"
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
#define S2R_E2_MASK 0x7 /* bit n: run mode n (0 = E2a, 1 = E2b, 2 = E2c) */
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

/*
 * Sleep until the SMC reports a power button or lid event (or the watchdog fires).
 * deep=false: plain WFI (E2a). deep=true: Level 3 overrides + reset-vector resume (E2b).
 */
static int test_smc_button_wake(int mode)
{
    bool deep = mode >= 1;
    const char *name = mode == 2 ? "E2c" : deep ? "E2b" : "E2a";

    printf("\n--- %s: SMC power button wake from %s ---\n", name,
           mode == 2 ? "Level 4 DEEP_SLEEP (power-gated core, no waker core)"
                     : deep ? "Level 3 deep WFI (reset-vector resume)" : "plain WFI");

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

    if (mode == 2 && cpu_features->sleep_mode != SLEEP_GLOBAL) {
        printf("  E2c only implemented for SLEEP_GLOBAL, skipping\n");
        smc_shutdown(smc);
        return 0;
    }

    if (deep) {
        if (mode == 2 && in_el2())
            reg_set(SYS_IMP_APL_VM_TMR_FIQ_ENA_EL2, VM_TMR_FIQ_ENA_ENA_P);
        if (cpu_features->sleep_mode == SLEEP_GLOBAL) {
            reg_mask(SYS_IMP_APL_ACC_OVRD, ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP_MASK,
                     ACC_OVRD_PWR_DN_SRM(3) | ACC_OVRD_DIS_L2_FLUSH_ACC_SLEEP(2) |
                         ACC_OVRD_TRAIN_DOWN_LINK(3) | ACC_OVRD_POWER_DOWN_CPM(3) |
                         ACC_OVRD_DISABLE_PIO_ON_WFI_CPU |
                         (mode == 2 ? ACC_OVRD_DEEP_SLEEP : 0));
        }
        if (cpu_features->cyc_ovrd) {
            cyc_ovrd_saved = mrs(SYS_IMP_APL_CYC_OVRD);
            reg_mask(SYS_IMP_APL_CYC_OVRD, CYC_OVRD_WFI_MODE_MASK,
                     CYC_OVRD_WFI_MODE(3) | CYC_OVRD_DISABLE_WFI_RET);
        }
    }

    wdt_arm(S2R_E2_TIMEOUT_MS);
    printf("  >>> Press the power button (or open the lid) within %d s, else the watchdog resets\n",
           S2R_E2_TIMEOUT_MS / 1000);

    u64 t0 = mrs(CNTPCT_EL0);
    bool woken = false;
    int sleeps = 0;

    while (!woken) {
        smc_poll(smc); /* empty the mailbox so the level IRQ is de-asserted */
        aic_unmask_irq(S2R_SMC_IRQ);

        s2r_sleep_type = mode == 2 ? 7 : deep ? 6 : 5;
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

    if (mode == 2) {
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


    wait_loop(10000000);

    s2r_check_secondary("start");

    /* Dump initial state */
    dump_cpu_sleep_state("init");
    dump_timer_state("init");

//#if 1
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
//#endif
    /* Level 3: ACC overrides without DEEP_SLEEP */
    s2r_sleep_count +=1;
    ret = test_level3_acc_no_deep_sleep(sleep_us);
    if (ret < 0) {
        printf("\nABORT: ACC override test failed.\n");
        return -1;
    }

    s2r_check_secondary("after level3");

    /* Level 4: ACC overrides with DEEP_SLEEP (power-gate) */
    s2r_sleep_count += 1;
    ret = test_level4_deep_sleep(sleep_us);
    if (ret < 0) {
        printf("\nABORT: DEEP_SLEEP test failed.\n");
        return -1;
    }

#if S2R_E2
    /* E2: interactive, needs a power button press (30 s watchdog) */
    for (int m = 0; m < 3; m++)
        if (S2R_E2_MASK & BIT(m))
            test_smc_button_wake(m);
#endif

    printf("\n========================================\n");
    printf("  All levels PASSED!\n");
    printf("========================================\n\n");

    s2r_sleep_type = 0xAA;
    return 0;
}
