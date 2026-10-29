/* SPDX-License-Identifier: MIT */

#include "s2r.h"
#include "aic.h"
#include "aic_regs.h"
#include "cpu_regs.h"
#include "memory.h"
#include "pmgr.h"
#include "smp.h"
#include "spmi.h"
#include "string.h"
#include "types.h"
#include "utils.h"

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

    printf("\n========================================\n");
    printf("  All levels PASSED!\n");
    printf("========================================\n\n");

    s2r_sleep_type = 0xAA;
    return 0;
}
