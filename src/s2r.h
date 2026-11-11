/* SPDX-License-Identifier: MIT */

#ifndef S2R_H
#define S2R_H

#include "types.h"

/*
 * S2R (Suspend to RAM) progressive test harness.
 *
 * Tests CPU sleep mechanisms at increasing depth:
 *   Level 0: Timer poll (sanity check)
 *   Level 1: Simple WFI + timer wake
 *   Level 2: Deep WFI with retention (CYC_OVRD)
 *   Level 3: ACC overrides (without DEEP_SLEEP power-gate)
 */

/*
 * s2r_sleep_type — debug context variable
 *
 * Set before each sleep entry so the exception handler and early boot
 * code can report what sleep level was being attempted.
 *
 * Values:
 *   0xFF = initial/not set
 *   0    = level 0 (timer poll, no WFI)
 *   1    = level 1 (simple WFI)
 *   2    = level 2 (deep WFI retention)
 *   3    = level 3 (ACC overrides, no DEEP_SLEEP)
 *   4    = level 4 (full deep sleep with ACC_OVRD_DEEP_SLEEP)
 *   5    = E2a (SMC button wake, plain WFI)
 *   6    = E2b (SMC button wake, Level 3 deep WFI)
 *   7    = E2c (SMC button wake, Level 4 DEEP_SLEEP, no waker core)
 *   8    = E2d (E2c after the macOS SMC sleep handshake)
 *   9    = E2e (E2d + GPIO wake group clear + per-core PMGR stop requests)
 *   10   = E2f (E2e + PMU boot_stage, sleep timestamp and sleep flag)
 *   11   = E2g (E2f with every secondary in deep sleep too)
 *   12   = E2h (E2g + macOS auto power states on memory/fabric/NUB domains)
 *   13   = E2i (E2h + PMP coprocessor idle and CPU stopped)
 *   14   = E2j (E2i with the PMP booted first, as under macOS)
 *   15   = E2k (E2j with all watchdogs off as macOS does, no safety watchdog)
 *   16   = E2l (E2k + display off + PMGR quiesce replay + SPMI +0x38)
 *   17   = E2m (E2l + SEP sleep message, NVMe shutdown, PMP ep 0x20 replay)
 *   18   = E2n (E2m + PMP ep 0x20 init as at macOS boot + AIC IPI quiesce)
 *   19   = E2o (E2n + power domains off as macOS leaves them before sleep)
 *   20   = E2p (E2o + every power state as in the macOS snapshot of hypervisor trace 8)
 *   21   = E2q (E2p + SEP/APCIE_GP/DEBUG_USB/DEBUG_AUTH forced off, ATC1 USB off last)
 *   22   = E2r (E2q + AOP booted, then SetAPPower 0x201 before sleep as macOS)
 *   0xAA = test complete
 */
extern volatile u32 s2r_sleep_type;

/*
 * s2r_test - run progressive S2R tests
 *
 * @sleep_us: timer duration in microseconds
 * @pmic:     if nonzero, send SPMI sleep/wakeup commands (reserved)
 *
 * Returns 0 on success, negative on failure.
 */
int s2r_test(u32 sleep_us, int pmic);

#define S2R_CTX_MAGIC 0x53325243545831ULL /* "S2RCTX1" */

/*
 * Linux s2idle core power-gating resume handoff (EXPERIMENTAL). kboot reserves the m1n1 image
 * in the Linux DT and publishes the physical address of this struct in
 * /chosen "asahi,m1n1-resume". Before a WFI with CYC_OVRD.DISABLE_WFI_RET, Linux writes ->entry
 * (physical address of cpu_resume) and ->magic, cleaned to PoC. When the boot CPU then comes
 * back through RVBAR, _cpu_reset_c() sees the magic before printing anything, clears it, runs
 * the CPU init quietly and jumps to ->entry at EL2 with the MMU and caches off.
 */
#define S2R_LINUX_RESUME_MAGIC 0x4c4e58524553554dULL /* "LNXRESUM" */
struct s2r_linux_resume {
    u64 magic;
    u64 entry;
    u64 count; /* resumes done, for Linux's statistics */
    u64 pad[5];
} __attribute__((aligned(64)));
extern struct s2r_linux_resume s2r_linux_resume;

/* E2n/E2o: reset through the watchdog as soon as the boot CPU wakes (see _cpu_reset_c) */
#define S2R_WAKE_BEACON

/* Saved CPU context for resume after a reset-vector wake (offsets in s2r_asm.S) */
struct s2r_ctx {
    u64 magic;
    u64 sp;
    u64 tpidr;
    u64 regs[13]; /* x18..x30 */
};
extern struct s2r_ctx s2r_ctx;

void s2r_flush(void);

/* Raw CNTPCT timestamps used to work out who woke the core (level 4) */
struct s2r_timestamps {
    u64 reset_entry; /* top of _cpu_reset_c on the resuming core */
    u64 waker_start; /* waker core started running */
    u64 waker_ipi;   /* waker sent the IPI */
    u64 waker_pmgr;  /* waker did the PMGR start write */
};
extern volatile struct s2r_timestamps s2r_ts;

/*
 * On-screen breadcrumbs for debugging the reset-vector resume without UART:
 * s2r_crumb(n) lights the n-th white square along the bottom of the screen
 * by writing straight to the physical framebuffer (safe with the MMU off).
 * No-op until s2r_crumb_init() ran.
 *
 *   0 = about to WFI      1 = _cpu_reset_c entered   2 = init_cpu done
 *   3 = exceptions ready  4 = MMU back on, about to s2r_restore
 *   6 = s2r_enter_deep returned in the test
 *   7 = waker sent IPI    8 = waker did PMGR start    10 = smp_call4 to waker returned
 *   11 = stop request written    12 = waker started running
 */
void s2r_crumb_init(void);
void s2r_crumb(int stage);
void s2r_crumb_disable(void);

/* Restore saved context and "return 1" from s2r_enter. Does not return. */
void s2r_restore(void) __attribute__((noreturn));

/*
 * s2r_enter - assembly WFI with callee-saved register save/restore
 *
 * @ctx: unused pointer (reserved for future context save)
 *
 * Returns 1 on normal wake (state retained).
 */
int s2r_enter(void *ctx);

/*
 * s2r_enter_deep - like s2r_enter, but saves the context in s2r_ctx and
 * cleans caches so it also returns 1 when the CPU wakes through the reset
 * vector (resumed by _cpu_reset_c). Used by level 3 and deeper.
 */
int s2r_enter_deep(void *ctx);

/*
* util function to wait on timer polling
*/
void wait_loop(u32 sleep_us);

#endif
