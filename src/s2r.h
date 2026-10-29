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
