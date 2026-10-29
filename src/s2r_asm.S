/* SPDX-License-Identifier: MIT */

#include "cpu_regs.h"

/*
 * s2r_enter(void *ctx)
 *
 * Saves callee-saved registers (x18-x30) and SP onto the stack,
 * then executes WFI.
 *
 * On wake (state retained), restores all registers and returns 1.
 *
 * x0 = reserved context pointer (can be NULL)
 * Returns: 1 on normal wake (state retained)
 */

/*
 * Context layout (struct s2r_ctx in s2r.c) -- keep in sync.
 */
#define CTX_MAGIC   0
#define CTX_SP      8
#define CTX_TPIDR   16
#define CTX_REGS    24 /* x18..x30 */

.text

/*
 * s2r_enter(void *ctx): shallow variant for levels 1/2. Saves callee-saved
 * registers on the stack, executes WFI, restores and returns 1. Assumes the
 * CPU state is retained (no reset on wake).
 */
.globl s2r_enter
.type s2r_enter, @function
s2r_enter:
    stp x30, x29, [sp, #-16]!
    stp x28, x27, [sp, #-16]!
    stp x26, x25, [sp, #-16]!
    stp x24, x23, [sp, #-16]!
    stp x22, x21, [sp, #-16]!
    stp x20, x19, [sp, #-16]!
    stp x18, xzr, [sp, #-16]!

    dsb sy
    isb
    wfi
    isb

    ldp x18, xzr, [sp], #16
    ldp x20, x19, [sp], #16
    ldp x22, x21, [sp], #16
    ldp x24, x23, [sp], #16
    ldp x26, x25, [sp], #16
    ldp x28, x27, [sp], #16
    ldp x30, x29, [sp], #16

    mov x0, #1
    ret

/*
 * s2r_enter_deep(void *ctx): level 3+ variant. Saves the context into
 * s2r_ctx and cleans caches so the CPU can be resumed from the reset vector
 * (see _cpu_reset_c). Returns 1 either way.
 */
.globl s2r_enter_deep
.type s2r_enter_deep, @function
s2r_enter_deep:
    adrp x1, s2r_ctx
    add x1, x1, :lo12:s2r_ctx

    mov x2, sp
    str x2, [x1, #CTX_SP]
    mrs x2, tpidr_el2
    str x2, [x1, #CTX_TPIDR]
    stp x18, x19, [x1, #(CTX_REGS + 0)]
    stp x20, x21, [x1, #(CTX_REGS + 16)]
    stp x22, x23, [x1, #(CTX_REGS + 32)]
    stp x24, x25, [x1, #(CTX_REGS + 48)]
    stp x26, x27, [x1, #(CTX_REGS + 64)]
    stp x28, x29, [x1, #(CTX_REGS + 80)]
    str x30,      [x1, #(CTX_REGS + 96)]

    /* Set magic and clean all caches to PoC (clobbers x30, restored below) */
    bl s2r_flush

    adrp x1, s2r_ctx
    add x1, x1, :lo12:s2r_ctx
    ldr x30, [x1, #(CTX_REGS + 96)]

    dsb sy
    isb

    /* Enter sleep -- CPU halts here until interrupt */
    wfi

    /* Shallow wake (state retained): fall through to common restore */
    isb

/*
 * s2r_restore: reload SP/TPIDR/callee-saved regs from s2r_ctx, clear the
 * magic and return 1 to the caller of s2r_enter_deep. Also reached from the reset
 * path (MMU already re-enabled) from _cpu_reset_c.
 */
.globl s2r_restore
.type s2r_restore, @function
s2r_restore:
    adrp x1, s2r_ctx
    add x1, x1, :lo12:s2r_ctx

    str xzr, [x1, #CTX_MAGIC]

    ldr x2, [x1, #CTX_TPIDR]
    msr tpidr_el2, x2
    ldr x2, [x1, #CTX_SP]
    mov sp, x2
    ldp x18, x19, [x1, #(CTX_REGS + 0)]
    ldp x20, x21, [x1, #(CTX_REGS + 16)]
    ldp x22, x23, [x1, #(CTX_REGS + 32)]
    ldp x24, x25, [x1, #(CTX_REGS + 48)]
    ldp x26, x27, [x1, #(CTX_REGS + 64)]
    ldp x28, x29, [x1, #(CTX_REGS + 80)]
    ldr x30,      [x1, #(CTX_REGS + 96)]

    /* Return 1 = woke up (state restored) */
    mov x0, #1
    ret
