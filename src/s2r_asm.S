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

.text

.globl s2r_enter
.type s2r_enter, @function
s2r_enter:
    /* Save callee-saved registers on the stack */
    stp x30, x29, [sp, #-16]!
    stp x28, x27, [sp, #-16]!
    stp x26, x25, [sp, #-16]!
    stp x24, x23, [sp, #-16]!
    stp x22, x21, [sp, #-16]!
    stp x20, x19, [sp, #-16]!
    stp x18, xzr, [sp, #-16]!

    /* Ensure all stores are visible before WFI */
    dsb sy
    isb

    /* Enter sleep — CPU halts here until interrupt */
    wfi

    /* ---- Wake up: CPU resumes here if state was retained ---- */

    isb

    /* Restore callee-saved registers */
    ldp x18, xzr, [sp], #16
    ldp x20, x19, [sp], #16
    ldp x22, x21, [sp], #16
    ldp x24, x23, [sp], #16
    ldp x26, x25, [sp], #16
    ldp x28, x27, [sp], #16
    ldp x30, x29, [sp], #16

    /* Return 1 = woke up normally with state retained */
    mov x0, #1
    ret
