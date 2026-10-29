/* SPDX-License-Identifier: MIT */

#ifndef __WDT_H__
#define __WDT_H__

#include "types.h"

void wdt_disable(void);
void wdt_reboot(void);

/* Arm the watchdog to reset the SoC after timeout_ms (needs wdt_disable() to have run once) */
void wdt_arm(u32 timeout_ms);
/* Stop the watchdog (no ADT lookup, usable right after a reset-vector resume) */
void wdt_stop(void);

/*
 * WD0 "bark" interrupt (WDT block, AIC IRQ 338 on T8103). Never sets RESET_EN
 * and never touches BITE_TIME: WD0 bite resets the machine to recovery mode.
 * wdt_wd0_irq_arm fires the IRQ after timeout_ms; wdt_wd0_irq_clear stops it and clears status.
 * wdt_wd0_irq_adt prints the ADT interrupts of the wdt node and returns the first one (or -1).
 */
void wdt_wd0_irq_arm(u32 timeout_ms);
void wdt_wd0_irq_clear(void);
int wdt_wd0_irq_adt(void);

#endif
