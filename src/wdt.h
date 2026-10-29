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

#endif
