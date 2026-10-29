/* SPDX-License-Identifier: MIT */

#ifndef SMC_H
#define SMC_H

#include "asc.h"
#include "rtkit.h"
#include "types.h"

typedef struct smc_dev smc_dev_t;

int smc_write_u32(smc_dev_t *smc, u32 key, u32 value);
int smc_write_flag(smc_dev_t *smc, u32 key, u8 value);
int smc_enable_notifications(smc_dev_t *smc);

smc_dev_t *smc_init(void);
void smc_shutdown(smc_dev_t *smc);

/*
 * Non-blocking mailbox service: handle every pending SMC message (notifications are
 * recorded). Returns the number of messages handled.
 */
int smc_poll(smc_dev_t *smc);
u32 smc_notification_count(smc_dev_t *smc);
u32 smc_last_notification(smc_dev_t *smc);

#endif
