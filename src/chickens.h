/* SPDX-License-Identifier: MIT */

#ifndef __CHICKENS_H__
#define __CHICKENS_H__

void init_cpu(void);
void init_cpu_quiet(void); /* same, no printf (MMU off, Linux owns the console) */

#endif
