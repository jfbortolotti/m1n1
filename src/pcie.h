/* SPDX-License-Identifier: MIT */

#ifndef PCIE_H
#define PCIE_H

int pcie_init(void);
int pcie_shutdown(void);

/*
 * t8103 apcie only (kboot dt_set_apcie_tunables, Linux pcie-apple power_off_gp): the PHY "fuse"
 * writes pcie_init_controller() does, as up to max (offset from PHY IP reg[3], mask, value)
 * triples in out[]. Returns the number of triples, or -1.
 */
int pcie_t8103_fuse_tunables(const char *path, u32 *out, int max);

#endif
