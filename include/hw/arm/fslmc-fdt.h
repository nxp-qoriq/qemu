/*
 * Freescale Management Complex (MC) Dynamic device tree
 * node generation API
 *
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * Author: Bharat Bhushan <bharat.bhushan@nxp.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.
 * See the COPYING file in the top-level directory.
 *
 */

#ifndef HW_FSL_MC_FDT_H
#define HW_FSL_MC_FDT_H

#include "exec/hwaddr.h"

/**
 * add_fsl_mc_bus_fdt_node - create all fsl mc bus node
 *
 * builds the root fsl mc bus node and map regions of other devices
 */
void add_fsl_mc_bus_fdt_node(void *fdt, const char *intc, hwaddr addr);

#endif
