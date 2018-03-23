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

#include "hw/arm/arm.h"
#include "qemu-common.h"
#include "hw/sysbus.h"

/*
 * struct that contains dimensioning parameters of the platform bus
 */
typedef struct {
    hwaddr fslmc_bus_base; /* start address of the bus */
    hwaddr fslmc_bus_size; /* size of the bus */
} FSLMCBusSystemParams;

/*
 * struct that contains all relevant info to build the fdt nodes of
 * platform bus and attached dynamic sysbus devices
 * in the future might be augmented with additional info
 * such as PHY, CLK handles ...
 */
typedef struct {
    const FSLMCBusSystemParams *system_params;
    struct arm_boot_info *binfo;
    const char *intc; /* parent interrupt controller name */
} FSLMCBusFDTParams;

/**
 * arm_register_platform_bus_fdt_creator - register a machine init done
 * notifier that creates the device tree nodes of the platform bus and
 * associated dynamic sysbus devices
 */
void fsl_register_mc_bus_fdt_creator(FSLMCBusFDTParams *fdt_params);

#endif
