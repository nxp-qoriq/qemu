/*
 * Freescale Management Complex (MC) device
 *
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
 *
 * Author: Bharat Bhushan <bharat.bhushan@nxp.com>
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of  the GNU General  Public License as published by
 * the Free Software Foundation;  either version 2 of the  License, or
 * (at your option) any later version.
 *
 */

#if !defined(FSL_MC_FSL_MC_H)
#define FSL_MC_FSL_MC_H

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "hw/pci/msi.h"

/**** MC Portals ****/
/* DPRC and DPMCP falls in this regions */
/* Size of each MC Portal */
#define FSLMC_NUM_MCPORTALS 1024
#define FSLMC_MCPORTAL_SIZE 0x10000
#define FSLMC_MCPORTALS_SIZE (FSLMC_MCPORTAL_SIZE * FSLMC_NUM_MCPORTALS)

/**** QBMan Portals ****/
/* Each QBman portal have two regions, CI and CE, of same size */
#define FSLMC_QBMAN_NUM_PORTALS 1024
#define FSLMC_QBMAN_REGION_SIZE 0x10000
#define FSLMC_QBMAN_PORTALS_CI_SIZE (FSLMC_QBMAN_REGION_SIZE * \
                                       FSLMC_QBMAN_NUM_PORTALS)
#define FSLMC_QBMAN_PORTALS_CE_SIZE (FSLMC_QBMAN_PORTALS_CI_SIZE)
#define FSLMC_QBMAN_PORTALS_CESH_SIZE (FSLMC_QBMAN_PORTALS_CI_SIZE)
#define FSLMC_QBMAN_PORTALS_SIZE (FSLMC_QBMAN_PORTALS_CI_SIZE + \
                                  FSLMC_QBMAN_PORTALS_CE_SIZE + \
                                  FSLMC_QBMAN_PORTALS_CESH_SIZE)

#define FSLMC_HOST_SYSFS_PATH  "/sys/bus/fsl-mc/devices/"
#define FSLMC_DEV_SYSPATH_LEN 100
#define FSLMC_DEV_NAME_LEN 10

struct FslMcBusState;

#define TYPE_FSL_MC_BUS "fsl-mc-bus"
#define FSL_MC_BUS(obj) OBJECT_CHECK(FslMcBusState, (obj), TYPE_FSL_MC_BUS)

struct FslMcBusState {
    BusState qbus;

    QLIST_HEAD(, FslMcDeviceState) device_list;
};
typedef struct FslMcBusState FslMcBusState;

#define TYPE_FSL_MC_HOST "fsl-mc-host"
#define FSL_MC_HOST(obj) OBJECT_CHECK(FslMcHostState, (obj), TYPE_FSL_MC_HOST)

typedef struct FslMcHostState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/
    FslMcBusState bus;
    uint64_t mc_bus_base_addr;
    MemoryRegion mc_portal;
    uint64_t mc_portals_offset;
    uint64_t mc_portals_size;
    MemoryRegion qbman_portal;
    uint64_t qbman_portals_offset;
    uint64_t qbman_portals_size;
    uint64_t qbman_portals_cesh_offset;
    uint64_t qbman_portals_cesh_size;
    uint64_t qbman_portals_ce_offset;
    uint64_t qbman_portals_ce_size;
    uint64_t qbman_portals_ci_offset;
    uint64_t qbman_portals_ci_size;
    uint32_t mc_bus_devid_start;
    uint32_t mc_bus_devid_num;
} FslMcHostState;

typedef struct FslMcHostClass {
    DeviceClass parent_class;
} FslMcHostClass;

#define TYPE_FSL_MC_DEVICE "fsl-mc-device"
#define FSL_MC_DEVICE(obj) OBJECT_CHECK(FslMcDeviceState, (obj), TYPE_FSL_MC_DEVICE)
#define FSL_MC_DEVICE_CLASS(klass) \
         OBJECT_CLASS_CHECK(FslMcDeviceClass, (klass), TYPE_FSL_MC_DEVICE)
#define FSL_MC_DEVICE_GET_CLASS(obj) \
        OBJECT_GET_CLASS(FslMcDeviceClass, (obj), TYPE_FSL_MC_DEVICE)

struct mcdev_region {
    off_t offset;
    size_t size;
};

/* Define max number of regions/irqs in a mc-device.
 * 10 is safe value for both as of now.
 * max number of regions is 2 and max number of irqs
 * is 1 of any known mc device.
 * TODO Remove static array for regions and irqs */
#define FSLMC_MAX_REGIONS 10
#define FSLMC_MAX_IRQS 10

struct mcdev_irqs {
    uint8_t irq_index;
    MSIMessage msi_msg;
};

typedef struct FslMcDeviceState {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/
    DeviceState qdev;
    FslMcBusState *bus;
    struct FslMcDeviceState *parent_mcdev;
    uint32_t device_id;
    struct mcdev_region regions[FSLMC_MAX_REGIONS];
    struct mcdev_irqs irqs[FSLMC_MAX_IRQS];
    QLIST_ENTRY(FslMcDeviceState) next;
} FslMcDeviceState;

typedef struct FslMcDeviceClass {
    DeviceClass parent_class;

    void (*realize)(FslMcDeviceState *dev, Error **errp);
    int (*init)(FslMcDeviceState *mcdev);
    int (*exit)(FslMcDeviceState *mcdev);
} FslMcDeviceClass;

int fsl_mc_register_device(FslMcDeviceState *mcdev, FslMcDeviceState *pmcdev,
                           char *device_type);
int fsl_mc_register_device_region(FslMcDeviceState *mcdev, int region_num,
                                  MemoryRegion *mem, char *device_type);
MSIMessage fslmc_get_msi_message(FslMcDeviceState *mcdev, uint8_t index);
void fslmc_set_msi_message(FslMcDeviceState *mcdev, MSIMessage msg,
                           uint8_t index);
uint32_t fsl_mc_get_device_id(FslMcDeviceState *mcdev);
uint64_t fslmc_get_region_addr(FslMcDeviceState *mcdev, uint8_t index);
uint32_t fslmc_get_region_size(FslMcDeviceState *mcdev, uint8_t index);
int fsl_mc_get_portal_ranges(hwaddr *mc_p_addr, hwaddr *mc_p_size,
                             hwaddr *qbman_p_addr, hwaddr *qbman_p_size);
int fsl_mc_get_root_mc_portal_region(hwaddr *mc_p_addr, hwaddr *mc_p_size);
int fslmc_get_region_base_and_offset(FslMcDeviceState *mcdev, uint8_t index,
                                     uint32_t *offset, uint64_t *base,
                                     char *device_type);
#endif /* !defined(FSL_MC_FSL_MC_H) */
