/*
 * Freescale Management Complex (MC) device
 *
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * Author: Bharat Bhushan <bharat.bhushan@nxp.com>
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of  the GNU General  Public License as published by
 * the Free Software Foundation;  either version 2 of the  License, or
 * (at your option) any later version.
 *
 * *****************************************************************
 *
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "fsl-mc.h"

/* For Linear allocation of device-id */
static uint32_t device_id;

/* QBMan have 1024 h/w portals and each QBMan h/w portal have three
 * address region:
 *  1) Cache Inhibited (CI) region of size 0x10000
 *  2) Cache Enable and non-shareable (CE_NS) region of size 0x10000
 *  3) Cache Enable and Shareable (CE_SH) region of size 0x10000
 *
 * CI regions of all QBMan portals are placed in contiguous address range.
 * Similarly CE_NS and CE_SH regions of all QBMan portals are also placed
 * in respective contiguous address ranges.
 *
 * CI address range starts from offset 0x0 of QBman address-space, CE_NS
 * address range is placed just after CI address space and CE_SH address
 * range is placed just after CE_NS address range.
 *
 * To support 1024 QBMan portals following ranges are defined:
 * CI address range = offset-0x0 to 0x4000000
 * CE_NS address range = offset-0x4000000 to 0x8000000
 * CE_SH address range = offset-0x8000000 to 0xC000000
 *
 * qbman_ci_offset, qbman_ce_offset and qbman_cesh_offset are  used for
 * linear allocation in respective address range .
 */
static uint64_t qbman_ce_offset;
static uint64_t qbman_ci_offset;
static uint64_t qbman_cesh_offset;

/* MC portals are in one contiguous range and each mc-portal is
 * of size 0x10000.
 * mcportal_offset is used for linear allocation of mc-portal address
 */
static uint64_t mcportal_offset;

static Property fsl_mc_props[] = {
    DEFINE_PROP_UINT64("mc_bus_base_addr", FslMcHostState, mc_bus_base_addr, 0),
    DEFINE_PROP_UINT64("mc_portals_offset", FslMcHostState,
                       mc_portals_offset, 0),
    DEFINE_PROP_UINT64("mc_portals_size", FslMcHostState,
                       mc_portals_size, 0),
    DEFINE_PROP_UINT64("qbman_portals_offset", FslMcHostState,
                       qbman_portals_offset, 0),
    DEFINE_PROP_UINT64("qbman_portals_size", FslMcHostState,
                       qbman_portals_size, 0),
    DEFINE_PROP_UINT32("mc_bus_devid_start", FslMcHostState,
                       mc_bus_devid_start, 0),
    DEFINE_PROP_UINT32("mc_bus_devid_num", FslMcHostState, mc_bus_devid_num, 0),
    DEFINE_PROP_END_OF_LIST(),
};

MSIMessage fslmc_get_msi_message(FslMcDeviceState *mcdev, uint8_t index)
{
    MSIMessage msg;

    msg.address = mcdev->irqs[index].msi_msg.address;
    msg.data = mcdev->irqs[index].msi_msg.data;
    return msg;
}

void fslmc_set_msi_message(FslMcDeviceState *mcdev, MSIMessage msg,
                           uint8_t index)
{
    mcdev->irqs[index].msi_msg.address = msg.address;
    mcdev->irqs[index].msi_msg.data = msg.data;
}

uint32_t fsl_mc_get_device_id(FslMcDeviceState *mcdev)
{
    return mcdev->device_id;
}

uint64_t fslmc_get_region_addr(FslMcDeviceState *mcdev, uint8_t index)
{
    return mcdev->regions[index].offset;
}

uint32_t fslmc_get_region_size(FslMcDeviceState *mcdev, uint8_t index)
{
    return mcdev->regions[index].size;
}

/* Linear allocation QBMan-portal regions */
static int fsl_mc_get_qbportal_offset(FslMcHostState *host, off_t *offset,
                                      int region_index)
{
    /* FIXME: Current implementation assumes
     * First region of QBman poratl is Cacheable and non-shareable (CE_NS)
     * Second region of QBman poratl is Cache-Inhibited (CI)
     * Third region of QBman poratl is Cacheable and shareable (CE_SH)
     * Remove assumption that first region is CE */
    if (region_index == 0) {
        if (qbman_ce_offset >= FSLMC_QBMAN_PORTALS_CE_SIZE) {
            return -ENOMEM;
        }

        *offset = host->qbman_portals_ce_offset + qbman_ce_offset;
        qbman_ce_offset += FSLMC_QBMAN_REGION_SIZE;
    } else if (region_index == 1) {
        if (qbman_ci_offset >= FSLMC_QBMAN_PORTALS_CI_SIZE) {
            return -ENOMEM;
        }

        *offset = host->qbman_portals_ci_offset + qbman_ci_offset;
        qbman_ci_offset += FSLMC_QBMAN_REGION_SIZE;
    } else {
        if (qbman_cesh_offset >= FSLMC_QBMAN_PORTALS_CESH_SIZE) {
            return -ENOMEM;
        }

        *offset = host->qbman_portals_cesh_offset + qbman_cesh_offset;
        qbman_cesh_offset += FSLMC_QBMAN_REGION_SIZE;
    }
    return 0;
}

/* Linear allocation mc-portal regions */
static int fsl_mc_get_mcportal_offset(FslMcHostState *host, off_t *offset)
{
    if (mcportal_offset >= host->mc_portals_size) {
        return -ENOMEM;
    }

    *offset = host->mc_portals_offset + mcportal_offset;
    mcportal_offset += FSLMC_MCPORTAL_SIZE;
    return 0;
}

int fsl_mc_get_portal_ranges(hwaddr *mc_p_addr, hwaddr *mc_p_size,
                              hwaddr *qbman_p_addr, hwaddr *qbman_p_size)
{
    DeviceState *dev;
    FslMcHostState *host;

    dev = qdev_find_recursive(sysbus_get_default(), TYPE_FSL_MC_HOST);
    host = FSL_MC_HOST(dev);
    if (host == NULL) {
        fprintf(stderr, "No FSL-MC Host bridge found\n");
        return -ENODEV;
    }

    *mc_p_addr = host->mc_bus_base_addr +  host->mc_portals_offset;
    *mc_p_size = host->mc_portals_size;
    *qbman_p_addr = host->mc_bus_base_addr + host->qbman_portals_offset;
    *qbman_p_size = host->qbman_portals_size;
    return 0;
}

static FslMcDeviceState *find_root_dprc_device(FslMcBusState *bus)
{
    FslMcDeviceState *mcdev = NULL;

    QLIST_FOREACH(mcdev, &bus->device_list, next) {
        if (mcdev->parent_mcdev == NULL) {
            return mcdev;
        }
    }
    return NULL;
}

int fsl_mc_get_root_mc_portal_region(hwaddr *mc_p_addr, hwaddr *mc_p_size)
{
    DeviceState *dev;
    FslMcHostState *host;
    FslMcDeviceState *mcdev = NULL;
    hwaddr addr;

    dev = qdev_find_recursive(sysbus_get_default(), TYPE_FSL_MC_HOST);
    host = FSL_MC_HOST(dev);
    if (host == NULL) {
        fprintf(stderr, "No FSL-MC Host bridge found\n");
        return -ENODEV;
    }

    mcdev = find_root_dprc_device(&host->bus);
    if (mcdev == NULL) {
        return -1;
    }

    /* Get to the Base of MC-Portal */
    addr = host->mc_bus_base_addr + host->mc_portals_offset;
    /* Add the Mc-portal device offset */
    addr += mcdev->regions[0].offset;
    *mc_p_addr = addr;
    *mc_p_size = mcdev->regions[0].size;
    return 0;
}

int fsl_mc_register_device_region(FslMcDeviceState *mcdev, int region_num,
                                  MemoryRegion *mem, char *device_type)
{
    FslMcBusState *bus;
    FslMcHostState *host;
    off_t offset;
    int ret;

    bus = mcdev->bus;
    if (bus == NULL) {
        fprintf(stderr, "FSL-MC Bus not found\n");
        return -ENODEV;
    }

    host = FSL_MC_HOST(bus->qbus.parent);
    if (host == NULL) {
        fprintf(stderr, "No FSL-MC Host bridge found\n");
        return -ENODEV;
    }

    if ((strncmp(device_type, "dprc", 4) == 0) ||
        (strncmp(device_type, "dpmcp", 5) == 0)) {
        /* DPRC and DPMCP have only one region */
        if (region_num >= 1) {
            return -EINVAL;
        }

        ret = fsl_mc_get_mcportal_offset(host, &offset);
        if (ret) {
            return ret;
        }

        mcdev->regions[region_num].offset = offset;
        /* FIXME: H/w exposes 0x40 bytes of 0x10000 mc-portal region.
         * While region beyond 0x40 (0x40 - 0x10000) is reserved and
         * not exposed.
         * Should we expose target page size aligned or only 0x40.
         */
        mcdev->regions[region_num].size = FSLMC_MCPORTAL_SIZE;
        memory_region_add_subregion(&host->mc_portal, offset, mem);
    } else if (strncmp(device_type, "dpio", 10) == 0) {
        /* QBman have only three regions (CE_NS, CI and CE_SH) */
        if (region_num >= 3) {
            return -EINVAL;
        }

        ret = fsl_mc_get_qbportal_offset(host, &offset, region_num);
        if (ret) {
            return ret;
        }

        mcdev->regions[region_num].offset = offset;
        mcdev->regions[region_num].size = FSLMC_QBMAN_REGION_SIZE;
        memory_region_add_subregion(&host->qbman_portal, offset, mem);
    } else {
        fprintf(stderr, "%s: Error No Matching device(%s) found\n",
                __func__, device_type);
        return -EINVAL;
    }
    return 0;
}

int fsl_mc_register_device(FslMcDeviceState *mcdev, FslMcDeviceState *pmcdev,
                           char *device_type)
{
    FslMcBusState *bus;
    FslMcDeviceState *ds;
    FslMcHostState *host;

    bus = mcdev->bus;
    if (bus == NULL) {
        fprintf(stderr, "No FSL-MC Bus found\n");
        return -ENODEV;
    }

    /* Check if device already registered */
    QLIST_FOREACH(ds, &bus->device_list, next) {
        if (ds == mcdev) {
            return -EEXIST;
        }
    }

    host = FSL_MC_HOST(bus->qbus.parent);
    if (host == NULL) {
        fprintf(stderr, "No FSL-MC Host bridge found\n");
        return -ENODEV;
    }

    mcdev->parent_mcdev = pmcdev;

    /* All devices in a dprc container share same device-id.
     * So device-id is allocated for a dprc container and same
     * set for all devices in that container.
     */
    if (strncmp(device_type, "dprc", 4) == 0) {
        if (device_id >= (host->mc_bus_devid_start + host->mc_bus_devid_num)) {
            printf("%s: out of device-id\n", __func__);
            return -ENODEV;
        }
        mcdev->device_id = host->mc_bus_devid_start + device_id;
        device_id++;
    } else {
        if (mcdev->parent_mcdev == NULL) {
            return -ENODEV;
        }
        mcdev->device_id = mcdev->parent_mcdev->device_id;
    }

    QLIST_INSERT_HEAD(&bus->device_list, mcdev, next);
    return 0;
}

static void fsl_mc_dev_unrealize(DeviceState *qdev, Error **errp)
{
    FslMcDeviceState *mcdev = (FslMcDeviceState *)qdev;
    FslMcDeviceClass *mcdc = FSL_MC_DEVICE_GET_CLASS(mcdev);

    if (mcdc->exit) {
        mcdc->exit(mcdev);
    }

    mcdev->bus = NULL;
}

static void fsl_mc_dev_realize(DeviceState *qdev, Error **errp)
{
    FslMcDeviceState *mcdev = (FslMcDeviceState *)qdev;
    FslMcDeviceClass *mcdc = FSL_MC_DEVICE_GET_CLASS(mcdev);
    FslMcBusState *bus;
    Error *local_err = NULL;

    bus = FSL_MC_BUS(qdev_get_parent_bus(qdev));
    mcdev->bus = bus;

    if (mcdc->realize) {
        mcdc->realize(mcdev, &local_err);
        if (local_err) {
            return;
        }
    }
}

static void fsl_mc_default_realize(FslMcDeviceState *mcdev, Error **errp)
{
    FslMcDeviceClass *mcdc = FSL_MC_DEVICE_GET_CLASS(mcdev);

    if (mcdc->init) {
        if (mcdc->init(mcdev) < 0) {
            error_setg(errp, "Device initialization failed");
            return;
        }
    }
}

static void fsl_mc_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    FslMcDeviceClass *mcdc = FSL_MC_DEVICE_CLASS(klass);

    dc->bus_type = TYPE_FSL_MC_BUS;
    dc->realize = fsl_mc_dev_realize;
    dc->unrealize = fsl_mc_dev_unrealize;
    mcdc->realize = fsl_mc_default_realize;
}

static const TypeInfo fsl_mc_device_info = {
    .name          = TYPE_FSL_MC_DEVICE,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(FslMcDeviceState),
    .class_size = sizeof(FslMcDeviceClass),
    .class_init    = fsl_mc_device_class_init,
};

static void fsl_mc_host_initfn(Object *obj)
{
    FslMcHostState *s = FSL_MC_HOST(obj);
    DeviceState *ds = DEVICE(obj);

    qbus_create_inplace(&s->bus, sizeof(s->bus), TYPE_FSL_MC_BUS, ds, NULL);
    QLIST_INIT(&s->bus.device_list);
}

static void fsl_mc_host_realize(DeviceState *dev, Error **errp)
{
    FslMcHostState *s = FSL_MC_HOST(dev);
    SysBusDevice *d = SYS_BUS_DEVICE(dev);

    if (s == NULL) {
        fprintf(stderr, "No FSL-MC Host bridge found\n");
        return;
    }

    /*
     * QBMan have 1024 h/w portals and each QBMan h/w portal have three
     * address region:
     *  1) Cache Inhibited (CI) region of size 0x10000
     *  2) Cache Enable (CE_NS) region of size 0x10000
     *  3) Cache Enable and Shareable (CE_SH) region of size 0x10000
     *
     * CI regions of all QBMan portals are placed in contiguous address range.
     * Similarly CE_NS and CE_SH regions of all QBMan portals are also placed
     * in respective contiguous address ranges.
     * CI address range starts from offset 0x0 of QBman address-space, CE_NS
     * address range is placed just after CI address space and CE_SH address
     * range is placed just after CE_NS address range.
     *
     * To support 1024 QBMan portals following ranges are defined:
     * CI address range = offset-0x0 to 0x4000000
     * CE_NS address range = offset-0x4000000 to 0x8000000
     * CE_SH address range = offset-0x8000000 to 0xC000000
     */
    s->qbman_portals_ci_offset = 0x0;
    s->qbman_portals_ci_size = FSLMC_QBMAN_PORTALS_CI_SIZE;
    s->qbman_portals_ce_offset = s->qbman_portals_ci_offset +
                                  FSLMC_QBMAN_PORTALS_CI_SIZE;
    s->qbman_portals_ce_size = FSLMC_QBMAN_PORTALS_CE_SIZE;
    s->qbman_portals_cesh_offset = s->qbman_portals_ce_offset +
                                    FSLMC_QBMAN_PORTALS_CE_SIZE;
    s->qbman_portals_cesh_size = FSLMC_QBMAN_PORTALS_CESH_SIZE;

    memory_region_init_io(&s->mc_portal, OBJECT(s), NULL, s,
                          "fsl_mc portal", s->mc_portals_size);
    sysbus_init_mmio(d, &s->mc_portal);

    memory_region_init_io(&s->qbman_portal, OBJECT(s), NULL, s,
                          "fsl_qbman portal", s->qbman_portals_size);
    sysbus_init_mmio(d, &s->qbman_portal);
}

static void fsl_mc_host_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->props = fsl_mc_props;
    dc->realize = fsl_mc_host_realize;
}

static const TypeInfo fsl_mc_host_info = {
    .name          = TYPE_FSL_MC_HOST,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(FslMcHostState),
    .instance_init = fsl_mc_host_initfn,
    .class_init    = fsl_mc_host_class_init,
};

static const TypeInfo fsl_mc_bus_info = {
    .name = TYPE_FSL_MC_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(FslMcBusState),
};

static void fsl_mc_register_types(void)
{
    type_register_static(&fsl_mc_bus_info);
    type_register_static(&fsl_mc_host_info);
    type_register_static(&fsl_mc_device_info);
}

type_init(fsl_mc_register_types)
