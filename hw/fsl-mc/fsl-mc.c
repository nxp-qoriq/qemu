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
     * QBMan have many h/w portals and each QBMan h/w portal
     * have two regions
     *  1) Cache Inhibited (CI) Region of size 0x10000
     *  2) Cache Enable (CE) Region of size 0x10000
     * CI regions of all QBMan portals are placed in
     * one contiguous range. Similarly CE regions of all
     * QBMan portals are also placed in one contiguous range
     *
     * To support 1024 QBMan portals following range
     * is defined:
     * CI regions size = 0x10000 * 1024 = 0x4000000
     * CE regions size = 0x10000 * 1024 = 0x4000000
     *
     * CI regions are at offset 0x0 and CE regions are placed
     * just after CI regions (offset 0x0x4000000).
     */
    s->qbman_portals_ci_offset = 0x0;
    s->qbman_portals_ci_size = FSLMC_QBMAN_PORTALS_CI_SIZE;
    s->qbman_portals_ce_offset = s->qbman_portals_ci_offset +
                                  FSLMC_QBMAN_PORTALS_CI_SIZE;
    s->qbman_portals_ce_size = FSLMC_QBMAN_PORTALS_CE_SIZE;

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
