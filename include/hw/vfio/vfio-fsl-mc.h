/*
 * vfio based device assignment support -Freescale Management Complex devices
 *
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * Author: Bharat Bhushan <bharat.bhushan@nxp.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#ifndef HW_VFIO_VFIO_FSL_MC_H
#define HW_VFIO_VFIO_FSL_MC_H

#include "hw/sysbus.h"
#include "hw/vfio/vfio-common.h"

#define TYPE_VFIO_FSL_MC "vfio-fsl-mc"

struct VFIOFslMcIrqs {
    uint8_t irq_index;
};

typedef struct VFIOFslmcDevice {
    FslMcDeviceState mcdev;
    VFIODevice vbasedev; /* not a QOM object */
    VFIORegion **regions;
    struct VFIOFslMcIrqs **irqs;
    void *parent_vdev;
    char device_type[10];
    uint16_t id;
    int32_t bootindex;
    QLIST_ENTRY(VFIOFslmcDevice) root_dprc_next;
    QLIST_HEAD(, VFIOFslmcDevice) device_list;
    QLIST_ENTRY(VFIOFslmcDevice) next;
} VFIOFslmcDevice;

typedef struct VFIOFslmcDeviceClass {
    /*< private >*/
    SysBusDeviceClass parent_class;
    /*< public >*/
} VFIOFslmcDeviceClass;

#define VFIO_FSL_MC_DEVICE(obj) \
     OBJECT_CHECK(VFIOFslmcDevice, (obj), TYPE_VFIO_FSL_MC)
#define VFIO_FSL_MC_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(VFIOFslmcDeviceClass, (klass), TYPE_VFIO_FSL_MC)
#define VFIO_FSL_MC_DEVICE_GET_CLASS(obj) \
     OBJECT_GET_CLASS(VFIOFslmcDeviceClass, (obj), TYPE_VFIO_FSL_MC)

#endif /*HW_VFIO_VFIO_FSL_MC_H*/