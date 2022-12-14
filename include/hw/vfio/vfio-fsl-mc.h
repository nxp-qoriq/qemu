/*
 * vfio based device assignment support -Freescale Management Complex devices
 *
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
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

typedef struct VFIOFSLMCMSIVector {
    /*
     * Two interrupt paths are configured per vector.  The first, is only used
     * for interrupts injected via QEMU.  This is typically the non-accel path,
     * but may also be used when we want QEMU to handle masking and pending
     * bits.  The KVM path bypasses QEMU and is therefore higher performance,
     * but requires masking at the device.  virq is used to track the MSI route
     * through KVM, thus kvm_interrupt is only available when virq is set to a
     * valid (>= 0) value.
     */
    EventNotifier interrupt;
    EventNotifier kvm_interrupt;
    struct VFIOFslmcDevice *vdev; /* back pointer to device */
    int virq;
    bool use;
} VFIOFSLMCMSIVector;

struct VFIOFslMcIrqs {
    uint8_t irq_index;
    VFIOFSLMCMSIVector *msi_vector;
};

struct dprc_open_cmd {
    uint32_t container_id;
};

struct dprc_get_attributes_response {
    uint32_t container_id;
    uint32_t icid;
    uint32_t options;
    uint32_t portal_id;
};

struct get_obj_region_cmd {
    uint32_t obj_id;
    uint16_t reserved1;
    uint8_t region_index;
    uint8_t reserved2;
    uint64_t reserved3[2];
    char obj_type[16];
};

struct get_obj_region_resp {
    uint64_t reserved1;
    uint64_t base_addr;
    uint32_t size;
/* Region Type */
#define DPRC_REGION_TYPE_MC_PORTAL 0
#define DPRC_REGION_TYPE_QBMAN_PORTAL 1
    uint8_t type;
    uint8_t reserved2;
    uint16_t reserved3;
#define DPRC_REGION_FLAG_CACHE_INHIBIT 0
#define DPRC_REGION_FLAG_CACHEABLE 1
#define DPRC_REGION_FLAG_SHAREABLE 2
    uint32_t flags;
};

struct get_obj_region_resp_v2 {
    /* Response word 0 */
    uint64_t reserved1;
    /* Response word 1 */
    uint32_t base_offset;
    uint32_t reserved2;
    /* Response word 2 */
    uint32_t size;
/* Region Type */
#define DPRC_REGION_TYPE_MC_PORTAL 0
#define DPRC_REGION_TYPE_QBMAN_PORTAL 1
    uint8_t type;
    uint8_t reserved3[3];
    /* Response word 2 */
#define DPRC_REGION_FLAG_CACHE_INHIBIT 0
#define DPRC_REGION_FLAG_CACHEABLE 1
    uint32_t flags;
    uint32_t reserved4;
    /* Response word 2 */
    uint64_t base_addr;
};

struct dprc_set_irq_cmd {
    uint32_t irq_val;
    uint8_t irq_index;
    uint8_t reserved1;
    uint16_t reserved2;
    uint64_t irq_addr;
    uint32_t irq_num;
};

struct dprc_get_irq_cmd {
    uint32_t reserved;
    uint8_t irq_index;
};

struct dprc_get_irq_resp {
    uint32_t irq_val;
    uint32_t reserved;
    uint64_t irq_addr;
    uint32_t irq_num;
    uint32_t irq_type;
};

struct dprc_set_obj_irq_cmd {
    uint32_t irq_val;
    uint8_t irq_index;
    uint8_t reserved1;
    uint16_t reserved2;
    uint64_t irq_addr;
    uint32_t irq_num;
    uint32_t obj_id;
    char obj_type[16];
};

struct dprc_get_obj_irq_cmd {
    uint32_t obj_id;
    uint8_t irq_index;
    uint8_t reserved1;
    uint16_t reserved2;
    char obj_type[16];
};

struct dprc_get_obj_irq_resp {
    uint32_t irq_val;
    uint32_t reserved;
    uint64_t irq_addr;
    uint32_t irq_num;
    uint32_t irq_type;
};

typedef union {
    uint64_t portal[8];
    struct {
        uint64_t header;
        uint64_t data[7];
    } p;
} MCPortal;

typedef struct VFIOFslmcDevice {
    FslMcDeviceState mcdev;
    VFIODevice vbasedev; /* not a QOM object */
    VFIORegion **regions;
    struct VFIOFslMcIrqs **irqs;
    void *parent_vdev;
    MCPortal mcportal;
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
