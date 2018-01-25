/*
 * vfio based device assignment support -Freescale
 * Management Complex (FSL_MC) devices
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

#include "qemu/osdep.h"
#include "qapi/error.h"
#include <sys/ioctl.h>
#include <linux/vfio.h>
#include <sys/types.h>
#include <dirent.h>
#include <strings.h>

#include "hw/fsl-mc/fsl-mc.h"
#include "hw/vfio/vfio-fsl-mc.h"
#include "qemu/error-report.h"
#include "qemu/range.h"
#include "sysemu/sysemu.h"
#include "exec/memory.h"
#include "qemu/queue.h"
#include "sysemu/kvm.h"

static QLIST_HEAD(, VFIOFslmcDevice) root_dprc_list =
    QLIST_HEAD_INITIALIZER(root_dprc_list);

static void vfio_fsl_mc_region_write(void *opaque, hwaddr addr,
                              uint64_t data, unsigned size)
{
}

static uint64_t vfio_fsl_mc_region_read(void *opaque,
                                 hwaddr addr, unsigned size)
{
    return 0;
}

const MemoryRegionOps vfio_region_fsl_mc_ops = {
    .read = vfio_fsl_mc_region_read,
    .write = vfio_fsl_mc_region_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/**
 * vfio_populate_device - Allocate and populate MMIO region
 * and IRQ structs according to driver returned information
 * @vbasedev: the VFIO device handle
 *
 */
static void vfio_populate_device(VFIODevice *vbasedev, Error **errp)
{
    int i, ret = -1;
    VFIOFslmcDevice *vdev =
        container_of(vbasedev, VFIOFslmcDevice, vbasedev);

    if (!(vbasedev->flags & VFIO_DEVICE_FLAGS_FSL_MC)) {
        error_setg(errp, "vfio-fsl-mc: this isn't a fsl_mc device");
        return;
    }

    vdev->regions = g_new0(VFIORegion *, vbasedev->num_regions);

    for (i = 0; i < vbasedev->num_regions; i++) {
        char *name = g_strdup_printf("VFIO %s region %d\n", vbasedev->name, i);

        vdev->regions[i] = g_new0(VFIORegion, 1);
        ret = vfio_region_setup(OBJECT(vdev), vbasedev,
                                vdev->regions[i], &vfio_region_fsl_mc_ops,
                                i, name);
        g_free(name);
        if (ret) {
            error_setg_errno(errp, -ret, "Error getting region %d info", i);
            goto reg_error;
        }
    }

    vdev->irqs = g_new0(struct VFIOFslMcIrqs *, vbasedev->num_irqs);
    for (i = 0; i < vbasedev->num_irqs; i++) {
        struct vfio_irq_info irq = { .argsz = sizeof(irq) };

        irq.index = i;
        ret = ioctl(vbasedev->fd, VFIO_DEVICE_GET_IRQ_INFO, &irq);
        if (ret) {
            error_setg_errno(errp, -ret, "failed to get device irq info");
            goto irq_err;
        }
        vdev->irqs[i] = g_new0(struct VFIOFslMcIrqs, 1);
        vdev->irqs[i]->irq_index = irq.index;
    }

    return;
irq_err:
    for (i = 0; i < vbasedev->num_irqs; i++) {
       g_free(vdev->irqs[i]);
    }
    g_free(vdev->irqs);

reg_error:
    for (i = 0; i < vbasedev->num_regions; i++) {
        if (vdev->regions[i]) {
            vfio_region_finalize(vdev->regions[i]);
        }
        g_free(vdev->regions[i]);
    }
    g_free(vdev->regions);
    return;
}

/* TODO: Reset functionality to be addded */
static void vfio_fsl_mc_compute_needs_reset(VFIODevice *vbasedev)
{
    vbasedev->needs_reset = false;
}

static int vfio_fsl_mc_hot_reset_multi(VFIODevice *vbasedev)
{
    return 0;
}

static void vfio_fsl_mc_eoi(VFIODevice *vbasedev)
{
}

/* Specialized functions for VFIO FSL-MC devices */
static VFIODeviceOps vfio_fsl_mc_ops = {
    .vfio_compute_needs_reset = vfio_fsl_mc_compute_needs_reset,
    .vfio_hot_reset_multi = vfio_fsl_mc_hot_reset_multi,
    .vfio_eoi = vfio_fsl_mc_eoi,
};

/**
 * vfio_base_device_init - perform preliminary VFIO setup
 * @vbasedev: the VFIO device handle
 * @errp: error object
 *
 * Implement the VFIO command sequence that allows to discover
 * assigned device resources: group extraction, device
 * fd retrieval, resource query.
 * Precondition: the device name must be initialized
 */
static int vfio_base_device_init(VFIODevice *vbasedev, Error **errp)
{
    VFIOGroup *group;
    VFIODevice *vbasedev_iter;
    char *tmp, group_path[PATH_MAX], *group_name;
    Error *err = NULL;
    ssize_t len;
    struct stat st;
    int groupid;
    int ret;

    /* @sysfsdev takes precedence over @host */
    if (vbasedev->sysfsdev) {
        g_free(vbasedev->name);
        vbasedev->name = g_strdup(basename(vbasedev->sysfsdev));
    } else {
        if (!vbasedev->name || strchr(vbasedev->name, '/')) {
            return -EINVAL;
        }

        vbasedev->sysfsdev = g_strdup_printf("/sys/bus/fsl-mc/devices/%s",
                                             vbasedev->name);
    }

    if (stat(vbasedev->sysfsdev, &st) < 0) {
        error_report("vfio: error: no such host device: %s",
                     vbasedev->sysfsdev);
        return -errno;
    }

    tmp = g_strdup_printf("%s/iommu_group", vbasedev->sysfsdev);
    len = readlink(tmp, group_path, sizeof(group_path));
    g_free(tmp);

    if (len < 0 || len >= sizeof(group_path)) {
        error_report("vfio: error no iommu_group for device");
        return len < 0 ? -errno : -ENAMETOOLONG;
    }

    group_path[len] = 0;
    group_name = basename(group_path);
    if (sscanf(group_name, "%d", &groupid) != 1) {
        error_report("vfio: error reading %s: %m", group_path);
        return -errno;
    }

    group = vfio_get_group(groupid, &address_space_memory, errp);
    if (!group) {
        error_report("vfio: failed to get group %d", groupid);
        return -ENOENT;
    }

    QLIST_FOREACH(vbasedev_iter, &group->device_list, next) {
        if (strcmp(vbasedev_iter->name, vbasedev->name) == 0) {
            error_report("vfio: error: device %s is already attached",
                         vbasedev->name);
            vfio_put_group(group);
            return -EBUSY;
        }
    }
    ret = vfio_get_device(group, vbasedev->name, vbasedev, errp);
    if (ret) {
        error_report("vfio: failed to get device %s", vbasedev->name);
        vfio_put_group(group);
        return ret;
    }

    vfio_populate_device(vbasedev, &err);
    if (err) {
        error_report("vfio: failed to populate device %s", vbasedev->name);
        vfio_put_group(group);
    }

    return ret;
}

static void vfio_fsl_mc_realize(FslMcDeviceState *mcdev, Error **errp)
{
    VFIOFslmcDevice *vdev = DO_UPCAST(VFIOFslmcDevice, mcdev, mcdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    VFIOFslmcDevice *parent_vdev = (VFIOFslmcDevice *)vdev->parent_vdev;
    FslMcDeviceState *pmcdev = NULL;
    int ret;
    char *temp;

    if (!(strncmp(vbasedev->name, "dprc", 4))) {
        /* "dprc" which does not have parent device is root-dprc */
        if (parent_vdev == NULL) {
            QLIST_INSERT_HEAD(&root_dprc_list, vdev, root_dprc_next);
            QLIST_INIT(&vdev->device_list);
        } else {
            /* Add to parent DPRC device list */
            pmcdev = &parent_vdev->mcdev;
            QLIST_INSERT_HEAD(&parent_vdev->device_list, vdev, next);
        }
    }

    vbasedev->type = VFIO_DEVICE_TYPE_FSL_MC;
    vbasedev->ops = &vfio_fsl_mc_ops;

    ret = vfio_base_device_init(vbasedev, errp);
    if (ret) {
        error_setg(errp, "Failed to initialize device");
        return;
    }

    strncpy(vdev->device_type, vbasedev->name, 10);
    temp = strchr(vdev->device_type, '.');
    *temp = '\0';
    temp++;
    vdev->id = atoi(temp);

    ret = fsl_mc_register_device(mcdev, pmcdev, vdev->device_type);
    if (ret) {
        error_setg(errp, "Failed to register device");
        return;
    }
    return;
}

static void vfio_fsl_mc_instance_init(Object *obj)
{
    FslMcDeviceState *mcdev = FSL_MC_DEVICE(obj);
    VFIOFslmcDevice *vdev = DO_UPCAST(VFIOFslmcDevice, mcdev, mcdev);

    device_add_bootindex_property(obj, &vdev->bootindex,
                                  "bootindex", NULL,
                                  &mcdev->qdev, NULL);
}

static const VMStateDescription vfio_fsl_mc_vmstate = {
    .name = TYPE_VFIO_FSL_MC,
    .unmigratable = 1,
};

static Property vfio_fsl_mc_dev_properties[] = {
    DEFINE_PROP_STRING("host", VFIOFslmcDevice, vbasedev.name),
    DEFINE_PROP_STRING("sysfsdev", VFIOFslmcDevice, vbasedev.sysfsdev),
    DEFINE_PROP_BOOL("x-no-mmap", VFIOFslmcDevice, vbasedev.no_mmap, false),
    DEFINE_PROP_PTR("parent_vdev", VFIOFslmcDevice, parent_vdev),
    DEFINE_PROP_END_OF_LIST(),
};

static void vfio_fsl_mc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    FslMcDeviceClass *mcdc = FSL_MC_DEVICE_CLASS(klass);

    dc->props = vfio_fsl_mc_dev_properties;
    dc->vmsd = &vfio_fsl_mc_vmstate;
    dc->desc = "VFIO-based fsl_mc device assignment";
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    mcdc->realize = vfio_fsl_mc_realize;
}

static const TypeInfo vfio_fsl_mc_dev_info = {
    .name = TYPE_VFIO_FSL_MC,
    .parent = TYPE_FSL_MC_DEVICE,
    .instance_size = sizeof(VFIOFslmcDevice),
    .class_init = vfio_fsl_mc_class_init,
    .instance_init = vfio_fsl_mc_instance_init,
};

static void register_vfio_fsl_mc_dev_type(void)
{
    type_register_static(&vfio_fsl_mc_dev_info);
}

type_init(register_vfio_fsl_mc_dev_type)
