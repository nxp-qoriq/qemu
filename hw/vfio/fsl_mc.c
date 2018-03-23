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

enum mc_cmd_status {
    MC_CMD_STATUS_OK = 0x0, /* Completed successfully */
    MC_CMD_STATUS_READY = 0x1, /* Ready to be processed */
    MC_CMD_STATUS_AUTH_ERR = 0x3, /* Authentication error */
    MC_CMD_STATUS_NO_PRIVILEGE = 0x4, /* No privilege */
    MC_CMD_STATUS_DMA_ERR = 0x5, /* DMA or I/O error */
    MC_CMD_STATUS_CONFIG_ERR = 0x6, /* Configuration error */
    MC_CMD_STATUS_TIMEOUT = 0x7, /* Operation timed out */
    MC_CMD_STATUS_NO_RESOURCE = 0x8, /* No resources */
    MC_CMD_STATUS_NO_MEMORY = 0x9, /* No memory available */
    MC_CMD_STATUS_BUSY = 0xA, /* Device is busy */
    MC_CMD_STATUS_UNSUPPORTED_OP = 0xB, /* Unsupported operation */
    MC_CMD_STATUS_INVALID_STATE = 0xC /* Invalid state */
};

/* command header masks and shifts */
#define CMDHDR_SRCID_MASK               0x00000000000000FF
#define CMDHDR_PRIORITY_MASK            0x0000000000008000
#define CMDHDR_STATUS_MASK              0x0000000000FF0000
#define CMDHDR_INTR_DIS_MASK            0x0000000001000000
#define CMDHDR_AUTH_ID_MASK             0x0000FFFF00000000
#define CMDHDR_CMD_CODE_MASK            0xFFF0000000000000
#define CMDHDR_CMD_CODE_VERSION_MASK    0x000F000000000000

#define CMDHDR_CMD_ID_O                 48 /* Command ID (code + version) field offset */
#define CMDHDR_CMD_ID_S                 16 /* Command ID (code + version) field size */
#define CMDHDR_CMD_CODE_O               52 /* Command code field offset */
#define CMDHDR_CMD_CODE_S               12 /* Command code field size */
#define CMDHDR_CMD_VER_O                48 /* Command ver field offset */
#define CMDHDR_CMD_VER_S                4  /* Command ver field size */
#define CMDHDR_AUTH_ID_O                32 /* Authentication ID field offset */
#define CMDHDR_AUTH_ID_S                16 /* Authentication ID field size */
#define CMDHDR_INTR_DIS_O               24 /* Interrupt disable filed offset*/
#define CMDHDR_INTR_DIS_S               1  /* Interrupt disable filed size*/
#define CMDHDR_STATUS_O                 16 /* Status field offset */
#define CMDHDR_STATUS_S                 8  /* Status field size*/
#define CMDHDR_PRI_O                    15 /* Priority field offset */
#define CMDHDR_PRI_S                    1  /* Priority field size */
#define CMDHDR_SRCID_O                  0  /* Src id field offset */
#define CMDHDR_SRCID_S                  8  /* Src id field size */

#define CMDHDR_PRIORITY_SHIFT           CMDHDR_PRI_O
#define CMDHDR_STATUS_SHIFT             CMDHDR_STATUS_O
#define CMDHDR_INTR_DIS_SHIFT           CMDHDR_INTR_DIS_O
#define CMDHDR_AUTH_ID_SHIFT            CMDHDR_AUTH_ID_O
#define CMDHDR_CMD_CODE_SHIFT           CMDHDR_CMD_CODE_O
#define CMDHDR_CMD_VERSION_SHIFT        CMDHDR_CMD_VER_O

/* MC command versioning
 * 0 : command version with mc f/w below 10.0.0
 * 1 : command version with mc f/w above (including) 10.0.0
 */
#define MC_CMD_HDR_NO_VER               0x0
#define MC_CMD_HDR_BASE_VER             0x1

/* DPRC Commands */
#define DPRC_CMD_CODE_OPEN             0x805
#define DPRC_CMD_CODE_CLOSE            0x800
#define DPRC_CMD_CODE_GET_ATTR         0x004
#define DPRC_CMD_CODE_GET_OBJ_REG      0x15E
#define DPRC_CMD_CODE_SET_IRQ          0x010
#define DPRC_CMD_CODE_GET_IRQ          0x011
#define DPRC_CMD_CODE_SET_OBJ_IRQ      0x15F

enum mcportal_state {
    MCPORTAL_CLOSE,
    MCPORTAL_OPEN
};

enum fslmc_cmdif_type {
    CMDIF_OTHER = 0,
    CMDIF_DPRC = 1,
    CMDIF_DPMCP = 2
};

typedef struct VFIOFSLMC_cmdif {
    struct VFIOFslmcDevice *mcportal_vdev;
    enum mcportal_state state;
    uint32_t portal_id;
    enum fslmc_cmdif_type cmdif_type;
    uint16_t token;
    /* DPRC or DPMCP device */
    QLIST_ENTRY(VFIOFSLMC_cmdif) next;
} VFIOFSLMC_cmdif;

static QLIST_HEAD(, VFIOFSLMC_cmdif) mc_cmdif_list =
    QLIST_HEAD_INITIALIZER(mc_cmdif_list);

static inline void fslmc_set_cmd_status(uint64_t *header,
                                        enum mc_cmd_status status)
{
    *header &= ~CMDHDR_STATUS_MASK;
    *header |= status << CMDHDR_STATUS_SHIFT;
}

static inline enum mc_cmd_status fslmc_get_cmd_status(uint64_t header)
{
    return ((header & CMDHDR_STATUS_MASK) >> CMDHDR_STATUS_SHIFT);
}

/* Return mc command */
static inline int fslmc_get_cmd(uint64_t header)
{
    return (int ) ((header & CMDHDR_CMD_CODE_MASK) >> CMDHDR_CMD_CODE_SHIFT);
}

static inline int fslmc_get_command_version(uint64_t header)
{
    return (int ) ((header & CMDHDR_CMD_CODE_VERSION_MASK) >>
                   CMDHDR_CMD_VERSION_SHIFT);
}

static inline uint16_t fslmc_get_cmd_token(uint64_t header)
{
    return (uint16_t )((header & CMDHDR_AUTH_ID_MASK) >> CMDHDR_AUTH_ID_SHIFT);
}

/* MC portals can only support one outstanding command at a time,
 * So an MC command must be completed before accepting next command.
 * TODO: Optimize for actual size of command/response
 */
static inline void vfio_fsl_mc_portal_send_cmd(VFIORegion *region,
                                               MCPortal *mcp)
{
    VFIODevice *vbasedev = region->vbasedev;
    uint64_t *p = &mcp->portal[0];
    int fd = vbasedev->fd;

    /* Send complete command to VFIO kernel driver */
    if (pwrite(fd, p, sizeof(*mcp), region->fd_offset) != sizeof(*mcp)) {
        error_report("%s(%s:pwrite: region%d+0x%x, 0x%"PRIx64 ",%lx) failed: %m",
                     __func__, vbasedev->name, region->nr, 0, *p, sizeof(*mcp));
    }

    /* Read Response from VFIO kernel driver */
    if (pread(fd, p, sizeof(*mcp), region->fd_offset) != sizeof(*mcp)) {
        error_report("%s(%s:pread: region%d+0x%x, 0x%"PRIx64 ",%lx) failed: %m",
                     __func__, vbasedev->name, region->nr, 0, *p, sizeof(*mcp));
    }
}

static VFIOFslmcDevice *get_mcportal_in_dprc(VFIOFslmcDevice *vdev, int portal_id)
{
    VFIOFslmcDevice *mcportal_vdev;

    /* This must be dprc device */
    if (strcmp(vdev->device_type, "dprc")) {
        return NULL;
    }

    /* DPRC device itself */
    if (vdev->id == portal_id) {
        return vdev;
    }

    /* One of child mcportal device in the container */
    QLIST_FOREACH(mcportal_vdev, &vdev->device_list, next) {
        if (((strncmp(mcportal_vdev->device_type, "dprc", 4) == 0) ||
            (strncmp(mcportal_vdev->device_type, "dpmcp", 5) == 0)) &&
            (mcportal_vdev->id == portal_id)) {
            return mcportal_vdev;
        }
    }
    return NULL;
}

static VFIOFSLMC_cmdif *get_mc_cmdif_from_token(uint16_t token)
{
    VFIOFSLMC_cmdif *mc_cmdif;

    QLIST_FOREACH(mc_cmdif, &mc_cmdif_list, next) {
        if (mc_cmdif->token == token) {
            return mc_cmdif;
        }
    }
    return NULL;
}

static VFIOFslmcDevice *get_vdev_in_dprc(VFIOFslmcDevice *vdev_dprc,
                                          const char *obj_type, int obj_id)
{
    VFIOFslmcDevice *vdev;

    QLIST_FOREACH(vdev, &vdev_dprc->device_list, next) {
        if ((strcmp(vdev->device_type, obj_type) == 0) &&
            (vdev->id == obj_id)) {
            return vdev;
        }
    }
    return NULL;
}

static int fslmc_auth_cmd(uint16_t token)
{
    VFIOFSLMC_cmdif *mc_cmdif;

    QLIST_FOREACH(mc_cmdif, &mc_cmdif_list, next) {
        if (mc_cmdif->token == token) {
            return 0;
        }
    }
    return -1;
}

static void dprc_open(VFIORegion *region, MCPortal *mcp)
{
    VFIODevice *vbasedev = region->vbasedev;
    VFIOFslmcDevice *vdev =
        container_of(vbasedev, VFIOFslmcDevice, vbasedev);
    VFIOFslmcDevice *mcportal_vdev = NULL;
    struct dprc_open_cmd *cmd = (struct dprc_open_cmd *)&mcp->p.data[0];
    uint32_t portal_id = cmd->container_id;
    uint16_t token;
    VFIOFSLMC_cmdif *mc_cmdif;

    vfio_fsl_mc_portal_send_cmd(region, mcp);
    if (fslmc_get_cmd_status(mcp->p.header) != MC_CMD_STATUS_OK) {
        return;
    }

    token = fslmc_get_cmd_token(mcp->p.header);
    QLIST_FOREACH(mc_cmdif, &mc_cmdif_list, next) {
        if (mc_cmdif->token == token) {
            printf("Instance already opened \n");
            goto err;
        }
    }

    /* Get the dprc opened on this mc-portal */
    mcportal_vdev = get_mcportal_in_dprc(vdev, portal_id);
    if (mcportal_vdev == NULL) {
        printf("Container (%d) to be opened does not exists \n", portal_id);
        goto err;
    }

    mc_cmdif = g_new0(VFIOFSLMC_cmdif, 1);
    mc_cmdif->token = token;
    mc_cmdif->mcportal_vdev = mcportal_vdev;
    mc_cmdif->state = MCPORTAL_OPEN;
    mc_cmdif->cmdif_type = CMDIF_DPRC;
    QLIST_INSERT_HEAD(&mc_cmdif_list, mc_cmdif, next);
    return;

err:
    fslmc_set_cmd_status(&mcp->p.header, MC_CMD_STATUS_NO_RESOURCE);
}

static void dprc_close(VFIORegion *region, MCPortal *mcp, uint16_t token)
{
    VFIOFSLMC_cmdif *mc_cmdif;

    vfio_fsl_mc_portal_send_cmd(region, mcp);
    if (fslmc_get_cmd_status(mcp->p.header) != MC_CMD_STATUS_OK) {
        return;
    }

    QLIST_FOREACH(mc_cmdif, &mc_cmdif_list, next) {
        if (mc_cmdif->token == token) {
            break;
        }
    }

    if (mc_cmdif == NULL) {
        printf("No mc_cmdif found\n");
        return;
    }

    mc_cmdif->state = MCPORTAL_CLOSE;
    mc_cmdif->token = 0;
    mc_cmdif->mcportal_vdev = NULL;
    QLIST_REMOVE(mc_cmdif, next);
}

static inline void dprc_set_icid(MCPortal *mcp, uint32_t sid)
{
    struct dprc_get_attributes_response *resp;

    resp = (struct dprc_get_attributes_response *)&mcp->p.data[0];
    resp->icid = sid;
}

static void dprc_handle_get_attr(VFIORegion *region, MCPortal *mcp,
                                 VFIOFSLMC_cmdif *mc_cmdif)
{
    VFIOFslmcDevice *vdev = mc_cmdif->mcportal_vdev;
    uint32_t sid;

    vfio_fsl_mc_portal_send_cmd(region, mcp);
    if (fslmc_get_cmd_status(mcp->p.header) != MC_CMD_STATUS_OK) {
        return;
    }

    /* update stream-id */
    sid = fsl_mc_get_device_id(&vdev->mcdev);
    dprc_set_icid(mcp, sid);
}

static void dprc_get_obj_region(VFIORegion *region, MCPortal *mcp,
                                VFIOFSLMC_cmdif *mc_cmdif)
{
    VFIOFslmcDevice *vdev_dprc = mc_cmdif->mcportal_vdev;
    VFIOFslmcDevice *vdev;
    struct get_obj_region_cmd *cmd =
               (struct get_obj_region_cmd *)&mcp->p.data[0];
    struct get_obj_region_resp *resp =
               (struct get_obj_region_resp *)&mcp->p.data[0];
    uint8_t region_index = cmd->region_index;
    uint8_t region_type;
    enum mc_cmd_status status = MC_CMD_STATUS_OK;

    vdev = get_vdev_in_dprc(vdev_dprc, cmd->obj_type, cmd->obj_id);
    if (vdev == NULL) {
        status = MC_CMD_STATUS_CONFIG_ERR;
        goto out;
    }

    if (strncmp((char *)cmd->obj_type, "dpio", 4)) {
        region_type = DPRC_REGION_TYPE_MC_PORTAL;
    } else {
        region_type = DPRC_REGION_TYPE_QBMAN_PORTAL;
    }

    memset(resp, 0, sizeof(mcp->p.data));
    resp->base_addr = fslmc_get_region_addr(&vdev->mcdev, region_index);
    resp->size = fslmc_get_region_size(&vdev->mcdev, region_index);
    resp->type = region_type;
    resp->flags = DPRC_REGION_FLAG_CACHE_INHIBIT;

out:
    fslmc_set_cmd_status(&mcp->p.header, status);
}

static int vfio_set_kvm_msi_irqfd(VFIOFslmcDevice *vdev, uint8_t irq_index)
{
    /* Add fast-path with KVM IRQFD */
    return -1;
}

static void dprc_set_irq(VFIORegion *region, MCPortal *mcp,
                         VFIOFSLMC_cmdif *mc_cmdif)
{
    VFIOFslmcDevice *vdev = mc_cmdif->mcportal_vdev;
    struct dprc_set_irq_cmd *cmd = (struct dprc_set_irq_cmd *)&mcp->p.data[0];
    uint8_t irq_index;
    MSIMessage msg;
    int ret;

    irq_index = cmd->irq_index;

    msg.data = cmd->irq_val;

    fslmc_set_msi_message(&vdev->mcdev, msg, irq_index);

    ret = vfio_set_kvm_msi_irqfd(vdev, irq_index);
    if (ret) {
        fslmc_set_cmd_status(&mcp->p.header, MC_CMD_STATUS_INVALID_STATE);
        return;
    }
    fslmc_set_cmd_status(&mcp->p.header, MC_CMD_STATUS_OK);
}

static void dprc_get_irq(VFIORegion *region, MCPortal *mcp,
                         VFIOFSLMC_cmdif *mc_cmdif)
{
    VFIOFslmcDevice *vdev = mc_cmdif->mcportal_vdev;
    struct dprc_get_irq_cmd *cmd = (struct dprc_get_irq_cmd *)&mcp->p.data[0];
    struct dprc_get_irq_resp *resp = (struct dprc_get_irq_resp *)&mcp->p.data[0];
    MSIMessage msg;

    msg = fslmc_get_msi_message(&vdev->mcdev, cmd->irq_index);
    resp->irq_val = msg.data;
    resp->irq_addr = msg.address;
    resp->irq_num = 1;
    resp->irq_type = 0;

    fslmc_set_cmd_status(&mcp->p.header, MC_CMD_STATUS_OK);
}

static void dprc_set_obj_irq(VFIORegion *region, MCPortal *mcp,
                         VFIOFSLMC_cmdif *mc_cmdif)
{
    VFIOFslmcDevice *vdev = mc_cmdif->mcportal_vdev;
    struct dprc_set_obj_irq_cmd *cmd =
               (struct dprc_set_obj_irq_cmd *)&mcp->p.data[0];
    MSIMessage msg;
    char *obj_type;
    int obj_id;
    int ret;

    obj_id = cmd->obj_id;
    obj_type = cmd->obj_type;

    if (strncmp(obj_type, "dprc", 4) != 0) {
        vdev = get_vdev_in_dprc(vdev, obj_type, obj_id);
        if (vdev == NULL) {
            fslmc_set_cmd_status(&mcp->p.header, MC_CMD_STATUS_CONFIG_ERR);
            return;
        }
    }

    msg.address = cmd->irq_addr;
    msg.data = cmd->irq_val;

    fslmc_set_msi_message(&vdev->mcdev, msg, cmd->irq_index);

    ret = vfio_set_kvm_msi_irqfd(vdev, cmd->irq_index);
    if (ret) {
        fslmc_set_cmd_status(&mcp->p.header, MC_CMD_STATUS_CONFIG_ERR);
        return;
    }
    fslmc_set_cmd_status(&mcp->p.header, MC_CMD_STATUS_OK);
}

static void vfio_handle_fslmc_command(VFIORegion *region,
                                      VFIOFslmcDevice *vdev)
{
    MCPortal *mcp = &vdev->mcportal;
    int cmd = fslmc_get_cmd(mcp->p.header);
    uint16_t token = fslmc_get_cmd_token(mcp->p.header);
    VFIOFSLMC_cmdif *mc_cmdif = NULL;

    /* We support MC f/w 10.0.0 and above */
    if(fslmc_get_command_version(mcp->p.header) == MC_CMD_HDR_NO_VER) {
        fslmc_set_cmd_status(&mcp->p.header, MC_CMD_STATUS_UNSUPPORTED_OP);
        return;
    }

    if ((cmd != DPRC_CMD_CODE_OPEN)) {
        if (fslmc_auth_cmd(token)) {
            fslmc_set_cmd_status(&mcp->p.header, MC_CMD_STATUS_AUTH_ERR);
            return;
        }
        mc_cmdif = get_mc_cmdif_from_token(token);
    }

    switch (cmd) {
    case DPRC_CMD_CODE_OPEN:
        dprc_open(region, mcp);
        break;
    case DPRC_CMD_CODE_CLOSE:
        dprc_close(region, mcp, token);
        break;
    case DPRC_CMD_CODE_GET_ATTR:
        dprc_handle_get_attr(region, mcp, mc_cmdif);
        break;
    case DPRC_CMD_CODE_GET_OBJ_REG:
        dprc_get_obj_region(region, mcp, mc_cmdif);
        break;
    case DPRC_CMD_CODE_SET_IRQ:
        dprc_set_irq(region, mcp, mc_cmdif);
        break;
    case DPRC_CMD_CODE_GET_IRQ:
        dprc_get_irq(region, mcp, mc_cmdif);
        break;
    case DPRC_CMD_CODE_SET_OBJ_IRQ:
        dprc_set_obj_irq(region, mcp, mc_cmdif);
        break;
    default:
        printf("%s: Unsupported MC Command %x\n", __func__, cmd);
        fslmc_set_cmd_status(&mcp->p.header, MC_CMD_STATUS_UNSUPPORTED_OP);
        break;
    }
}

/* Each MC Portals are of 64kB size and first 64B are used for
 * command/response. Accesses outside the 64 bytes of the portal
 * are not permitted
 *
 * MC Command format
 *  8 Bytes (Offset 0 to 7) - command header
 *  56 Bytes (Offset 8 - 63) - Command Data
 *
 * MC command is complete when status is written in cmoomand header.
 */
static void vfio_fsl_mc_prepare_cmd(VFIORegion *region, hwaddr offset,
                                  unsigned size, uint64_t value)
{
    VFIODevice *vbasedev = region->vbasedev;
    VFIOFslmcDevice *vdev =
        container_of(vbasedev, VFIOFslmcDevice, vbasedev);
    uint64_t data = value;

    if (size == 4) {
        data = vdev->mcportal.portal[offset / 8];
        if (offset & 4) {
            data &= 0x00000000FFFFFFFFULL;
            data |= (value << 32);
        } else {
            data &= 0xFFFFFFFF00000000ULL;
            data |= value;
        }
    }

    vdev->mcportal.portal[offset / 8] = data;

    /* Handle when mc command is complete */
    if ((size == 8 && !offset) || (offset == 0x4)) {
        vfio_handle_fslmc_command(region, vdev);
    }
}

static uint64_t vfio_fsl_mc_portal_read(VFIODevice *vbasedev, hwaddr offset)
{
    uint64_t value;
    VFIOFslmcDevice *vdev =
        container_of(vbasedev, VFIOFslmcDevice, vbasedev);

    value = vdev->mcportal.portal[offset / 8];

    return value;
}

static uint64_t vfio_fsl_mc_region_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    VFIORegion *region = opaque;
    VFIODevice *vbasedev = region->vbasedev;
    VFIOFslmcDevice *vdev =
        container_of(vbasedev, VFIOFslmcDevice, vbasedev);
    union {
        uint32_t dword;
        uint64_t qword;
    } buf;
    uint64_t data = 0;

    /* Support read on DPRC devices only */
    if (strncmp(vdev->device_type, "dprc", 4)) {
        printf("Read to device (%s) type not supported", vdev->device_type);
        return 0;
    }

    /* Accesses outside the 64 bytes of the portal are not permitted */
    if (offset > 64 || ((offset + size) > 64)) {
        printf("vfio: unsupported read to offset, %lx\n", offset);
        /* FIXME: How MC f/w respond ? Behave like MC f/w */
        return 0;
    }

    buf.qword = vfio_fsl_mc_portal_read(vbasedev, offset);

    switch (size) {
    case 4:
        data = le32_to_cpu(buf.dword);
        break;
    case 8:
        data = le64_to_cpu(buf.qword);
        break;
    default:
        hw_error("vfio: unsupported read size, %d bytes", size);
        break;
    }

    return data;
}

static void vfio_fsl_mc_region_write(void *opaque, hwaddr offset,
                              uint64_t data, unsigned size)
{
    VFIORegion *region = opaque;
    VFIODevice *vbasedev = region->vbasedev;
    VFIOFslmcDevice *vdev =
        container_of(vbasedev, VFIOFslmcDevice, vbasedev);
    union {
        uint32_t dword;
        uint64_t qword;
    } buf;

    /* Support read on DPRC devices only */
    if (strncmp(vdev->device_type, "dprc", 4)) {
        printf("Write to device (%s) not supported", vdev->device_type);
        return;
    }

    /* Accesses outside the 64 bytes of portal are not permitted */
    if (offset > 64 || ((offset + size) > 64)) {
        printf("vfio: unsupported write to offset, %lx\n", offset);
        /* FIXME: How MC f/w respond ? Behave like MC f/w */
        return;
    }

    switch (size) {
    case 4:
        buf.dword = cpu_to_le32(data);
        break;
    case 8:
        buf.qword = cpu_to_le64(data);
        break;
    default:
        hw_error("vfio: unsupported write size, %d bytes", size);
        break;
    }
    vfio_fsl_mc_prepare_cmd(region, offset, size, buf.qword);
}

const MemoryRegionOps vfio_region_fsl_mc_ops = {
    .read = vfio_fsl_mc_region_read,
    .write = vfio_fsl_mc_region_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
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

/* If a filename starts with "dp", have a dot (.) and a device-id
 * (integer value) then it is a valid fsl-mc device
 */
static bool vfio_fslmc_is_mcdev(const char* mcdev_name)
{
    char *name;
    char *endp;
    unsigned long value;

    if (memcmp("dp", mcdev_name, 2)) {
        return false;
    }

    name = strchr(mcdev_name, '.');
    if (!strchr(mcdev_name, '.')) {
        return false;
    }

    value = strtoul(++name, &endp, 10);
    if ((value > INT32_MAX) || (endp == name)) {
        return false;
    }

    return true;
}

static void vfio_fsl_mc_create_qdev(VFIOFslmcDevice *parent_vdev,
                                    const char *mcdev_name)
{
    DeviceState *dev;

    dev = qdev_create(&parent_vdev->mcdev.bus->qbus, TYPE_VFIO_FSL_MC);
    dev->id = TYPE_VFIO_FSL_MC;
    qdev_prop_set_string(dev, "host", mcdev_name);
    qdev_prop_set_ptr(dev, "parent_vdev", parent_vdev);
    /* Do not mmap dprc region, instead use slowpatch.
     * TODO: Selectively allow dpmcp device region mmap
     */
    if (strncmp(mcdev_name, "dprc", 4) == 0) {
        qdev_prop_set_bit(dev, "x-no-mmap", true);
    } else {
        qdev_prop_set_bit(dev, "x-no-mmap", false);
    }

    qdev_init_nofail(dev);
}

static void vfio_fsl_mc_scan_dprc(VFIOFslmcDevice *vdev)
{
    VFIODevice *vbasedev = &vdev->vbasedev;
    char dev_syspath[FSLMC_DEV_SYSPATH_LEN];
    DIR *dir;
    struct dirent *entry;
    int len;

    if (strncmp(vdev->device_type, "dprc", 10)) {
        goto out;
    }

    memset(&dev_syspath[0], 0, FSLMC_DEV_SYSPATH_LEN);
    strncpy(dev_syspath, FSLMC_HOST_SYSFS_PATH, FSLMC_DEV_SYSPATH_LEN);
    len = strlen(dev_syspath);
    strncat(dev_syspath, vbasedev->name, (FSLMC_DEV_SYSPATH_LEN - len));
    dir = opendir(dev_syspath);
    if (!dir) {
        error_report("vfio-fslmc: Failed to open directory: %s\n", dev_syspath);
        goto out;
    }

    while ((entry = readdir(dir))) {
        if (!vfio_fslmc_is_mcdev(entry->d_name)) {
            continue;
        }

        vfio_fsl_mc_create_qdev(vdev, entry->d_name);
    }

    closedir(dir);
out:
    return;
}

static void vfio_fsl_mc_realize(FslMcDeviceState *mcdev, Error **errp)
{
    VFIOFslmcDevice *vdev = DO_UPCAST(VFIOFslmcDevice, mcdev, mcdev);
    VFIODevice *vbasedev = &vdev->vbasedev;
    VFIOFslmcDevice *parent_vdev = (VFIOFslmcDevice *)vdev->parent_vdev;
    FslMcDeviceState *pmcdev = NULL;
    int ret, i;
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
    } else {
        /* If device is not "dprc" then it must have a parent dprc */
        if (parent_vdev == NULL) {
            error_setg(errp, "Device does not have parent dprc device");
            return;
        }

        pmcdev = &parent_vdev->mcdev;
        /* Add to device list of parent DPRC */
        QLIST_INSERT_HEAD(&parent_vdev->device_list, vdev, next);
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

    for (i = 0; i < vbasedev->num_regions; i++) {
        VFIORegion *region;

        if (vfio_region_mmap(vdev->regions[i])) {
            error_report("%s mmap unsupported. Performance may be slow",
                         memory_region_name(vdev->regions[i]->mem));
        }

        region = vdev->regions[i];
        ret = fsl_mc_register_device_region(mcdev, i, region->mem,
                                            vdev->device_type);
        if (ret) {
            error_setg(errp,  "Failed to Register device");
            return;
        }
        vfio_region_mmaps_set_enabled(region, true);
    }

    /* Scan dprc container and realize child devices */
    vfio_fsl_mc_scan_dprc(vdev);
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
    DEFINE_PROP_BOOL("x-no-mmap", VFIOFslmcDevice, vbasedev.no_mmap, true),
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
