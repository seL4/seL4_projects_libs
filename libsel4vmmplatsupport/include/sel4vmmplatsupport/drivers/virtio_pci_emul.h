/*
 * Copyright 2022, UNSW (ABN 57 195 873 179)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <autoconf.h>
#include <string.h>
#include <platsupport/io.h>
#include <ethdrivers/raw.h>
#include <satadrivers/raw.h>
#include <sel4vmmplatsupport/drivers/virtio_pci_console.h>
#include <sel4vm/guest_vm.h>
#include <virtio/virtio_ring.h>
#include <virtio/virtio_pci.h>
#include <virtio/virtio_net.h>
#include <virtio/virtio_con.h>
#include <virtio/virtio_config.h>

#define RX_QUEUE 0
#define TX_QUEUE 1

typedef enum virtio_pci_devices {
    VIRTIO_NET,
    VIRTIO_CONSOLE,
    VIRTIO_BLOCK,
} virtio_pci_devices_t;

#define VQUEUE_NUM_VRINGS (VIRTIO_CON_MAX_PORTS*2+2)
typedef struct v_queue {
    int status;
    uint16_t queue;
    struct vring vring[VQUEUE_NUM_VRINGS];
    uint16_t queue_size[VQUEUE_NUM_VRINGS];
    uint32_t queue_pfn[VQUEUE_NUM_VRINGS];
    uint16_t last_idx[VQUEUE_NUM_VRINGS];
} vqueue_t;

typedef struct virtio_emul {
    /* pointer to internal information */
    void *internal;
    /* generic io port interface functions */
    int (*io_in)(struct virtio_emul *emul, unsigned int offset, unsigned int size, unsigned int *result);
    int (*io_out)(struct virtio_emul *emul, unsigned int offset, unsigned int size, unsigned int value);
    /* notify of a status change in the underlying driver.
     * typically this would be due to link coming up
     * meaning that transmits can finally happen */
    void (*notify)(struct virtio_emul *emul);
    /* device specific io port interface functions*/
    bool (*device_io_in)(struct virtio_emul *emul, unsigned int offset, unsigned int size, unsigned int *result);
    bool (*device_io_out)(struct virtio_emul *emul, unsigned int offset, unsigned int size, unsigned int result);
    /* generic virtqueue structure */
    vqueue_t virtq;
    vm_t *vm;
} virtio_emul_t;

virtio_emul_t *virtio_emul_init(ps_io_ops_t io_ops, int queue_size, vm_t *vm, void *driver,
                                void *config, virtio_pci_devices_t device);

void ring_used_add(virtio_emul_t *emul, struct vring *vring, struct vring_used_elem elem);

struct vring_desc ring_desc(virtio_emul_t *emul, struct vring *vring, uint16_t idx);

uint16_t ring_avail_idx(virtio_emul_t *emul, struct vring *vring);

uint16_t ring_avail(virtio_emul_t *emul, struct vring *vring, uint16_t idx);

void *net_virtio_emul_init(virtio_emul_t *emul, ps_io_ops_t io_ops, ethif_driver_init driver, void *config);

void *console_virtio_emul_init(virtio_emul_t *emul, ps_io_ops_t io_ops, console_driver_init driver, void *config);

void *block_virtio_emul_init(virtio_emul_t *emul, ps_io_ops_t io_ops, diskif_driver_init driver, void *config);
