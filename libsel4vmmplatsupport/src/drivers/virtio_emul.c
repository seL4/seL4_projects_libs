/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vmmplatsupport/drivers/virtio_pci_emul.h>

#include "virtio_emul_helpers.h"

uint16_t ring_avail_idx(virtio_emul_t *emul, struct vring *vring)
{
    uint16_t idx;
    vm_guest_read_mem(emul->vm, &idx, (uintptr_t)&vring->avail->idx, sizeof(vring->avail->idx));
    return idx;
}

uint16_t ring_avail(virtio_emul_t *emul, struct vring *vring, uint16_t idx)
{
    uint16_t elem;
    vm_guest_read_mem(emul->vm, &elem, (uintptr_t) & (vring->avail->ring[idx % vring->num]), sizeof(elem));
    return elem;
}

struct vring_desc ring_desc(virtio_emul_t *emul, struct vring *vring, uint16_t idx)
{
    struct vring_desc desc;
    vm_guest_read_mem(emul->vm, &desc, (uintptr_t) & (vring->desc[idx]), sizeof(desc));
    return desc;
}

void ring_used_add(virtio_emul_t *emul, struct vring *vring, struct vring_used_elem elem)
{
    uint16_t guest_idx;
    vm_guest_read_mem(emul->vm, &guest_idx, (uintptr_t)&vring->used->idx, sizeof(vring->used->idx));
    vm_guest_write_mem(emul->vm, &elem, (uintptr_t)&vring->used->ring[guest_idx % vring->num], sizeof(elem));
    guest_idx++;
    vm_guest_write_mem(emul->vm, &guest_idx, (uintptr_t)&vring->used->idx, sizeof(vring->used->idx));
}

static int emul_io_in(virtio_emul_t *emul, unsigned int offset, unsigned int size, unsigned int *result)
{
    if (emul->device_io_in(emul, offset, size, result)) {
        return 0;
    }
    switch (offset) {
    case VIRTIO_PCI_STATUS:
        assert(size == 1);
        *result = emul->virtq.status;
        break;
    case VIRTIO_PCI_QUEUE_NUM:
        assert(size == 2);
        *result = emul->virtq.queue_size[emul->virtq.queue];
        break;
    case VIRTIO_PCI_QUEUE_PFN:
        assert(size == 4);
        *result = emul->virtq.queue_pfn[emul->virtq.queue];
        break;
    case VIRTIO_PCI_ISR:
        assert(size == 1);
        *result = 1;
        break;
    default:
        printf("Unhandled offset of 0x%x of size %d, reading\n", offset, size);
        assert(!"panic");
    }
    return 0;
}

static int emul_io_out(virtio_emul_t *emul, unsigned int offset, unsigned int size, unsigned int value)
{
    if (emul->device_io_out(emul, offset, size, value)) {
        return 0;
    }
    switch (offset) {
    case VIRTIO_PCI_STATUS:
        assert(size == 1);
        emul->virtq.status = value & 0xff;
        break;
    case VIRTIO_PCI_QUEUE_SEL:
        assert(size == 2);
        emul->virtq.queue = (value & 0xffff);
        assert(emul->virtq.queue >= 0 && emul->virtq.queue < VQUEUE_NUM_VRINGS);
        break;
    case VIRTIO_PCI_QUEUE_PFN: {
        assert(size == 4);
        int queue = emul->virtq.queue;
        emul->virtq.queue_pfn[queue] = value;
        vring_init(&emul->virtq.vring[queue], emul->virtq.queue_size[queue], (void *)((uintptr_t)value << 12),
                   VIRTIO_PCI_VRING_ALIGN);
        break;
    }
    case VIRTIO_PCI_QUEUE_NOTIFY:
        if (value == RX_QUEUE) {
            /* Currently RX packets will just get dropped if there was no space
             * so we will never have work to do if the client suddenly adds
             * more buffers */
        } else if (value == TX_QUEUE) {
            emul->notify(emul);
        } else {
            assert(!"panic");
        }
        break;
    default:
        printf("Unhandled offset of 0x%x of size %d, writing 0x%x\n", offset, size, value);
        assert(!"panic");
    }
    return 0;
}

virtio_emul_t *virtio_emul_init(ps_io_ops_t io_ops, int queue_size, vm_t *vm, void *driver,
                                void *config, virtio_pci_devices_t device)
{
    virtio_emul_t *emul = NULL;
    emul = calloc(1, sizeof(*emul));
    if (!emul) {
        free(emul);
        return NULL;
    }
    /* module specific initialisation function */
    switch (device) {
    case VIRTIO_CONSOLE:
        emul->internal = console_virtio_emul_init(emul, io_ops, (console_driver_init)driver, config);
        break;
    case VIRTIO_NET:
        emul->internal = net_virtio_emul_init(emul, io_ops, (ethif_driver_init)driver, config);
        break;
    case VIRTIO_BLOCK:
        emul->internal = block_virtio_emul_init(emul, io_ops, (diskif_driver_init)driver, config);
        break;
    case VIRTIO_VSOCK:
        emul->internal = vsock_virtio_emul_init(emul, io_ops, (vsock_driver_init)driver, config);
        break;
    }
    if (emul->internal == NULL) {
        return NULL;
    }
    emul->vm = vm;
    /* create dummy rings. we never actually dereference the rings so they can be null */
    for (int i = 0; i < VQUEUE_NUM_VRINGS; i++) {
        emul->virtq.queue_size[i] = queue_size;
        vring_init(&emul->virtq.vring[i], emul->virtq.queue_size[i], 0, VIRTIO_PCI_VRING_ALIGN);
    }
    emul->io_in = emul_io_in;
    emul->io_out = emul_io_out;

    return emul;
}
