/*
 * Copyright 2022, UNSW (ABN 57 195 873 179)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <sel4vmmplatsupport/drivers/virtio_pci_emul.h>

#include "virtio_emul_helpers.h"

/* Temporary buffer used during TX */
char buf[VIRTIO_VSOCK_CAMKES_MTU];

typedef struct vsock_internal {
    struct vsock_passthrough driver;
    /* what queue did data come through? */
    int queue_num;
} vsock_internal_t;

static void emul_vsock_rx_complete(virtio_emul_t *emul, char *buf, unsigned int len)
{
    vsock_internal_t *vsock = emul->internal;
    vqueue_t *virtq = &emul->virtq;
    int i;
    struct vring *vring = &virtq->vring[RX_QUEUE];

    uint16_t guest_idx = ring_avail_idx(emul, vring);
    uint16_t idx = virtq->last_idx[RX_QUEUE];

    if (idx != guest_idx) {
        /* total length of the written packet so far */
        size_t tot_written = 0;
        /* amount of the current descriptor written */
        size_t desc_written = 0;
        /* how much we have written of the current buffer */
        size_t buf_written = 0;
        uint16_t desc_head = ring_avail(emul, vring, idx);
        /* start walking the descriptors */
        struct vring_desc desc;
        uint16_t desc_idx = desc_head;
        do {
            desc = ring_desc(emul, vring, desc_idx);
            /* determine how much we can copy */
            uint32_t copy;
            copy = len - buf_written;
            copy = MIN(copy, desc.len - desc_written);
            vm_guest_write_mem(emul->vm, buf + buf_written, (uintptr_t)desc.addr + desc_written, copy);
            /* update amounts */
            tot_written += copy;
            desc_written += copy;
            buf_written += copy;
            /* see what's gone over */
            if (desc_written == desc.len) {
                if (!desc.flags & VRING_DESC_F_NEXT) {
                    /* descriptor chain is too short to hold the whole packet.
                     * just truncate */
                    break;
                }
                desc_idx = desc.next;
                desc_written = 0;
            }
        } while (buf_written != len);
        /* now put it in the used ring */
        struct vring_used_elem used_elem = {desc_head, tot_written};
        ring_used_add(emul, vring, used_elem);

        /* record that we've used this descriptor chain now */
        virtq->last_idx[RX_QUEUE]++;
        /* notify the guest that there is something in its used ring */
        vsock->driver.injectIRQ(vsock->driver.vsock_data);
    }
}

void vsock_rx_complete(virtio_emul_t *emul, char *buf, unsigned int len)
{
    emul_vsock_rx_complete(emul, buf, len);
}

static void vsock_handle_packet(virtio_emul_t *emul, void *buffer, unsigned int len)
{
    vsock_internal_t *vsock = emul->internal;
    struct virtio_vsock_packet *packet = (struct virtio_vsock_packet *) buffer;
    int cid = packet->hdr.dst_cid;

    /* If we truncated the packet earlier, make sure header reflects new len */
    if (len == VIRTIO_VSOCK_CAMKES_MTU) {
        packet->hdr.len = VIRTIO_VSOCK_CAMKES_MTU - sizeof(struct virtio_vsock_hdr);
    }

    vsock->driver.forward(cid, buffer, len);
}

static void emul_vsock_notify_tx(virtio_emul_t *emul)
{
    vsock_internal_t *vsock = emul->internal;
    vqueue_t *virtq = &emul->virtq;
    struct vring *vring = &virtq->vring[vsock->queue_num];

    /* read the index */
    uint16_t guest_idx = ring_avail_idx(emul, vring);

    /* process what we can of the ring */
    uint16_t idx = virtq->last_idx[vsock->queue_num];
    while (idx != guest_idx) {
        /* read the head of the descriptor chain */
        uint16_t desc_head = ring_avail(emul, vring, idx);
        /* length of the final packet to deliver */
        uint32_t len = 0;
        /* start walking the descriptors */
        struct vring_desc desc;
        uint16_t desc_idx = desc_head;

        do {
            desc = ring_desc(emul, vring, desc_idx);

            /* truncate packets that are too large */
            uint32_t this_len = MIN(VIRTIO_VSOCK_CAMKES_MTU - len, desc.len);
            vm_guest_read_mem(emul->vm, buf + len, (uintptr_t)desc.addr, this_len);
            len += this_len;
            desc_idx = desc.next;
        } while (desc.flags & VRING_DESC_F_NEXT && len < VIRTIO_VSOCK_CAMKES_MTU);

        /* Handle the packet */
        vsock_handle_packet(emul, buf, len);

        /* next */
        idx++;
        struct vring_used_elem used_elem = {desc_head, 0};
        ring_used_add(emul, &virtq->vring[vsock->queue_num], used_elem);
        vsock->driver.injectIRQ(vsock->driver.vsock_data);
    }
    /* update which parts of the ring we have processed */
    virtq->last_idx[vsock->queue_num] = idx;
}

static bool vsock_device_emul_io_in(struct virtio_emul *emul, unsigned int offset, unsigned int size,
                                    unsigned int *result)
{
    vsock_internal_t *vsock = emul->internal;

    bool handled = false;
    switch (offset) {
    case VIRTIO_PCI_HOST_FEATURES:
        handled = true;
        assert(size == 4);
        /* There are no feature bits for virtIO sock. */
        *result = 0;
        break;
    case VIRTIO_VSOCK_CFG_GUEST_CID ... VIRTIO_VSOCK_CFG_GUEST_CID + 7:
        handled = true;
        assert(size == 1);

        /* Set VIRTIO_VSOCK_CFG_GUEST_CID in little-endian */
        if (offset == VIRTIO_VSOCK_CFG_GUEST_CID) {
            *result = vsock->driver.guest_cid;
        } else {
            *result = 0;
        }

        break;
    }

    return handled;
}

static bool vsock_device_emul_io_out(struct virtio_emul *emul, unsigned int offset, unsigned int size,
                                     unsigned int value)
{
    vsock_internal_t *vsock = emul->internal;

    bool handled = false;
    switch (offset) {
    case VIRTIO_PCI_GUEST_FEATURES:
        handled = true;
        assert(size == 4);
        /* There are no feature bits for virtIO sock. */
        assert(value == 0);
        break;
    case VIRTIO_PCI_QUEUE_NOTIFY:
        handled = true;
        vsock->queue_num = value;
        if (value == RX_QUEUE) {
            /* Ignore RX packets for now (see virtio_emul.c) */
        } else if (value == EVENT_QUEUE) {

        } else {
            /* Generic TX handler, right now the event queue only gets
             * notified when the guest wants the VMM to fill info in the
             * event queue */
            emul->notify(emul);
        }
        break;
    }

    return handled;
}

void *vsock_virtio_emul_init(virtio_emul_t *emul, ps_io_ops_t io_ops, vsock_driver_init driver, void *config)
{
    vsock_internal_t *internal = calloc(1, sizeof(*internal));
    if (!internal) {
        goto error;
    }

    emul->notify = emul_vsock_notify_tx;
    emul->device_io_in = vsock_device_emul_io_in;
    emul->device_io_out = vsock_device_emul_io_out;

    int err = driver(&internal->driver, io_ops, config);
    if (err) {
        ZF_LOGE("Failed to initialize driver");
        goto error;
    }

    return (void *)internal;
error:
    if (emul) {
        free(emul);
    }
    if (internal) {
        free(internal);
    }
    return NULL;
}
