/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vmmplatsupport/drivers/virtio_pci_emul.h>
#include <stdbool.h>

#include "virtio_emul_helpers.h"

#define BUF_SIZE 2048

typedef struct ethif_virtio_emul_internal {
    struct eth_driver driver;
    uint8_t mac[6];
    ps_dma_man_t dma_man;
} ethif_internal_t;

typedef struct emul_tx_cookie {
    uint16_t desc_head;
    void *vaddr;
} emul_tx_cookie_t;

static uintptr_t emul_allocate_rx_buf(void *iface, size_t buf_size, void **cookie)
{
    virtio_emul_t *emul = (virtio_emul_t *)iface;
    ethif_internal_t *net = emul->internal;
    if (buf_size > BUF_SIZE) {
        return 0;
    }
    void *vaddr = ps_dma_alloc(&net->dma_man, BUF_SIZE, net->driver.dma_alignment, 1, PS_MEM_NORMAL);
    if (!vaddr) {
        return 0;
    }
    uintptr_t phys = ps_dma_pin(&net->dma_man, vaddr, BUF_SIZE);
    *cookie = vaddr;
    return phys;
}

static void emul_rx_complete(void *iface, unsigned int num_bufs, void **cookies, unsigned int *lens)
{
    virtio_emul_t *emul = (virtio_emul_t *)iface;
    ethif_internal_t *net = (ethif_internal_t *)emul->internal;
    vqueue_t *vq = &emul->virtq;
    int i;
    struct vring *vring = &vq->vring[RX_QUEUE];

    /* grab the next receive chain */
    struct virtio_net_hdr virtio_hdr;
    memset(&virtio_hdr, 0, sizeof(virtio_hdr));
    uint16_t guest_idx = ring_avail_idx(emul, vring);
    uint16_t idx = vq->last_idx[RX_QUEUE];
    if (idx != guest_idx) {
        /* total length of the written packet so far */
        size_t tot_written = 0;
        /* amount of the current descriptor written */
        size_t desc_written = 0;
        /* how much we have written of the current buffer */
        size_t buf_written = 0;
        /* the current buffer. -1 indicates the virtio net buffer */
        int current_buf = -1;
        uint16_t desc_head = ring_avail(emul, vring, idx);
        /* start walking the descriptors */
        struct vring_desc desc;
        uint16_t desc_idx = desc_head;
        do {
            desc = ring_desc(emul, vring, desc_idx);
            /* determine how much we can copy */
            uint32_t copy;
            void *buf_base = NULL;
            if (current_buf == -1) {
                copy = sizeof(struct virtio_net_hdr) - buf_written;
                buf_base = &virtio_hdr;
            } else {
                copy = lens[current_buf] - buf_written;
                buf_base = cookies[current_buf];
            }
            copy = MIN(copy, desc.len - desc_written);
            vm_guest_write_mem(emul->vm, buf_base + buf_written, (uintptr_t)desc.addr + desc_written, copy);
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
            if (current_buf == -1) {
                if (buf_written == sizeof(struct virtio_net_hdr)) {
                    current_buf++;
                    buf_written = 0;
                }
            } else {
                if (buf_written == lens[current_buf]) {
                    current_buf++;
                    buf_written = 0;
                }
            }
        } while (current_buf < num_bufs);
        /* now put it in the used ring */
        struct vring_used_elem used_elem = {desc_head, tot_written};
        ring_used_add(emul, vring, used_elem);

        /* record that we've used this descriptor chain now */
        vq->last_idx[RX_QUEUE]++;
        /* notify the guest that there is something in its used ring */
        net->driver.i_fn.raw_handleIRQ(&net->driver, 0);
    }
    for (i = 0; i < num_bufs; i++) {
        ps_dma_unpin(&net->dma_man, cookies[i], BUF_SIZE);
        ps_dma_free(&net->dma_man, cookies[i], BUF_SIZE);
    }
}

static void emul_tx_complete(void *iface, void *cookie)
{
    virtio_emul_t *emul = (virtio_emul_t *)iface;
    ethif_internal_t *net = emul->internal;
    emul_tx_cookie_t *tx_cookie = (emul_tx_cookie_t *)cookie;
    /* free the dma memory */
    ps_dma_unpin(&net->dma_man, tx_cookie->vaddr, BUF_SIZE);
    ps_dma_free(&net->dma_man, tx_cookie->vaddr, BUF_SIZE);
    /* put the descriptor chain into the used list */
    struct vring_used_elem used_elem = {tx_cookie->desc_head, 0};
    ring_used_add(emul, &emul->virtq.vring[TX_QUEUE], used_elem);
    free(tx_cookie);
    /* notify the guest that we have completed some of its buffers */
    net->driver.i_fn.raw_handleIRQ(&net->driver, 0);
}

static void emul_notify_tx(virtio_emul_t *emul)
{
    ethif_internal_t *net = (ethif_internal_t *)emul->internal;
    struct vring *vring = &emul->virtq.vring[TX_QUEUE];
    /* read the index */
    uint16_t guest_idx = ring_avail_idx(emul, vring);
    /* process what we can of the ring */
    uint16_t idx = emul->virtq.last_idx[TX_QUEUE];
    while (idx != guest_idx) {
        uint16_t desc_head;
        /* read the head of the descriptor chain */
        desc_head = ring_avail(emul, vring, idx);
        /* allocate a packet */
        void *vaddr = ps_dma_alloc(&net->dma_man, BUF_SIZE, net->driver.dma_alignment, 1, PS_MEM_NORMAL);
        if (!vaddr) {
            /* try again later */
            break;
        }
        uintptr_t phys = ps_dma_pin(&net->dma_man, vaddr, BUF_SIZE);
        assert(phys);
        /* length of the final packet to deliver */
        uint32_t len = 0;
        /* we want to skip the initial virtio header, as this should
         * not be sent to the actual ethernet driver. This records
         * how much we have skipped so far. */
        uint32_t skipped = 0;
        /* start walking the descriptors */
        struct vring_desc desc;
        uint16_t desc_idx = desc_head;
        do {
            desc = ring_desc(emul, vring, desc_idx);
            uint32_t skip = 0;
            /* if we haven't yet skipped the full virtio net header, work
             * out how much of this descriptor should be skipped */
            if (skipped < sizeof(struct virtio_net_hdr)) {
                skip = MIN(sizeof(struct virtio_net_hdr) - skipped, desc.len);
                skipped += skip;
            }
            /* truncate packets that are too large */
            uint32_t this_len = desc.len - skip;
            this_len = MIN(BUF_SIZE - len, this_len);
            vm_guest_read_mem(emul->vm, vaddr + len, (uintptr_t)desc.addr + skip, this_len);
            len += this_len;
            desc_idx = desc.next;
        } while (desc.flags & VRING_DESC_F_NEXT);
        /* ship it */
        emul_tx_cookie_t *cookie = calloc(1, sizeof(*cookie));
        assert(cookie);
        cookie->desc_head = desc_head;
        cookie->vaddr = vaddr;
        int result = net->driver.i_fn.raw_tx(&net->driver, 1, &phys, &len, cookie);
        switch (result) {
        case ETHIF_TX_COMPLETE:
            emul_tx_complete(emul, cookie);
            break;
        case ETHIF_TX_FAILED:
            ps_dma_unpin(&net->dma_man, vaddr, BUF_SIZE);
            ps_dma_free(&net->dma_man, vaddr, BUF_SIZE);
            free(cookie);
            break;
        }
        /* next */
        idx++;
    }
    /* update which parts of the ring we have processed */
    emul->virtq.last_idx[TX_QUEUE] = idx;
}

static void emul_tx_complete_external(void *iface, void *cookie)
{
    emul_tx_complete(iface, cookie);
    /* space may have cleared for additional transmits */
    emul_notify_tx(iface);
}

static struct raw_iface_callbacks emul_callbacks = {
    .tx_complete = emul_tx_complete_external,
    .rx_complete = emul_rx_complete,
    .allocate_rx_buf = emul_allocate_rx_buf
};

static int emul_notify(virtio_emul_t *emul)
{
    if (emul->virtq.status != VIRTIO_CONFIG_S_DRIVER_OK) {
        return -1;
    }
    emul_notify_tx(emul);
    return 0;
}

bool net_device_emul_io_in(struct virtio_emul *emul, unsigned int offset,
                           unsigned int size, unsigned int *result)
{
    bool handled = false;
    switch (offset) {
    case VIRTIO_PCI_HOST_FEATURES:
        handled = true;
        assert(size == 4);
        //Net only
        *result = BIT(VIRTIO_NET_F_MAC);
        break;
    case VIRTIO_PCI_CONFIG_OFF(false) ...(VIRTIO_PCI_CONFIG_OFF(false) + VIRTIO_NET_CONFIG_MAC_SZ - 1):
        assert(size == 1);
        *result = ((ethif_internal_t *)emul->internal)->mac[offset - VIRTIO_PCI_CONFIG_OFF(false)];
        handled = true;
        break;
    }
    return handled;
}

bool net_device_emul_io_out(struct virtio_emul *emul, unsigned int offset,
                            unsigned int size, unsigned int value)
{
    bool handled = false;
    switch (offset) {
    case VIRTIO_PCI_GUEST_FEATURES:
        handled = true;
        assert(size == 4);
        //Net only
        assert(value == BIT(VIRTIO_NET_F_MAC));
        break;
    }
    return handled;
}

void *net_virtio_emul_init(virtio_emul_t *emul, ps_io_ops_t io_ops, ethif_driver_init driver, void *config)
{
    ethif_internal_t *internal = NULL;
    internal = calloc(1, sizeof(*internal));
    if (!internal) {
        goto error;
    }
    emul->notify = emul_notify_tx;
    emul->device_io_in = net_device_emul_io_in;
    emul->device_io_out = net_device_emul_io_out;
    internal->driver.cb_cookie = emul;
    internal->driver.i_cb = emul_callbacks;
    internal->dma_man = io_ops.dma_manager;
    int err = driver(&internal->driver, io_ops, config);
    if (err) {
        ZF_LOGE("Failed to initialize driver");
        goto error;
    }
    int mtu;
    internal->driver.i_fn.low_level_init(&internal->driver, internal->mac, &mtu);
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
