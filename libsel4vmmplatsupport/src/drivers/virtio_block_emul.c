/*
 * Copyright 2019, Dornerworks
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <sel4vmmplatsupport/drivers/virtio_pci_emul.h>
#include <stdbool.h>

#include "virtio_emul_helpers.h"

#define BUF_SIZE 8192
#define MAX_DATA_BUF_SIZE 4096

#define NUM_REQUEST_ADDRS 3

typedef struct blkif_virtio_emul_internal {
    struct disk_driver driver;
    struct virtio_blk_config cfg;
    ps_dma_man_t dma_man;
} blkif_virtio_emul_internal_t;

typedef struct emul_tx_cookie {
    uint16_t desc_head;
    void *vaddr;
} emul_tx_cookie_t;

static void complete_virtio_blk_request(void *iface, void *cookie)
{
    virtio_emul_t *emul = (virtio_emul_t *) iface;
    blkif_virtio_emul_internal_t *blk = emul->internal;
    emul_tx_cookie_t *tx_cookie = (emul_tx_cookie_t *)cookie;
    /* free the dma memory */
    ps_dma_unpin(&blk->dma_man, tx_cookie->vaddr, BUF_SIZE);
    ps_dma_free(&blk->dma_man, tx_cookie->vaddr, BUF_SIZE);
    /* put the descriptor chain into the used list */
    struct vring_used_elem used_elem = {tx_cookie->desc_head, 0};
    ring_used_add(emul, &emul->virtq.vring[emul->virtq.queue], used_elem);
    free(tx_cookie);
    /* notify the guest that we have completed some of its buffers */
    blk->driver.i_fn.raw_handleIRQ(&blk->driver, 0);
}

static void handle_virtio_blk_request(virtio_emul_t *emul)
{
    /* Create Local Copies of the Passed in Structure */
    blkif_virtio_emul_internal_t *blk = (blkif_virtio_emul_internal_t *) emul->internal;
    struct vring *vring = &emul->virtq.vring[emul->virtq.queue];

    /* read the index */
    uint16_t guest_idx = ring_avail_idx(emul, vring);

    /* process what we can of the ring */
    uint16_t idx = emul->virtq.last_idx[emul->virtq.queue];
    uint32_t buf_len = 0;

    uint64_t desc_addrs[NUM_REQUEST_ADDRS];

    while (idx != guest_idx) {
        uint16_t desc_head;

        /* read the head of the descriptor chain */
        desc_head = ring_avail(emul, vring, idx);

        /* allocate a packet */
        void *vaddr = ps_dma_alloc(&blk->dma_man, BUF_SIZE, blk->driver.dma_alignment, 1, PS_MEM_NORMAL);
        if (!vaddr) {
            /* try again later */
            break;
        }
        uintptr_t phys = ps_dma_pin(&blk->dma_man, vaddr, BUF_SIZE);
        assert(phys);

        /* length of the final packet to deliver */
        uint32_t len = 0;

        /* start walking the descriptors */
        struct vring_desc desc;
        uint16_t desc_idx = desc_head;
        int i = 0;
        do {
            desc = ring_desc(emul, vring, desc_idx);
            /* truncate packets that are too large */
            uint32_t this_len = MIN(BUF_SIZE - len, desc.len);
            vm_guest_read_mem(emul->vm, vaddr + len, (uintptr_t) desc.addr, this_len);
            /* Save off the descriptor addresses so we can write back to the VM */
            desc_addrs[i] = desc.addr;
            /* The second descriptor (index 1) is the data buffer.
             *  The length of this buffer determines how much we need to
             *  copy to or from this buffer.
             */
            if (i == 1) {
                buf_len = desc.len;
            }
            i++;
            len += this_len;
            desc_idx = desc.next;
        } while (desc.flags & VRING_DESC_F_NEXT);
        /* ship it */
        emul_tx_cookie_t *cookie = calloc(1, sizeof(*cookie));
        assert(cookie);
        cookie->desc_head = desc_head;
        cookie->vaddr = vaddr;

        /* Currently we can only handle buffers of a certain size or less.
         *  We could fix this, but not sure if it is necessary based on the
         *  FileSystem types that have been tested
         */
        assert(buf_len <= MAX_DATA_BUF_SIZE);

        struct virtio_blk_outhdr hdr;
        memcpy(&hdr, vaddr, sizeof(struct virtio_blk_outhdr));

        /* Calculate the addresses to which we actually write data */
        void *guest_buf_start = vaddr + sizeof(struct virtio_blk_outhdr);
        void *req_status_start = vaddr + sizeof(struct virtio_blk_outhdr) + buf_len;

        /* Start disk read or write chain */
        int result = blk->driver.i_fn.raw_xfer(&blk->driver, hdr.type, hdr.sector, buf_len, (uintptr_t) guest_buf_start);

        switch (result) {
        case VIRTIO_BLK_XFER_COMPLETE:
            *(uint8_t *)req_status_start = VIRTIO_BLK_S_OK;
            if (VIRTIO_BLK_T_IN == hdr.type) {
                /* We assume descriptor address at index 1 is the buffer */
                vm_guest_write_mem(emul->vm, vaddr + sizeof(struct virtio_blk_outhdr), desc_addrs[1], buf_len);
            }
            /* We assume descriptor address at index 2 is the status of the IO cmd*/
            vm_guest_write_mem(emul->vm, vaddr + sizeof(struct virtio_blk_outhdr) + buf_len, desc_addrs[2], 1);
            complete_virtio_blk_request(emul, cookie);
            break;
        case VIRTIO_BLK_XFER_FAILED:
            *(uint8_t *)req_status_start = VIRTIO_BLK_S_IOERR;
            vm_guest_write_mem(emul->vm, vaddr + sizeof(struct virtio_blk_outhdr) + buf_len, desc_addrs[2], 1);
            complete_virtio_blk_request(emul, cookie);
            break;
        }
        /* next */
        idx++;
    }
    /* update which parts of the ring we have processed */
    emul->virtq.last_idx[emul->virtq.queue] = idx;
}

static bool emul_io_in(struct virtio_emul *emul, unsigned int offset, unsigned int size, unsigned int *result)
{
    bool handled = false;
    blkif_virtio_emul_internal_t *blkif_internal = emul->internal;
    switch (offset) {
    case VIRTIO_PCI_HOST_FEATURES:
        handled = true;
        assert(size == 4);
        *result = (BIT(VIRTIO_BLK_F_BLK_SIZE) | BIT(VIRTIO_BLK_F_SEG_MAX) | BIT(VIRTIO_BLK_F_SIZE_MAX));
        break;
    case VIRTIO_PCI_CONFIG_OFF(0) ... VIRTIO_PCI_CONFIG_OFF(0) + sizeof(struct virtio_blk_config):
        handled = true;
        assert(size == 1);
        memcpy(result, (((uint8_t *)&blkif_internal->cfg) + offset - VIRTIO_PCI_CONFIG_OFF(0)), size);
        break;
    }
    return handled;
}

static bool emul_io_out(struct virtio_emul *emul, unsigned int offset, unsigned int size, unsigned int value)
{
    bool handled = false;
    blkif_virtio_emul_internal_t *blkif_internal = emul->internal;
    switch (offset) {
    case VIRTIO_PCI_GUEST_FEATURES:
        handled = true;
        assert(size == 4);
        /* Guest can support a subset of these features */
        assert(value & (BIT(VIRTIO_BLK_F_BLK_SIZE) | BIT(VIRTIO_BLK_F_SEG_MAX) | BIT(VIRTIO_BLK_F_SIZE_MAX)));
        break;
    case VIRTIO_PCI_QUEUE_NOTIFY:
        handled = true;
        handle_virtio_blk_request(emul);
    }
    return handled;
}

static void emul_notify(virtio_emul_t *emul)
{
    if (emul->virtq.status != VIRTIO_CONFIG_S_DRIVER_OK) {
        return;
    }
    handle_virtio_blk_request(emul);
}

void *block_virtio_emul_init(virtio_emul_t *emul, ps_io_ops_t io_ops, diskif_driver_init driver, void *config)
{
    blkif_virtio_emul_internal_t *internal = NULL;

    int err;
    internal = calloc(1, sizeof(*internal));
    if (!emul || !internal) {
        goto error;
    }
    emul->device_io_in = emul_io_in;
    emul->device_io_out = emul_io_out;
    emul->notify = emul_notify;
    internal->driver.cb_cookie = emul;
    internal->dma_man = io_ops.dma_manager;
    err = driver(&internal->driver, io_ops, config);
    if (err) {
        ZF_LOGE("Fafiled to initialize driver");
        goto error;
    }
    internal->driver.i_fn.low_level_init(&internal->driver, &internal->cfg);
    return (void *)internal;
error:
    if (internal) {
        free(internal);
    }
    return NULL;
}
