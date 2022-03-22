/*
 * Simple ramdisk implementation to test virtIO-blk
 * Copyright (c) 2022 UNSW
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SECTOR_SIZE   512

/*
 * Provide a 256Mb RAM disk
 */
#define DISK_SIZE (256*(1024*1024))/SECTOR_SIZE)

uint8_t disc[SECTOR_SIZE*DISK_SIZE];
static struct virtio_blk_config disk_config  = {
    .capacity = DISK_SIZE,
    .blk_size = 4096,
    .physical_block_exponent = 3, // 4k blocks = 2^3 sectors
    .min_io_size = 1,
    .opt_io_size = 1
};

static int read(void *buf, uint32_t offset, size_t nsec)
{
    if (offset + nsec > DISK_SIZE)
        nsec = DISK_SIZE - offset;
    memcpy(buf, disc + offset * SECTOR_SIZE, nsec * SECTOR_SIZE);
    return nsec;
}


static int write(void *buf, uint32_t offset, size_t nsec)
{
    if (offset + nsec > DISK_SIZE)
        nsec = DISK_SIZE - offset;
    memcpy(disc + offset * SECTOR_SIZE, buf, nsec * SECTOR_SIZE);
    return nsec;
}

/*
 * Handle reads from PCI BAR 0
 */
bool blk_device_emul_io_in(struct virtio_emul *emul,
                           unsigned int offset,
                           unsigned int size,
                           unsigned int *result)
{
    bool handled = false;
    switch (offset) {
    case VIRTIO_PCI_HOST_FEATURES:
        handled = true;
        assert(size == 4);
        *result = BIT(VIRTIO_BLK_F_BLK_SIZE);  // currently provide a bare-bones LBA only device
        break;

    case VIRTIO_PCI_ISR_STATUS:
        break;

    case 0x14 ... 0x28:
        *result = *(unsigned int *)((char *)&disk_config + port);
        handled = true;
        break;
    }
    return handled;
}

bool blk_device_emul_io_out(struct virtio_emul *emul,
                            unsigned int offset,
                            unsigned int size,
                            unsigned int value)
{
    bool handled = false;
    switch (offset) {
    case VIRTIO_PCI_GUEST_FEATURES:
        handled = true;
        break;
    }
    
    return handled;
}

static void
emul_notify_tx(virto_emul_t *emul)
{
    blk_internal_t *blk = (blk_internal_t *)emul->internal;
    struct vring *vring = &emul->virtq.vring[TX_QUEUE];
    /* read the index */
    uint16_t guest_idx = ring_avail_idx(emul, vring);
    /* process what we can of the ring */
    uint16_t idx = emul->virtq.last_idx[TX_QUEUE];
    while (idx != guest_idx) {
        uint16_t desc_head;
        /* read the head of the descriptor chain */
        desc_head = ring_avail(emul, vring, idx);
        struct vring_desc desc;
        uint16_t desc_idx = desc_head;
        do {
            desc = ring_desc(emul, vring, desc_idx);
            //TODO --- Grab SG lists and do them
        }
    }
}
