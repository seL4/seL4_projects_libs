/*
 * Copyright 2021, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <platsupport/io.h>

#include <sel4vmmplatsupport/drivers/virtio.h>
#include <sel4vmmplatsupport/drivers/virtio_blk.h>

#include <pci/helper.h>
#include <sel4vmmplatsupport/drivers/pci_helper.h>

#define QUEUE_SIZE 128

static ps_io_ops_t ops;

static int virtio_blk_io_in(void *cookie, unsigned int port_no, unsigned int size, unsigned int *result)
{
    virtio_blk_t *blk = (virtio_blk_t *)cookie;
    unsigned int offset = port_no - blk->iobase;
    unsigned int val;
    int err = blk->emul->io_in(blk->emul, offset, size, &val);
    if (err) {
        return err;
    }
    *result = val;
    return 0;
}

static int virtio_blk_io_out(void *cookie, unsigned int port_no, unsigned int size, unsigned int value)
{
    int ret;
    virtio_blk_t *blk = (virtio_blk_t *)cookie;
    unsigned int offset = port_no - blk->iobase;
    ret = blk->emul->io_out(blk->emul, offset, size, value);
    return ret;
}

static int emul_driver_init(struct disk_driver *driver, ps_io_ops_t io_ops, void *config)
{
    virtio_blk_t *blk = (virtio_blk_t *)config;
    driver->disk_data = config;
    driver->dma_alignment = sizeof(uintptr_t);
    driver->i_fn = blk->emul_driver_funcs;
    blk->emul_driver = driver;
    return 0;
}

static void *malloc_dma_alloc(void *cookie, size_t size, int align, int cached, ps_mem_flags_t flags)
{
    assert(cached);
    int error;
    void *ret;
    error = posix_memalign(&ret, align, size);
    if (error) {
        return NULL;
    }
    return ret;
}

static void malloc_dma_free(void *cookie, void *addr, size_t size)
{
    free(addr);
}

static uintptr_t malloc_dma_pin(void *cookie, void *addr, size_t size)
{
    return (uintptr_t)addr;
}

static void malloc_dma_unpin(void *cookie, void *addr, size_t size)
{
}

static void malloc_dma_cache_op(void *cookie, void *addr, size_t size, dma_cache_op_t op)
{
}

static vmm_pci_entry_t vmm_virtio_blk_pci_bar(unsigned int iobase, size_t iobase_size_bits,
                                              unsigned int interrupt_pin, unsigned int interrupt_line)
{
    vmm_pci_device_def_t *pci_config;
    int err = ps_calloc(&ops.malloc_ops, 1, sizeof(*pci_config), (void **) &pci_config);
    ZF_LOGF_IF(err, "Failed to allocate pci_config");
    *pci_config = (vmm_pci_device_def_t) {
        .vendor_id = VIRTIO_PCI_VENDOR_ID,
        .device_id = VIRTIO_BLOCK_PCI_DEVICE_ID,
        .command = PCI_COMMAND_IO,
        .header_type = PCI_HEADER_TYPE_NORMAL,
        .subsystem_vendor_id = VIRTIO_PCI_SUBSYSTEM_VENDOR_ID,
        .subsystem_id = VIRTIO_ID_BLOCK,
        .interrupt_pin = interrupt_pin,
        .interrupt_line = interrupt_line,
        .bar0 = iobase | PCI_BASE_ADDRESS_SPACE_IO,
        .cache_line_size = 64,
        .latency_timer = 64,
        .prog_if = VIRTIO_PCI_CLASS_BLOCK & 0xff,
        .subclass = (VIRTIO_PCI_CLASS_BLOCK >> 8) & 0xff,
        .class_code = (VIRTIO_PCI_CLASS_BLOCK >> 16) & 0xff,
    };
    vmm_pci_entry_t entry = (vmm_pci_entry_t) {
        .cookie = pci_config,
        .ioread = vmm_pci_mem_device_read,
        .iowrite = vmm_pci_entry_ignore_write
    };

    vmm_pci_bar_t bars[1] = {{
            .mem_type = NON_MEM,
            .address = iobase,
            .size_bits = iobase_size_bits
        }
    };
    vmm_pci_entry_t virtio_pci_bar;
    virtio_pci_bar = vmm_pci_create_bar_emulation(entry, 1, bars);

    return virtio_pci_bar;
}

virtio_blk_t *common_make_virtio_blk(vm_t *vm, vmm_pci_space_t *pci, vmm_io_port_list_t *ioport,
                                     ioport_range_t ioport_range, ioport_type_t port_type,
                                     unsigned int interrupt_pin, unsigned int interrupt_line,
                                     raw_diskiface_funcs_t backend)
{
    int err = ps_new_stdlib_malloc_ops(&ops.malloc_ops);
    ZF_LOGF_IF(err, "Failed to get malloc ops");

    virtio_blk_t *blk;
    err = ps_calloc(&ops.malloc_ops, 1, sizeof(*blk), (void **)&blk);
    ZF_LOGF_IF(err, "Failed to allocate virtio blk");

    ioport_interface_t virtio_io_interface = {blk, virtio_blk_io_in, virtio_blk_io_out, "VIRTIO PCI BLK"};
    ioport_entry_t *io_entry = vmm_io_port_add_handler(ioport, ioport_range, virtio_io_interface, port_type);
    if (!io_entry) {
        ZF_LOGE("Failed to add vmm io port handler");
        return NULL;
    }

    size_t iobase_size_bits = BYTES_TO_SIZE_BITS(io_entry->range.size);
    blk->iobase = io_entry->range.start;
    ZF_LOGE("iobase_size_bits = %zu", iobase_size_bits);

    vmm_pci_entry_t entry = vmm_virtio_blk_pci_bar(io_entry->range.start, iobase_size_bits,
                                                   interrupt_pin, interrupt_line);
    vmm_pci_add_entry(pci, entry, NULL);

    ps_io_ops_t ioops;
    ioops.dma_manager = (ps_dma_man_t) {
        .cookie = NULL,
        .dma_alloc_fn = malloc_dma_alloc,
        .dma_free_fn = malloc_dma_free,
        .dma_pin_fn = malloc_dma_pin,
        .dma_unpin_fn = malloc_dma_unpin,
        .dma_cache_op_fn = malloc_dma_cache_op
    };

    blk->emul_driver_funcs = backend;
    blk->emul = virtio_emul_init(ioops, QUEUE_SIZE, vm, emul_driver_init, blk, VIRTIO_BLOCK);

    assert(blk->emul);
    return blk;
}

static int emul_raw_xfer(struct disk_driver *driver, uint8_t direction, uint64_t sector, uint32_t len,
                         uintptr_t guest_buf_phys)
{
    ZF_LOGF("not implemented");
}

static void emul_raw_handle_irq(struct disk_driver *driver, int irq)
{
    ZF_LOGF("not implemented");
}

static void emul_raw_poll(struct disk_driver *driver)
{
    ZF_LOGF("not implemented");
}

static void emul_low_level_init(struct disk_driver *driver, struct virtio_blk_config *cfg)
{
    ZF_LOGF("not implemented");
}

static void emul_print_state(struct disk_driver *driver)
{
    ZF_LOGF("not implemented");
}

static raw_diskiface_funcs_t emul_driver_funcs = {
    .raw_xfer = emul_raw_xfer,
    .raw_handleIRQ = emul_raw_handle_irq,
    .raw_poll = emul_raw_poll,
    .print_state = emul_print_state,
    .low_level_init = emul_low_level_init
};

raw_diskiface_funcs_t virtio_blk_default_backend(void)
{
    return emul_driver_funcs;
}
