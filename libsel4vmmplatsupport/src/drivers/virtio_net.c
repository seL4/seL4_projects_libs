/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <platsupport/io.h>

#include <sel4vmmplatsupport/drivers/virtio.h>
#include <sel4vmmplatsupport/drivers/virtio_net.h>

#include <pci/helper.h>
#include <sel4vmmplatsupport/drivers/pci_helper.h>

#define QUEUE_SIZE 128

static ps_io_ops_t ops;

static int virtio_net_io_in(void *cookie, unsigned int port_no, unsigned int size, unsigned int *result)
{
    virtio_net_t *net = (virtio_net_t *)cookie;
    unsigned int offset = port_no - net->iobase;
    unsigned int val;
    int err = net->emul->io_in(net->emul, offset, size, &val);
    if (err) {
        return err;
    }
    *result = val;
    return 0;
}

static int virtio_net_io_out(void *cookie, unsigned int port_no, unsigned int size, unsigned int value)
{
    int ret;
    virtio_net_t *net = (virtio_net_t *)cookie;
    unsigned int offset = port_no - net->iobase;
    ret = net->emul->io_out(net->emul, offset, size, value);
    return ret;
}

static int emul_raw_tx(struct eth_driver *driver, unsigned int num, uintptr_t *phys, unsigned int *len, void *cookie)
{
    ZF_LOGF("not implemented");
}

static void emul_raw_handle_irq(struct eth_driver *driver, int irq)
{
    ZF_LOGF("not implemented");
}

static void emul_raw_poll(struct eth_driver *driver)
{
    ZF_LOGF("not implemented");
}

static void emul_low_level_init(struct eth_driver *driver, uint8_t *mac, int *mtu)
{
    ZF_LOGF("not implemented");
}

static void emul_print_state(struct eth_driver *driver)
{
    ZF_LOGF("not implemented");
}

static struct raw_iface_funcs emul_driver_funcs = {
    .raw_tx = emul_raw_tx,
    .raw_handleIRQ = emul_raw_handle_irq,
    .raw_poll = emul_raw_poll,
    .print_state = emul_print_state,
    .low_level_init = emul_low_level_init
};

static int emul_driver_init(struct eth_driver *driver, ps_io_ops_t io_ops, void *config)
{
    virtio_net_t *net = (virtio_net_t *)config;
    driver->eth_data = config;
    driver->dma_alignment = sizeof(uintptr_t);
    driver->i_fn = net->emul_driver_funcs;
    net->emul_driver = driver;
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


struct raw_iface_funcs virtio_net_default_backend()
{
    return emul_driver_funcs;
}

static vmm_pci_entry_t vmm_virtio_net_pci_bar(unsigned int iobase,
                                              size_t iobase_size_bits, unsigned int interrupt_pin,
                                              unsigned int interrupt_line, bool emulate_bar_access)
{
    vmm_pci_device_def_t *pci_config;
    int err = ps_calloc(&ops.malloc_ops, 1, sizeof(*pci_config), (void **)&pci_config);
    ZF_LOGF_IF(err, "Failed to allocate pci config");
    *pci_config = (vmm_pci_device_def_t) {
        .vendor_id = VIRTIO_PCI_VENDOR_ID,
        .device_id = VIRTIO_NET_PCI_DEVICE_ID,
        .command = PCI_COMMAND_IO | PCI_COMMAND_MEMORY,
        .header_type = PCI_HEADER_TYPE_NORMAL,
        .subsystem_vendor_id    = VIRTIO_PCI_SUBSYSTEM_VENDOR_ID,
        .subsystem_id       = VIRTIO_ID_NET,
        .interrupt_pin = interrupt_pin,
        .interrupt_line = interrupt_line,
        .bar0 = iobase | PCI_BASE_ADDRESS_SPACE_IO,
        .cache_line_size = 64,
        .latency_timer = 64,
        .prog_if = VIRTIO_PCI_CLASS_NET & 0xff,
        .subclass = (VIRTIO_PCI_CLASS_NET >> 8) & 0xff,
        .class_code = (VIRTIO_PCI_CLASS_NET >> 16) & 0xff,
    };
    vmm_pci_entry_t entry = (vmm_pci_entry_t) {
        .cookie = pci_config,
        .ioread = vmm_pci_mem_device_read,
        .iowrite = vmm_pci_mem_device_write
    };

    vmm_pci_bar_t bars[1] = {{
            .mem_type = NON_MEM,
            .address = iobase,
            .size_bits = iobase_size_bits
        }
    };
    return vmm_pci_create_bar_emulation(entry, 1, bars);
}

virtio_net_t *common_make_virtio_net(vm_t *vm, vmm_pci_space_t *pci, vmm_io_port_list_t *ioport,
                                     ioport_range_t ioport_range, ioport_type_t port_type, unsigned int interrupt_pin, unsigned int interrupt_line,
                                     struct raw_iface_funcs backend, bool emulate_bar_access)
{
    int err = ps_new_stdlib_malloc_ops(&ops.malloc_ops);
    ZF_LOGF_IF(err, "Failed to get malloc ops");

    virtio_net_t *net;
    err = ps_calloc(&ops.malloc_ops, 1, sizeof(*net), (void **)&net);
    ZF_LOGF_IF(err, "Failed to allocate virtio net");

    ioport_interface_t virtio_io_interface = {net, virtio_net_io_in, virtio_net_io_out, "VIRTIO PCI NET"};
    ioport_entry_t *io_entry = vmm_io_port_add_handler(ioport, ioport_range, virtio_io_interface, port_type);
    if (!io_entry) {
        ZF_LOGE("Failed to add vmm io port handler");
        return NULL;
    }

    size_t iobase_size_bits = BYTES_TO_SIZE_BITS(io_entry->range.size);
    net->iobase = io_entry->range.start;

    vmm_pci_entry_t entry = vmm_virtio_net_pci_bar(io_entry->range.start, iobase_size_bits, interrupt_pin, interrupt_line,
                                                   emulate_bar_access);
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

    net->emul_driver_funcs = backend;
    net->emul = virtio_emul_init(ioops, QUEUE_SIZE, vm, emul_driver_init, net, VIRTIO_NET);

    assert(net->emul);
    return net;
}
