/*
 * Copyright 2021, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4vm/guest_vm.h>

#include <sel4vmmplatsupport/ioports.h>
#include <sel4vmmplatsupport/drivers/pci.h>
#include <sel4vmmplatsupport/drivers/virtio_pci_emul.h>

typedef struct virtio_blk {
    unsigned int iobase;
    virtio_emul_t *emul;
    struct disk_driver *emul_driver;
    raw_diskiface_funcs_t emul_driver_funcs;
    ps_io_ops_t ioops;
} virtio_blk_t;

virtio_blk_t *common_make_virtio_blk(vm_t *vm, vmm_pci_space_t *pci, vmm_io_port_list_t *ioport,
                                     ioport_range_t ioport_range, ioport_type_t port_type,
                                     unsigned int interrupt_pin, unsigned int interrupt_line,
                                     raw_diskiface_funcs_t backend, bool emulate_bar_access);

raw_diskiface_funcs_t virtio_blk_default_backend(void);
