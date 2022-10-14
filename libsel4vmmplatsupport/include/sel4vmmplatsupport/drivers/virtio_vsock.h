/*
 * Copyright 2022, UNSW (ABN 57 195 873 179)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <sel4vm/guest_vm.h>

#include <sel4vmmplatsupport/ioports.h>
#include <sel4vmmplatsupport/drivers/pci.h>
#include <sel4vmmplatsupport/drivers/virtio_pci_emul.h>

typedef struct virtio_vsock {
    unsigned int iobase;
    virtio_emul_t *emul;
    struct vsock_passthrough emul_driver_funcs;
    ps_io_ops_t ioops;
} virtio_vsock_t;

virtio_vsock_t *common_make_virtio_vsock(vm_t *vm,
                                         vmm_pci_space_t *pci,
                                         vmm_io_port_list_t *ioport,
                                         ioport_range_t ioport_range,
                                         ioport_type_t port_type,
                                         unsigned int interrupt_pin,
                                         unsigned int interrupt_line,
                                         struct vsock_passthrough backend);
