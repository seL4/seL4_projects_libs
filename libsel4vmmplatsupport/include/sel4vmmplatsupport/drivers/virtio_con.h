/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/***
 * @module virtio_con.h
 * This interface provides the ability to initalise a VMM virtio console driver. This creating a virtio
 * PCI device in the VM's virtual pci. This can subsequently be accessed through '/dev/hvc0' in the guest.
 */

#include <sel4vm/guest_vm.h>

#include <sel4vmmplatsupport/ioports.h>
#include <sel4vmmplatsupport/drivers/pci.h>
#include <sel4vmmplatsupport/drivers/virtio_pci_emul.h>

/***
 * @struct virtio_con
 * Virtio Console Driver Interface
 * @param {unsigned int} iobase                                 IO Port base for virtio con device
 * @param {virtio_emul_t *} emul                                Virtio console emulation interface: VMM <-> Guest
 * @param {struct eth_driver *} emul_driver                     Backend console driver interface: VMM <-> console driver
 * @param {struct virtio_console_passthrough} emul_driver_funcs Virtio console emulation functions: VMM <-> Guest
 * @param {ps_io_ops_t} ioops                                   Platform support io ops datastructure
 */
typedef struct virtio_con {
    unsigned int iobase;
    virtio_emul_t *emul;
    struct virtio_console_driver *emul_driver;
    struct virtio_console_passthrough emul_driver_funcs;
    ps_io_ops_t ioops;
} virtio_con_t;

/***
 * @function common_make_virtio_con(vm, pci, ioport, ioport_range, port_type, interrupt_pin, interrupt_lin, backend)
 * Initialise a new virtio_con device with Base Address Registers (BARs) starting at iobase and backend functions
 * specified by the console_passthrough struct.
 * @param {vm_t *} vm                               Handle to the VM
 * @param {vmm_pci_space_t *} pci                   PCI library instance to register virtio con device
 * @param {vmm_io_port_list_t *} ioport             IOPort library instance to register virtio con ioport
 * @param {ioport_range_t} ioport_range             BAR port for front end emulation
 * @param {ioport_type_t} iotype                    Type of ioport i.e. whether to alloc or use given range
 * @param {unsigned int} interrupt_pin              PCI interrupt pin e.g. INTA = 1, INTB = 2 ,...
 * @param {unsigned int} interrupt_line             PCI interrupt line for virtio con IRQS
 * @param {struct console_passthrough} backend      Function pointers to backend implementation. Can be initialised by
 *                                                  virtio_con_default_backend for default methods.
 * @return                                          Pointer to an initialised virtio_con_t, NULL if error.
 */
virtio_con_t *common_make_virtio_con(vm_t *vm,
                                     vmm_pci_space_t *pci,
                                     vmm_io_port_list_t *ioport,
                                     ioport_range_t ioport_range,
                                     ioport_type_t port_type,
                                     unsigned int interrupt_pin,
                                     unsigned int interrupt_line,
                                     struct virtio_console_passthrough backend);
