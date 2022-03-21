/*
 * Copyright 2022, UNSW
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/***
 * @module virtio_blk.h
 * This interface provides the ability to initalise a VMM virtio block driver.
 *  A virtio PCI device is created in the VM's virtual pci. 
 * This can subsequently be accessed as a block device (e.g., disc) in the
 * guest.
 */

#include <sel4vm/guest_vm.h>

#include <sel4vmmplatsupport/ioports.h>
#include <sel4vmmplatsupport/drivers/pci.h>
#include <sel4vmmplatsupport/drivers/virtio_pci_emul.h>

/***
 * @struct virtio_blk
 * Virtio Block Driver Interface
 * @param {unsigned int} iobase                         IO Port base for device
 * @param {virtio_emul_t *} emul                        Virtio Block emulation interface: VMM <-> Guest
 * @param {struct blk_driver *} emul_driver             Backend Block driver interface: VMM <-> block device driver
 * @param {struct raw_iface_funcs} emul_driver_funcs    Virtio Block emulation functions: VMM <-> Guest
 * @param {ps_io_ops_t} ioops                           Platform support ioops for dma management
 */
typedef struct virtio_blk {
    unsigned int iobase;
    virtio_emul_t *emul;
    struct eth_driver *emul_driver;
    struct raw_iface_funcs emul_driver_funcs;
    ps_io_ops_t ioops;
} virtio_blk_t;

/***
 * @function common_make_virtio_blk(vm, pci, ioport, ioport_range,
 *  port_type, interrupt_pin, interrupt_line, backend)
 * Initialise a new virtio_blk device with Base Address Registers (BARs)
 *  starting at iobase and backend functions
 * specified by the raw_iface_funcs struct.
 * @param {vm_t *} vm                       A handle to the VM
 * @param {vmm_pci_space_t *} pci           PCI library instance to register virtio net device
 * @param {vmm_io_port_list_t *} ioport     IOPort library instance to register virtio blk ioport
 * @param {ioport_range_t} ioport_range     BAR port for front end emulation
 * @param {ioport_type_t} port_type         Type of ioport i.e. whether to alloc or use given range
 * @param {unsigned int} interrupt_pin      PCI interrupt pin e.g. INTA = 1, INTB = 2 ,...
 * @param {unsigned int} interrupt_line     PCI interrupt line for virtio net IRQS
 * @param {struct raw_iface_funcs} backend  Function pointers to backend implementation. Can be initialised by
 *                                          virtio_blk_default_backend for default methods.
 * @return                                  Pointer to an initialised virtio_blk_t, NULL if error.
 */
virtio_net_t *common_make_virtio_blk(vm_t *vm, vmm_pci_space_t *pci,
                                     vmm_io_port_list_t *ioport,
                                     ioport_range_t ioport_range,
                                     ioport_type_t port_type,
                                     unsigned int interrupt_pin,
                                     unsigned int interrupt_line,
                                     struct raw_iface_funcs backend);

/***
 * @function virtio_blk_default_backend()
 * @return          A struct with a default virtio_blk backend. It is the
 *                  responsibility of the caller to
 *                  update these function pointers with its own custom backend.
 */
struct raw_iface_funcs virtio_blk_default_backend(void);
