/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/***
 * @module cross_vm_connection.h
 * The crossvm connection module facilitates the creation of communication channels between VM's and other
 * components on a seL4-based system. The module exports registered cross vm connections to a Linux VM such that
 * processes can access them from userlevel. This being facilitated over a virtual PCI device.
 */

#include <sel4vm/guest_vm.h>

#include <sel4vmmplatsupport/drivers/virtio.h>
#include <sel4vmmplatsupport/drivers/pci_helper.h>
#include <pci/helper.h>

typedef void (*event_callback_fn)(void *arg);
typedef void (*emit_fn)(void);
typedef int (*consume_callback_fn)(event_callback_fn, void *arg);
typedef int (*alloc_free_interrupt_fn)(void);

/***
 * @struct crossvm_dataport_handle
 * Datastructure representing a dataport of a crossvm connection
 * @param {size_t} frame_size_bits  Bit size of a single frame in the `frames` member
 * @param {int} num_frames          Total number of frames in the `frames` member
 * @param {seL4_CPtr *} frames      The set of frames backing the dataport
 */
typedef struct crossvm_dataport_handle {
    size_t frame_size_bits;
    unsigned int num_frames;
    seL4_CPtr *frames;
} crossvm_dataport_handle_t;

/***
 * @struct crossvm_handle
 * Datastructure representing a single crossvm connection
 * @param {crossvm_dataport_handle_t *} dataport    The dataport associated with the crossvm connection
 * @param {emit_fn} emit_fn                         The function pointer to the crossvm emit method
 * @param {seL4_Word} consume_id                    The identifier used for the crossvm connection when receiving incoming notifications
 *                                                  This is matched on when invoking `consume_connection_event`
 */
typedef struct crossvm_handle {
    crossvm_dataport_handle_t *dataport;
    emit_fn emit_fn;
    seL4_Word consume_id;
    const char *connection_name;
} crossvm_handle_t;

/***
 * @function cross_vm_connections_init_common(vm, connection_base_addr, connections, num_connections, pci, alloc_irq)
 * Install a set of cross vm connections into a guest VM (for either x86 or ARM VM platforms)
 * @param {vm_t *} vm                           A handle to the VM
 * @param {uintptr_t} connection_base_addr      The base guest physical address that can be used to reserve memory
 *                                              for the crossvm connectors
 * @param {crossvm_handle_t *} connections      The set of crossvm connections to be initialised and installed in the guest
 * @param {int} num_connection                  The number of connections passed in through the 'connections' parameter
 * @param {vmm_pci_space_t *} pci               A handle to the VM's host PCI device. The connections are advertised through the
 *                                              PCI device
 * @param {alloc_free_interrupt_fn} alloc_irq   A function that is used to allocated an irq number for the crossvm connections
 * @return                                      -1 on failure otherwise 0 for success
 */
int cross_vm_connections_init_common(vm_t *vm, uintptr_t connection_base_addr, crossvm_handle_t *connections,
                                     int num_connections, vmm_pci_space_t *pci, alloc_free_interrupt_fn alloc_irq);

/***
 * @function consume_connection_event(vm, event_id, inject_irq)
 * Handler to consume a cross vm connection event. This being called by the VMM when it recieves a notification from an
 * external process. The event is then relayed onto the VM.
 * @param {vm_t *} vm                   A handle to the VM
 * @param {seL4_Word} event_id          The id that corresponds to the occuring event
 * @param {bool} inject_irq             Whether to inject an interrupt into the VM
 */
void consume_connection_event(vm_t *vm, seL4_Word event_id, bool inject_irq);
