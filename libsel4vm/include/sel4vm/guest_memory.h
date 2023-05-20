/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <sel4/sel4.h>

#include <sel4vm/arch/guest_memory_arch.h>

/***
 * @module guest_memory.h
 * The libsel4vm memory interface provides useful abstractions to manage your guest VM's address space.
 * This interface can be leveraged for uses such as mapping device memory into your VM instance
 * or for creating emulated device regions binded with custom handlers/callbacks.
 * The main mechanisms this interface provides involves reserving memory regions and using those reservations
 * to either map memory into your guest VM's address space or emulate subsequent accesses. Reservations
 * are created through either using `vm_reserve_memory_at` or `vm_reserve_anon_memory`.
 * The user can then further back the reservation with seL4 frames by performing `vm_map_reservation`.
 */

typedef struct vm vm_t;
typedef struct vm_vcpu vm_vcpu_t;

/***
 * @struct vm_frame_t
 * Structure representing a mappable memory frame
 * @param {seL4_CPtr} cptr              Capability to frame
 * @param {seL4_CapRights_t} rights     Mapping rights of frame
 * @param {uintptr_t} vaddr             Virtual address of which to map the frame into
 * @param {size_t} size_bits            Size of frame in bits
 */
typedef struct vm_frame {
    seL4_CPtr cptr; /** Capability to frame */
    seL4_CapRights_t rights; /** Mapping rights of frame */
    uintptr_t vaddr; /** Virtual address of which to map the frame into */
    size_t size_bits; /** Size of frame in bits */
} vm_frame_t;

/**
 * Enumeration of results that can be returned by a memory fault callback (type 'memory_fault_callback_fn')
 */
typedef enum memory_fault_result {
    FAULT_HANDLED, /** The memory fault was handled, advance execution */
    FAULT_UNHANDLED, /** The memory fault was left unhandled */
    FAULT_RESTART, /** The memory fault should be restarted, restart execution */
    FAULT_IGNORE, /** Ignore the memory fault, advance execution */
    FAULT_ERROR /** Handling the memory fault resulted in an error */
} memory_fault_result_t;

/**
 * Type signature of memory fault handler/callback function, provided when creating a memory reservation
 * @param {vm_t *} vm               A handle to the VM
 * @param {vm_vcpu_t} vcpu          A handle to the fault vcpu
 * @param {uintptr_t} fault addr    Faulting address
 * @param {size_t} fault_length     Length of faulted access
 * @param {void *} cookie           User cookie to pass onto callback
 * @return                          Fault handling status code: HANDLED, UNHANDLED, RESTART, ERROR
 */
typedef memory_fault_result_t (*memory_fault_callback_fn)(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                          size_t fault_length,
                                                          void *cookie);
/**
 * Type signature of memory map iterator function, provided when mapping a memory reservation
 * @param {uintptr_t} addr      Address being mapped
 * @param {void *} cookie       User cookie to pass onto iterator
 * @return                      vm_frame_t describing the memory frame that corresponds with the given address
 */
typedef vm_frame_t (*memory_map_iterator_fn)(uintptr_t addr, void *cookie);

typedef struct vm_memory_reservation vm_memory_reservation_t;
typedef struct vm_memory_reservation_cookie vm_memory_reservation_cookie_t;

/***
 * @function vm_reserve_memory_at(vm, addr, size, fault_callback, cookie)
 * Reserve a region of the VM's memory at a given base address
 * @param {vm_t *} vm                                       A handle to the VM
 * @param {uintptr} addr                                    Base address of the memory region being reserved
 * @param {size_t} size                                     Size of the memory region being reserved
 * @param {memory_fault_callback_fn} fault_callback         Callback function that will be invoked if memory region is faulted on
 * @param {void *} cookie                                   User cookie to pass onto to callback
 * @return                                                  NULL on failure otherwise a pointer to a reservation object representing the reserved region
 */
vm_memory_reservation_t *vm_reserve_memory_at(vm_t *vm, uintptr_t addr, size_t size,
                                              memory_fault_callback_fn fault_callback, void *cookie);

/***
 * @function vm_reserve_anon_memory(vm, size, align, fault_callback, cookie, addr)
 * Reserve an anonymous region of the VM's memory. This uses memory previously made anonymous
 * through the `vm_memory_make_anon` function.
 * @param {vm_t *} vm                                       A handle to the VM
 * @param {size_t} size                                     Size of the anoymous emory region being reserved
 * @param {size_t} align                                    Requested alignment of the memory region
 * @param {memory_fault_callback_fn} fault_callback         Callback function that will be invoked if memory region is faulted on
 * @param {void *} cookie                                   User cookie to pass onto to callback
 * @param {uintptr_t *} addr                                Pointer that will be set with the base address of the reserved anonymous region
 * @return                                                  NULL on failure otherwise a pointer to a reservation object representing the reserved region
 */
vm_memory_reservation_t *vm_reserve_anon_memory(vm_t *vm, size_t size, size_t align,
                                                memory_fault_callback_fn fault_callback, void *cookie, uintptr_t *addr);

/*** vm_memory_make_anon(vm, addr, size)
 * @function
 * Create an anoymous region of the VM's memory. This claims a region of VM memory that can be used for the creation
 * of anonymous reservations (achieved by calling 'vm_reserve_anon_memory').
 * @param {vm_t *} vm               A handle to the VM
 * @param {uintptr} addr            Base address of the memory region being made into an anoymous reservation
 * @param {size_t} size             Size of the memory region being reserved
 * @return                          -1 on failure otherwise 0 for success
 */
int vm_memory_make_anon(vm_t *vm, uintptr_t addr, size_t size);

/***
 * @function vm_free_reserved_memory(vm, reservation)
 * Free memory reservation from the VM
 * @param {vm_t *} vm                                   A handle to the VM
 * @param {vm_memory_reservation_t *} reservation       Pointer to the reservation being free'd
 * @return                                              -1 on failure otherwise 0 for success
 */
int vm_free_reserved_memory(vm_t *vm, vm_memory_reservation_t *reservation);

/***
 * @function vm_map_reservation(vm, reservation, map_iterator, cookie)
 * Map a reservation into the VM's virtual address space
 * @param {vm_t *} vm                                   A handle to the VM
 * @param {vm_memory_reservation_t *} reservation       Pointer to reservation object being mapped
 * @param {memory_map_iterator_fn} map_iterator         Iterator function that returns a cap to the memory region being mapped
 * @param {void *} cookie                               Cookie to pass onto map_iterator function
 * @return                                              -1 on failure otherwise 0 for success
 */
int vm_map_reservation(vm_t *vm, vm_memory_reservation_t *reservation, memory_map_iterator_fn map_iterator,
                       void *cookie);

/***
 * @function vm_map_reservation_frames(vm, reservation, frames, num_frames, frame_size_bits)
 * Map a reservation into the VM's virtual address space from given frames
 * @param {vm_t *} vm                                   A handle to the VM
 * @param {vm_memory_reservation_t *} reservation       Pointer to reservation object being mapped
 * @param {seL4_CPtr *} frames                          Array of frame caps
 * @param {size_t} num_frames                           Number of frame caps
 * @param {size_t} frame_size_bits                      Bit size of a single frame
 * @return                                              -1 on failure otherwise 0 for success
 */
int vm_map_reservation_frames(vm_t *vm, vm_memory_reservation_t *reservation,
                              seL4_CPtr *frames, size_t num_frames,
                              size_t frame_size_bits);

/***
 * @function vm_get_reservation_memory_region(reservation, addr, size)
 * Get the memory region information (address & size) from a given reservation
 * @param {vm_memory_reservation_t *} reservation           Pointer to reservation object
 * @param {uintptr_t *} addr                                Pointer that will be set with the address of reservation
 * @param {size_t *} size                                   Pointer that will be set with the size of reservation
 */
void vm_get_reservation_memory_region(vm_memory_reservation_t *reservation, uintptr_t *addr, size_t *size);

/***
 * @function vm_memory_init(vm)
 * Initialise a VM's memory management interface
 * @param {vm_t *} vm               A handle to the VM
 * @return                          -1 on failure otherwise 0 for success
*/
int vm_memory_init(vm_t *vm);
