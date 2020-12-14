/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/***
 * @module guest_vm_arch.h
 * The guest arm vm interface is central to using libsel4vm on an ARM platform, providing definitions of the arm guest vm
 * datastructures and primitives to configure the VM instance.
 */

#include <sel4vm/guest_vm.h>

typedef struct fault fault_t;
typedef struct vm vm_t;
typedef struct vm_vcpu vm_vcpu_t;

typedef int (*smc_handler_callback_fn)(vm_vcpu_t *vcpu, seL4_UserContext *regs);
typedef int (*unhandled_vcpu_fault_callback_fn)(vm_vcpu_t *vcpu, uint32_t hsr, void *cookie);

#define VM_CSPACE_SIZE_BITS    4
#define VM_FAULT_EP_SLOT       1
#define VM_CSPACE_SLOT         VM_FAULT_EP_SLOT + CONFIG_MAX_NUM_NODES

/***
 * @struct vm_arch
 * Structure representing ARM specific vm properties
 * @param {smc_handler_callback_fn} vm_smc_handler     A callback for a custom SMC call handler
 */
struct vm_arch {
    smc_handler_callback_fn vm_smc_handler;
};

/***
 * @struct vm_vcpu_arch
 * Structure representing ARM specific vcpu properties
 * @param {fault_t *} fault                                             Current VCPU fault
 * @param {unhandled_vcpu_fault_callback_fn} unhandled_vcpu_callback    A callback for processing unhandled vcpu faults
 * @param {void *} unhandled_vcpu_callback_cookie                       A cookie to supply to the vcpu fault handler
 */
struct vm_vcpu_arch {
    fault_t *fault;
    unhandled_vcpu_fault_callback_fn unhandled_vcpu_callback;
    void *unhandled_vcpu_callback_cookie;
};

/***
 * @function vm_register_unhandled_vcpu_fault_callback(vcpu, vcpu_fault_callback, cookie)
 * Register a callback for processing unhandled vcpu faults
 * @param {vm_vcpu_t *} vcpu                    A handle to the VCPU
 * @param {unhandled_vcpu_fault_callback_fn}    A user supplied callback to process unhandled vcpu faults
 * @param {void *}                              A cookie to supply to the vcpu fault handler
 * @return                                      0 on success, -1 on error
 */
int vm_register_unhandled_vcpu_fault_callback(vm_vcpu_t *vcpu, unhandled_vcpu_fault_callback_fn vcpu_fault_callback,
                                              void *cookie);

/***
 * @function vm_register_smc_handler_callback(vm, vm_smc_handler)
 * Register a callback for a user level SMC handler to replace vm_smc_handle_default.
 * This is useful when custom behavior, such as SMC emulation or forwarding device
 * specific SMC calls to the Secure Monitor is needed.
 *
 * If default behavior is still required after custom processing the default handler can
 * still be called from the callback.
 *
 * To forward an SMC call to the Secure Monitor, use smc_forward.
 *
 * On error return the guest will fault. Return 0 and use smc_set_return_value to
 * return -1 to the VM without stopping it.
 *
 * @param {vm_t *} vm                  A handle to the VM
 * @param {smc_handler_callback_fn}    A user supplied callback to handle VM SMC calls
 * @return                             0 on success, -1 on error
 */
int vm_register_smc_handler_callback(vm_t *vm, smc_handler_callback_fn vm_smc_handler);
