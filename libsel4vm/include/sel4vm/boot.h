/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <simple/simple.h>
#include <vka/vka.h>
#include <vspace/vspace.h>

#include <sel4vm/guest_vm.h>

/***
 * @module boot.h
 * The libsel4vm boot interface provides us with base abstractions to create, initialise and configure VM and VCPU instances.
 */

/**
 * ID of the boot vcpu in a VM
 */
#define BOOT_VCPU 0

/***
 * @function vm_init(vm, vka, host_simple, host_vspace, io_ops, host_endpoint, name)
 * Initialise/Create VM
 * @param {vm_t *} vm                   Handle to the VM being initialised
 * @param {vka_t *} vka                 Initialised handle to virtual kernel allocator for seL4 kernel object allocation
 * @param {simple_t *} host_simple      Initialised handle to hosts simple environment
 * @param {vspace_t} host_vspace        Initialised handle to hosts vspace
 * @param {ps_io_ops_t *} ps_io_ops     Initialised handle to platforms io ops
 * @param {seL4_CPtr} host_enpoint      Host's endpoint. The library will wait and manage the endpoint when running a VM instance
 * @param {const char *} name           String used to describe VM. Useful for debugging
 * @return                              0 on success, otherwise -1 for error
 */
#ifdef CONFIG_KERNEL_MCS
int vm_init(vm_t *vm, vka_t *vka, simple_t *host_simple, vspace_t host_vspace,
            ps_io_ops_t *io_ops, seL4_CPtr host_endpoint, seL4_CPtr host_reply, const char *name);
#else
int vm_init(vm_t *vm, vka_t *vka, simple_t *host_simple, vspace_t host_vspace,
            ps_io_ops_t *io_ops, seL4_CPtr host_endpoint, const char *name);
#endif

/***
 * @function vm_create_vcpu(vm, priority)
 * Create a VCPU for a given VM
 * @param {vm_t *} vm       A handle to VM being configured with a new vcpu
 * @param {int} priority    The scheduling priority assigned to the VCPU thread
 * @return                  NULL for error, otherwise pointer to created vm_vcpu_t object
 */
vm_vcpu_t *vm_create_vcpu(vm_t *vm, int priority);

/***
 * @function vm_assign_vcpu_target(vcpu, target_cpu)
 * Assign a vcpu with logical target cpu to run on
 * @param {vm_vcpu_t *} vcpu    A handle to the VCPU
 * @param {int} target          Logical target CPU ID
 * @return                      -1 for error, otherwise 0 for success
 */
int vm_assign_vcpu_target(vm_vcpu_t *vcpu, int target_cpu);
