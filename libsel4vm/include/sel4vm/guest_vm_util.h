/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/***
 * @module guest_vm_util.h
 * The libsel4vm VM util interface provides a set of useful methods to query a guest vm instance.
 */

#include <sel4vm/guest_vm.h>

/***
 * @function vm_get_vcpu_tcb(vcpu)
 * Get the TCB CPtr a given VCPU is associated with
 * @param {vm_vcpu_t} vcpu  A handle to the vcpu
 * @return                  seL4_CPtr of TCB object
 */
static inline seL4_CPtr vm_get_vcpu_tcb(vm_vcpu_t *vcpu)
{
    return vcpu->tcb.tcb.cptr;
}

/***
 * @function vm_get_vcpu(vm, vcpu_id)
 * Get the VCPU CPtr associatated with a given logical ID
 * @param {vm_t *} vm       A handle to the vm owning the vcpu
 * @param {int} vcpu_id     Logical ID of the vcpu
 * @return                  seL4_CapNull if no vcpu exists, otherwise the seL4_CPtr of the VCPU object
 */
static inline seL4_CPtr vm_get_vcpu(vm_t *vm, int vcpu_id)
{
    if (vcpu_id >= vm->num_vcpus) {
        return seL4_CapNull;
    }
    return vm->vcpus[vcpu_id]->vcpu.cptr;
}

/***
 * @function vm_vcpu_for_target_cpu(vm, target_cpu)
 * Get the VCPU object that is assigned to a given target core ID
 * @param {vm_t *} vm           A handle to the vm owning the vcpu
 * @param {int} target_cpu      Target core ID
 * @return                      NULL if no vcpu is assigned to the target core, otherwise the vm_vcpu_t object
 */
static inline vm_vcpu_t *vm_vcpu_for_target_cpu(vm_t *vm, int target_cpu)
{
    for (int i = 0; i < vm->num_vcpus; i++) {
        if (vm->vcpus[i]->target_cpu == target_cpu) {
            return vm->vcpus[i];
        }
    }
    return NULL;
}

/***
 * @function vm_find_free_unassigned_vcpu(vm)
 * Find a VCPU object that hasn't been assigned to a target core
 * @param {vm_t *} vm           A handle to the vm owning the vcpu
 * @return                      NULL if no vcpu can be found, otherwise the vm_vcpu_t object
 */
static inline vm_vcpu_t *vm_find_free_unassigned_vcpu(vm_t *vm)
{
    for (int i = 0; i < vm->num_vcpus; i++) {
        if (vm->vcpus[i]->target_cpu == -1) {
            return vm->vcpus[i];
        }
    }
    return NULL;
}

/***
 * @function vm_find_free_unassigned_cpu(vm)
 * Find an unallocated physical cpu
 * @param {vm_t *} vm           A handle to the vm owning the vcpu
 * @return                      -1 if no pcpu can be found, otherwise the available pcpu
 */
static inline int vm_find_free_unassigned_cpu(vm_t *vm)
{
    for (int core = 0; core < CONFIG_MAX_NUM_NODES; core++) {
        int core_allocated = 0;
        for (int i = 0; i < vm->num_vcpus; i++) {
            if (vm->vcpus[i]->target_cpu == core) {
                core_allocated = 1;
                break;
            }
        }
        if (!core_allocated) {
            return core;
        }
    }
    return -1;
}

/***
 * @function is_vcpu_online(vcpu)
 * Find if a given VCPU is online
 * @param {vm_vcpu_t *} vcpu    A handle to the vcpu
 * @return                      True if the vcpu is online, otherwise False
 */
static inline bool is_vcpu_online(vm_vcpu_t *vcpu)
{
    return vcpu->vcpu_online;
}

/***
 * @function vm_get_vspace(vm)
 * Get the vspace of a given VM instance
 * @param {vm_t *} vm       A handle to the VM
 * @return                  A vspace_t object
 */
static inline vspace_t *vm_get_vspace(vm_t *vm)
{
    return &vm->mem.vm_vspace;
}

/***
 * @function vm_get_vmm_vspace(vm)
 * Get the vspace of the host VMM associated with a given VM instance
 * @param {vm_t *} vm       A handle to the VM
 * @return                  A vspace_t object
 */
static inline vspace_t *vm_get_vmm_vspace(vm_t *vm)
{
    return &vm->mem.vmm_vspace;
}
