/*
 * Copyright 2019, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */

#pragma once

#include <sel4vm/guest_vm.h>

static inline seL4_CPtr vm_get_vcpu_tcb(vm_vcpu_t *vcpu)
{
    return vcpu->tcb.tcb.cptr;
}

static inline seL4_CPtr vm_get_vcpu(vm_t *vm, int vcpu_id)
{
    if (vcpu_id >= vm->num_vcpus) {
        return seL4_CapNull;
    }
    return vm->vcpus[vcpu_id]->vcpu.cptr;
}

static inline vm_vcpu_t *vm_vcpu_for_target_cpu(vm_t *vm, int target_cpu)
{
    for (int i = 0; i < vm->num_vcpus; i++) {
        if (vm->vcpus[i]->target_cpu == target_cpu) {
            return vm->vcpus[i];
        }
    }
    return NULL;
}

static inline vm_vcpu_t *vm_find_free_unassigned_vcpu(vm_t *vm)
{
    for (int i = 0; i < vm->num_vcpus; i++) {
        if (vm->vcpus[i]->target_cpu == -1) {
            return vm->vcpus[i];
        }
    }
    return NULL;
}

static inline bool is_vcpu_online(vm_vcpu_t *vcpu) {
    return vcpu->vcpu_online;
}

static inline vspace_t *vm_get_vspace(vm_t *vm)
{
    return &vm->mem.vm_vspace;
}

static inline vspace_t *vm_get_vmm_vspace(vm_t *vm)
{
    return &vm->mem.vmm_vspace;
}
