/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <autoconf.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <utils/util.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/boot.h>
#include <sel4vm/guest_vm_exits.h>
#include <sel4vm/guest_vm_util.h>

#include "vm_boot.h"

static int curr_vcpu_index = 0;

int vm_init(vm_t *vm, vka_t *vka, simple_t *host_simple, vspace_t host_vspace,
            ps_io_ops_t *io_ops, seL4_CPtr host_endpoint, const char *name)
{
    int err;
    bzero(vm, sizeof(vm_t));
    /* Initialise vm fields */
    vm->vka = vka;
    vm->simple = host_simple;
    vm->io_ops = io_ops;
    vm->mem.vmm_vspace = host_vspace;
    vm->host_endpoint = host_endpoint;
    vm->vm_name = strndup(name, strlen(name));
    vm->run.exit_reason = VM_GUEST_UNKNOWN_EXIT;
    /* Initialise ram region */
    vm->mem.num_ram_regions = 0;
    vm->mem.ram_regions = malloc(0);
    assert(vm->vcpus);
    /* Initialise vm memory management interface */
    err = vm_memory_init(vm);
    if (err) {
        ZF_LOGE("Failed to initialise VM memory manager");
        return err;
    }

    /* Initialise vm architecture support */
    err = vm_init_arch(vm);
    if (err) {
        ZF_LOGE("Failed to initialise VM architecture support");
        return err;
    }

    /* Flag that the vm has been initialised */
    vm->vm_initialised = true;
    return 0;
}

vm_vcpu_t *vm_create_vcpu(vm_t *vm, int priority)
{
    int err;
    if (vm->num_vcpus >= CONFIG_MAX_NUM_NODES) {
        ZF_LOGE("Failed to create vcpu, reached maximum number of support vcpus");
        return NULL;
    }
    vm_vcpu_t *vcpu_new = calloc(1, sizeof(vm_vcpu_t));
    assert(vcpu_new);
    /* Create VCPU */
    err = vka_alloc_vcpu(vm->vka, &vcpu_new->vcpu);
    assert(!err);
    /* Initialise vcpu fields */
    vcpu_new->vm = vm;
    vcpu_new->vcpu_id = curr_vcpu_index++;
    vcpu_new->tcb.priority = priority;
    vcpu_new->vcpu_online = false;
    vcpu_new->target_cpu = -1;
    err = vm_create_vcpu_arch(vm, vcpu_new);
    assert(!err);
#ifdef CONFIG_DEBUG_BUILD
    char vcpu_name[32];
    snprintf(vcpu_name, sizeof(vcpu_name), "%s:%d", vm->vm_name, vcpu_new->vcpu_id);
    seL4_DebugNameThread(vm_get_vcpu_tcb(vcpu_new), vcpu_name);
#endif
    vm->vcpus[vm->num_vcpus] = vcpu_new;
    vm->num_vcpus++;
    return vcpu_new;
}

int vm_assign_vcpu_target(vm_vcpu_t *vcpu, int target_cpu)
{
    if (vcpu == NULL) {
        ZF_LOGE("Failed to assign target cpu - Invalid vcpu");
        return -1;
    }
    vm_vcpu_t *target_vcpu = vm_vcpu_for_target_cpu(vcpu->vm, target_cpu);
    if (target_vcpu) {
        ZF_LOGE("Failed to assign target cpu - A VCPU is already assigned to core %d", target_cpu);
        return -1;
    }
    vcpu->target_cpu = target_cpu;
    return 0;
}
