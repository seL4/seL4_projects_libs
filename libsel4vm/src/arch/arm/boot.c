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
#include <sel4/sel4.h>
#include <vka/object.h>
#include <vka/capops.h>
#include <sel4utils/mapping.h>
#include <sel4utils/api.h>

#include <sel4vm/boot.h>
#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_vm_util.h>
#include <sel4vm/arch/guest_arm_context.h>

#include "arm_vm.h"
#include "vm_boot.h"
#include "guest_vspace.h"
#include "fault.h"

int vm_init_arch(vm_t *vm)
{
    seL4_Word cspace_root_data;
    cspacepath_t src = {0};
    cspacepath_t dst = {0};
    vka_t *vka;
    int err;

    /* Check arm specific initialisation parameters */
    if (!vm) {
        ZF_LOGE("Failed to initialise vm arch: Invalid vm");
        return -1;
    }

    /* Create a cspace */
    vka = vm->vka;
    err = vka_alloc_cnode_object(vka, VM_CSPACE_SIZE_BITS, &vm->cspace.cspace_obj);
    assert(!err);
    vka_cspace_make_path(vka, vm->cspace.cspace_obj.cptr, &src);
    vm->cspace.cspace_root_data = api_make_guard_skip_word(seL4_WordBits - VM_CSPACE_SIZE_BITS);
    dst.root = vm->cspace.cspace_obj.cptr;
    dst.capPtr = VM_CSPACE_SLOT;
    dst.capDepth = VM_CSPACE_SIZE_BITS;
    err = vka_cnode_mint(&dst, &src, seL4_AllRights, vm->cspace.cspace_root_data);
    assert(!err);

    /* Create a vspace */
    err = vka_alloc_vspace_root(vka, &vm->mem.vm_vspace_root);
    assert(!err);
    err = simple_ASIDPool_assign(vm->simple, vm->mem.vm_vspace_root.cptr);
    assert(err == seL4_NoError);
    err = vm_init_guest_vspace(&vm->mem.vmm_vspace, &vm->mem.vmm_vspace, &vm->mem.vm_vspace, vm->vka,
                               vm->mem.vm_vspace_root.cptr);
    assert(!err);
    return err;
}

int vm_create_vcpu_arch(vm_t *vm, vm_vcpu_t *vcpu)
{
    int err;
    seL4_Word null_cap_data = seL4_NilData;
    cspacepath_t src = {0};
    cspacepath_t dst = {0};

    seL4_Word badge = VCPU_BADGE_CREATE((seL4_Word)vcpu->vcpu_id);

    /* Badge the endpoint */
    vka_cspace_make_path(vm->vka, vm->host_endpoint, &src);
    err = vka_cspace_alloc_path(vm->vka, &dst);
    assert(!err);
    err = vka_cnode_mint(&dst, &src, seL4_AllRights, badge);
    assert(!err);

    /* Copy it to the cspace of the VM for fault IPC */
    src = dst;
    dst.root = vm->cspace.cspace_obj.cptr;
    dst.capPtr = VM_FAULT_EP_SLOT + vcpu->vcpu_id;
    dst.capDepth = VM_CSPACE_SIZE_BITS;
    err = vka_cnode_copy(&dst, &src, seL4_AllRights);
    assert(!err);

    /* Create TCB */
    err = vka_alloc_tcb(vm->vka, &vcpu->tcb.tcb);
    assert(!err);
    err = seL4_TCB_Configure(vcpu->tcb.tcb.cptr, dst.capPtr,
                             vm->cspace.cspace_obj.cptr, vm->cspace.cspace_root_data,
                             vm->mem.vm_vspace_root.cptr, null_cap_data, 0, seL4_CapNull);
    assert(!err);
    err = seL4_TCB_SetSchedParams(vcpu->tcb.tcb.cptr, simple_get_tcb(vm->simple), vcpu->tcb.priority,
                                  vcpu->tcb.priority);
    assert(!err);
    err = seL4_ARM_VCPU_SetTCB(vcpu->vcpu.cptr, vcpu->tcb.tcb.cptr);
    assert(!err);
    vcpu->vcpu_arch.fault = fault_init(vcpu);
    assert(vcpu->vcpu_arch.fault);
    vcpu->vcpu_arch.unhandled_vcpu_callback = NULL;
    vcpu->vcpu_arch.unhandled_vcpu_callback_cookie = NULL;

#if CONFIG_MAX_NUM_NODES > 1
    if (seL4_TCB_SetAffinity(vcpu->tcb.tcb.cptr, vcpu->vcpu_id)) {
        err = -1;
    }
#endif /* CONFIG_MAX_NUM_NODES > 1 */

#ifdef CONFIG_DEBUG_BUILD
    char vcpu_name[32];
    snprintf(vcpu_name, sizeof(vcpu_name), "%s:%d", vm->vm_name, vcpu->vcpu_id);
    seL4_DebugNameThread(vm_get_vcpu_tcb(vcpu), vcpu_name);
#endif

    return err;
}
