/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <sel4vm/guest_vm.h>
#include <sel4vm/arch/guest_x86_context.h>
#include <sel4vm/arch/vmcall.h>

#include "vm.h"
#include "guest_state.h"
#include "vmexit.h"

static vmcall_handler_t *get_handle(vm_t *vm, int token);

static vmcall_handler_t *get_handle(vm_t *vm, int token)
{
    int i;
    for (i = 0; i < vm->arch.vmcall_num_handlers; i++) {
        if (vm->arch.vmcall_handlers[i].token == token) {
            return &vm->arch.vmcall_handlers[i];
        }
    }

    return NULL;
}

int vm_reg_new_vmcall_handler(vm_t *vm, vmcall_handler func, int token)
{
    unsigned int *hnum = &(vm->arch.vmcall_num_handlers);
    if (get_handle(vm, token) != NULL) {
        return -1;
    }

    vm->arch.vmcall_handlers = realloc(vm->arch.vmcall_handlers, sizeof(vmcall_handler_t) * (*hnum + 1));
    if (vm->arch.vmcall_handlers == NULL) {
        return -1;
    }

    vm->arch.vmcall_handlers[*hnum].func = func;
    vm->arch.vmcall_handlers[*hnum].token = token;
    vm->arch.vmcall_num_handlers++;

    ZF_LOGD("Reg. handler %u for vm, total = %u\n", *hnum - 1, *hnum);
    return 0;
}

int vm_vmcall_handler(vm_vcpu_t *vcpu)
{
    int res;
    vmcall_handler_t *h;
    seL4_Word token;
    if (vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EAX, &token)) {
        return VM_EXIT_HANDLE_ERROR;
    }
    h = get_handle(vcpu->vm, (int) token);
    if (h == NULL) {
        ZF_LOGE("Failed to find handler for token:%lx\n", token);
        vm_guest_exit_next_instruction(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        return VM_EXIT_HANDLED;
    }

    res = h->func(vcpu);
    if (res == 0) {
        vm_guest_exit_next_instruction(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        return VM_EXIT_HANDLED;
    }

    return VM_EXIT_HANDLE_ERROR;
}
