/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4/sel4.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/arch/guest_x86_context.h>

#include "guest_state.h"
#include "vmcs.h"

int vm_set_thread_context(vm_vcpu_t *vcpu, seL4_VCPUContext context)
{
    MACHINE_STATE_DIRTY(vcpu->vcpu_arch.guest_state->machine.context);
    vcpu->vcpu_arch.guest_state->machine.context = context;
    return 0;
}

int vm_set_thread_context_reg(vm_vcpu_t *vcpu, vcpu_context_reg_t reg, seL4_Word value)
{
    MACHINE_STATE_DIRTY(vcpu->vcpu_arch.guest_state->machine.context);
    (&vcpu->vcpu_arch.guest_state->machine.context.eax)[reg] = value;
    return 0;
}

int vm_get_thread_context(vm_vcpu_t *vcpu, seL4_VCPUContext *context)
{
    if (IS_MACHINE_STATE_UNKNOWN(vcpu->vcpu_arch.guest_state->machine.context)) {
        ZF_LOGE("Failed to get thread context: Context is unsynchronised. The VCPU hasn't exited?");
        return -1;
    }
    *context = vcpu->vcpu_arch.guest_state->machine.context;
    return 0;
}

int vm_get_thread_context_reg(vm_vcpu_t *vcpu, unsigned int reg, seL4_Word *value)
{
    if (IS_MACHINE_STATE_UNKNOWN(vcpu->vcpu_arch.guest_state->machine.context)) {
        ZF_LOGE("Failed to get thread context register: Context is unsynchronised. The VCPU hasn't exited?");
        return -1;
    }
    *value = (&vcpu->vcpu_arch.guest_state->machine.context.eax)[reg];
    return 0;
}

int vm_set_vmcs_field(vm_vcpu_t *vcpu, seL4_Word field, seL4_Word value)
{
    int err = 0;
    switch (field) {
    case VMX_GUEST_CR0:
        vm_guest_state_set_cr0(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_GUEST_CR3:
        vm_guest_state_set_cr3(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_GUEST_CR4:
        vm_guest_state_set_cr4(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_GUEST_RFLAGS:
        vm_guest_state_set_rflags(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_GUEST_IDTR_BASE:
        vm_guest_state_set_idt_base(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_GUEST_IDTR_LIMIT:
        vm_guest_state_set_idt_limit(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_GUEST_GDTR_BASE:
        vm_guest_state_set_gdt_base(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_GUEST_GDTR_LIMIT:
        vm_guest_state_set_gdt_limit(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_GUEST_CS_SELECTOR:
        vm_guest_state_set_cs_selector(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_GUEST_RIP:
        vm_guest_state_set_eip(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_CONTROL_PRIMARY_PROCESSOR_CONTROLS:
        vm_guest_state_set_control_ppc(vcpu->vcpu_arch.guest_state, value);
        break;
    case VMX_CONTROL_ENTRY_INTERRUPTION_INFO:
        vm_guest_state_set_control_entry(vcpu->vcpu_arch.guest_state, value);
        break;
    default:
        /* Write through to VMCS */
        err = vm_vmcs_write(vcpu->vcpu.cptr, field, value);
    }
    return err;
}

int vm_get_vmcs_field(vm_vcpu_t *vcpu, seL4_Word field, seL4_Word *value)
{
    int err = 0;
    seL4_Word val;
    switch (field) {
    case VMX_GUEST_CR0:
        val = vm_guest_state_get_cr0(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        break;
    case VMX_GUEST_CR3:
        val = vm_guest_state_get_cr3(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        break;
    case VMX_GUEST_CR4:
        val = vm_guest_state_get_cr4(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        break;
    case VMX_GUEST_RFLAGS:
        val = vm_guest_state_get_rflags(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        break;
    case VMX_GUEST_IDTR_BASE:
        val = vm_guest_state_get_idt_base(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        break;
    case VMX_GUEST_IDTR_LIMIT:
        val = vm_guest_state_get_idt_limit(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        break;
    case VMX_GUEST_GDTR_BASE:
        val = vm_guest_state_get_gdt_base(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        break;
    case VMX_GUEST_GDTR_LIMIT:
        val = vm_guest_state_get_gdt_limit(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        break;
    case VMX_GUEST_CS_SELECTOR:
        val = vm_guest_state_get_cs_selector(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        break;
    case VMX_GUEST_RIP:
        val = vm_guest_state_get_eip(vcpu->vcpu_arch.guest_state);
        break;
    case VMX_CONTROL_PRIMARY_PROCESSOR_CONTROLS:
        val = vm_guest_state_get_control_ppc(vcpu->vcpu_arch.guest_state);
        break;
    case VMX_CONTROL_ENTRY_INTERRUPTION_INFO:
        val = vm_guest_state_get_control_entry(vcpu->vcpu_arch.guest_state);
        break;
    default:
        /* Write through to VMCS */
        err = vm_vmcs_read(vcpu->vcpu.cptr, field, &val);
    }
    *value = val;
    return err;
}

int vm_vmcs_read(seL4_CPtr vcpu, seL4_Word field, seL4_Word *value)
{

    seL4_X86_VCPU_ReadVMCS_t UNUSED result;

    if (!vcpu) {
        return -1;
    }

    result = seL4_X86_VCPU_ReadVMCS(vcpu, field);
    if (result.error != seL4_NoError) {
        return -1;
    }
    *value = result.value;
    return 0;
}

/*write a field and its value into the VMCS*/
int vm_vmcs_write(seL4_CPtr vcpu, seL4_Word field, seL4_Word value)
{

    seL4_X86_VCPU_WriteVMCS_t UNUSED result;
    if (!vcpu) {
        return -1;
    }

    result = seL4_X86_VCPU_WriteVMCS(vcpu, field, value);
    if (result.error != seL4_NoError) {
        return -1;
    }
    return 0;
}

int vm_sync_guest_context(vm_vcpu_t *vcpu)
{
    if (IS_MACHINE_STATE_MODIFIED(vcpu->vcpu_arch.guest_state->machine.context)) {
        seL4_VCPUContext context;
        int err = vm_get_thread_context(vcpu, &context);
        if (err) {
            ZF_LOGE("Failed to sync guest context");
            return -1;
        }
        seL4_X86_VCPU_WriteRegisters(vcpu->vcpu.cptr, &context);
        /* Sync our context */
        MACHINE_STATE_SYNC(vcpu->vcpu_arch.guest_state->machine.context);
    }
    return 0;
}
