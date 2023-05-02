/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

/*handling msr read & write exceptions*/

#include <stdio.h>
#include <stdlib.h>

#include <sel4/sel4.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/arch/guest_x86_context.h>

#include "vm.h"
#include "guest_state.h"
#include "processor/msr.h"
#include "processor/lapic.h"
#include "interrupt.h"

#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
static seL4_Word vm_msr_read(seL4_CPtr vcpu, unsigned int field)
{

    seL4_X86_VCPU_ReadMSR_t result;

    assert(vcpu);

    result = seL4_X86_VCPU_ReadMSR(vcpu, (seL4_Word)field);
    assert(result.error == seL4_NoError);

    return result.value;
}

static void vm_msr_write(seL4_CPtr vcpu, unsigned int field, seL4_Word value)
{

    seL4_X86_VCPU_WriteMSR_t result;

    assert(vcpu);

    result = seL4_X86_VCPU_WriteMSR(vcpu, (seL4_Word)field, value);
    ZF_LOGF_IF(result.error != seL4_NoError, "MSR writing failed, error number: %d", result.error);
}
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */

int vm_rdmsr_handler(vm_vcpu_t *vcpu)
{

    int ret = 0;
    seL4_Word msr_no;
    if (vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_ECX, &msr_no)) {
        return VM_EXIT_HANDLE_ERROR;
    }
    uint64_t data = 0;
    seL4_Word vm_data;

    ZF_LOGD("rdmsr ecx 0x"SEL4_PRIx_word"\n", msr_no);

    // src reference: Linux kernel 3.11 kvm arch/x86/kvm/x86.c
    switch (msr_no) {
    case MSR_IA32_PLATFORM_ID:
    case MSR_IA32_EBL_CR_POWERON:
    case MSR_IA32_DEBUGCTLMSR:
    case MSR_IA32_LASTBRANCHFROMIP:
    case MSR_IA32_LASTBRANCHTOIP:
    case MSR_IA32_LASTINTFROMIP:
    case MSR_IA32_LASTINTTOIP:
    case MSR_IA32_MISC_ENABLE:
        data = 0;
        break;

    case MSR_IA32_UCODE_REV:
        data = 0x100000000ULL;
        break;

    case MSR_P6_PERFCTR0:
    case MSR_P6_PERFCTR1:
    case MSR_P6_EVNTSEL0:
    case MSR_P6_EVNTSEL1:
    case MSR_IA32_PERF_GLOBAL_STATUS_SET:
        /* performance counters not supported. */
        data = 0;
        break;

    case 0xcd: /* fsb frequency */
        data = 3;
        break;

    case MSR_EBC_FREQUENCY_ID:
        data = 1 << 24;
        break;

    case MSR_IA32_APICBASE:
        data = vm_lapic_get_base_msr(vcpu);
        break;

#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
    case MSR_EFER:
        vm_get_vmcs_field(vcpu, VMX_GUEST_EFER, &vm_data);
        data = (uint64_t) vm_data;
        break;

    case MSR_STAR:
    case MSR_LSTAR:
    case MSR_CSTAR:
    case MSR_SYSCALL_MASK:
        vm_data = vm_msr_read(vcpu->vcpu.cptr, msr_no);
        data = (uint64_t) vm_data;
        break;
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */

    default:
        ZF_LOGW("rdmsr WARNING unsupported msr_no 0x%x\n", msr_no);
        // generate a GP fault
        vm_inject_exception(vcpu, 13, 1, 0);
        return VM_EXIT_HANDLED;

    }

    if (!ret) {
        vm_set_thread_context_reg(vcpu, VCPU_CONTEXT_EAX, (uint32_t)(data & 0xffffffff));
        vm_set_thread_context_reg(vcpu, VCPU_CONTEXT_EDX, (uint32_t)(data >> 32));
        vm_guest_exit_next_instruction(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        return VM_EXIT_HANDLED;
    }

    return VM_EXIT_HANDLE_ERROR;
}

int vm_wrmsr_handler(vm_vcpu_t *vcpu)
{

    int ret = 0;

    seL4_Word msr_no;
    if (vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_ECX, &msr_no)) {
        return VM_EXIT_HANDLE_ERROR;
    }
    seL4_Word val_high, val_low;

    if (vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EDX, &val_high)
        || vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EAX, &val_low)) {
        return VM_EXIT_HANDLE_ERROR;
    }

    ZF_LOGD("wrmsr ecx 0x%x   value: 0x%x  0x%x\n", msr_no, val_high, val_low);

    // src reference: Linux kernel 3.11 kvm arch/x86/kvm/x86.c
    switch (msr_no) {
    case MSR_IA32_UCODE_REV:
    case MSR_IA32_UCODE_WRITE:
        break;

    case MSR_P6_PERFCTR0:
    case MSR_P6_PERFCTR1:
    case MSR_P6_EVNTSEL0:
    case MSR_P6_EVNTSEL1:
    case MSR_IA32_PERF_GLOBAL_STATUS_SET:
        /* performance counters not supported. */
        break;

    case MSR_IA32_APICBASE:
        vm_lapic_set_base_msr(vcpu, val_low);
        break;

#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
    case MSR_EFER:
        vm_set_vmcs_field(vcpu, VMX_GUEST_EFER, val_low);
        break;

    case MSR_STAR:
    case MSR_LSTAR:
    case MSR_CSTAR:
    case MSR_SYSCALL_MASK:
        vm_msr_write(vcpu->vcpu.cptr, msr_no, (seL4_Word)(((seL4_Word)val_high << 32ull) | val_low));
        break;
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */

    default:
        ZF_LOGW("wrmsr WARNING unsupported msr_no 0x%x\n", msr_no);
        // generate a GP fault
        vm_inject_exception(vcpu, 13, 1, 0);
        return VM_EXIT_HANDLED;
    }

    vm_guest_exit_next_instruction(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
    return VM_EXIT_HANDLED;

}
