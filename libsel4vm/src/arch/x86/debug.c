/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

/* Debugging helper functions used by VMM lib.
 *     Authors:
 *         Qian Ge
 */

#include <stdio.h>
#include <stdlib.h>

#include <sel4/sel4.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/arch/guest_x86_context.h>

#include "debug.h"
#include "guest_state.h"
#include "vmcs.h"

#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS

void vm_print_guest_context(vm_vcpu_t *vcpu)
{
    seL4_Word data_exit_info, data_exit_error;
    if (vm_vmcs_read(vcpu->vcpu.cptr, VMX_DATA_EXIT_INTERRUPT_INFO, &data_exit_info) ||
        vm_vmcs_read(vcpu->vcpu.cptr, VMX_DATA_EXIT_INTERRUPT_ERROR, &data_exit_error)) {
        return;
    }

    printf("================== GUEST OS CONTEXT =================\n");

    printf("exit info : reason 0x"SEL4_PRIx_word"    qualification 0x"SEL4_PRIx_word"   "
           "instruction len 0x"SEL4_PRIx_word" interrupt info 0x"SEL4_PRIx_word" interrupt error 0x"SEL4_PRIx_word"\n",
           vm_guest_exit_get_reason(vcpu->vcpu_arch.guest_state),
           vm_guest_exit_get_qualification(vcpu->vcpu_arch.guest_state),
           vm_guest_exit_get_int_len(vcpu->vcpu_arch.guest_state), data_exit_info, data_exit_error);
    printf("            guest physical 0x"SEL4_PRIx_word"     rflags 0x"SEL4_PRIx_word"\n",
           vm_guest_exit_get_physical(vcpu->vcpu_arch.guest_state),
           vm_guest_state_get_rflags(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr));
    printf("            guest interruptibility 0x"SEL4_PRIx_word"   control entry 0x"SEL4_PRIx_word"\n",
           vm_guest_state_get_interruptibility(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr),
           vm_guest_state_get_control_entry(vcpu->vcpu_arch.guest_state));

    printf("rip 0x"SEL4_PRIx_word"\n",
           vm_guest_state_get_eip(vcpu->vcpu_arch.guest_state));
    seL4_Word rax, rbx, rcx;
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EAX, &rax);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EBX, &rbx);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_ECX, &rcx);
    printf("rax 0x"SEL4_PRIx_word"         rbx 0x"SEL4_PRIx_word"      rcx 0x"SEL4_PRIx_word"\n", rax, rbx, rcx);
    seL4_Word rdx, rsi, rdi;
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EDX, &rdx);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_ESI, &rsi);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EDI, &rdi);
    printf("rdx 0x"SEL4_PRIx_word"         rsi 0x"SEL4_PRIx_word"      rdi 0x"SEL4_PRIx_word"\n", rdx, rsi, rdi);
    seL4_Word rbp;
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EBP, &rbp);
    printf("rbp 0x"SEL4_PRIx_word"\n", rbp);
    seL4_Word r8, r9, r10;
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_R8, &r8);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_R9, &r9);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_R10, &r10);
    printf("r8 0x"SEL4_PRIx_word"          r9 0x"SEL4_PRIx_word"       r10 0x"SEL4_PRIx_word"\n", r8, r9, r10);
    seL4_Word r11, r12, r13;
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_R11, &r11);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_R12, &r12);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_R13, &r13);
    printf("r11 0x"SEL4_PRIx_word"         r12 0x"SEL4_PRIx_word"      r13 0x"SEL4_PRIx_word"\n", r11, r12, r13);
    seL4_Word r14, r15;
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_R14, &r14);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_R15, &r15);
    printf("r14 0x"SEL4_PRIx_word"         r15 0x"SEL4_PRIx_word"\n", r14, r15);
    printf("cr0 0x"SEL4_PRIx_word"      cr3 0x"SEL4_PRIx_word"   cr4 0x"SEL4_PRIx_word"\n",
           vm_guest_state_get_cr0(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr),
           vm_guest_state_get_cr3(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr),
           vm_guest_state_get_cr4(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr));
}

#else /* not CONFIG_X86_64_VTX_64BIT_GUESTS */

/* Print out the context of a guest OS thread. */
void vm_print_guest_context(vm_vcpu_t *vcpu)
{
    seL4_Word data_exit_info, data_exit_error;
    if (vm_vmcs_read(vcpu->vcpu.cptr, VMX_DATA_EXIT_INTERRUPT_INFO, &data_exit_info) ||
        vm_vmcs_read(vcpu->vcpu.cptr, VMX_DATA_EXIT_INTERRUPT_ERROR, &data_exit_error)) {
        return;
    }
    printf("================== GUEST OS CONTEXT =================\n");

    printf("exit info : reason 0x%x    qualification 0x%x   instruction len 0x%x interrupt info 0x%x interrupt error 0x%x\n",
           vm_guest_exit_get_reason(vcpu->vcpu_arch.guest_state), vm_guest_exit_get_qualification(vcpu->vcpu_arch.guest_state),
           vm_guest_exit_get_int_len(vcpu->vcpu_arch.guest_state), data_exit_info, data_exit_error);
    printf("            guest physical 0x%x     rflags 0x%x \n",
           vm_guest_exit_get_physical(vcpu->vcpu_arch.guest_state), vm_guest_state_get_rflags(vcpu->vcpu_arch.guest_state,
                                                                                              vcpu->vcpu.cptr));
    printf("            guest interruptibility 0x%x   control entry 0x%x\n",
           vm_guest_state_get_interruptibility(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr),
           vm_guest_state_get_control_entry(vcpu->vcpu_arch.guest_state));

    printf("eip 0x%8x\n",
           vm_guest_state_get_eip(vcpu->vcpu_arch.guest_state));
    seL4_Word eax, ebx, ecx;
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EAX, &eax);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EBX, &ebx);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_ECX, &ecx);
    printf("eax 0x%8x         ebx 0x%8x      ecx 0x%8x\n", eax, ebx, ecx);
    seL4_Word edx, esi, edi;
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EDX, &edx);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_ESI, &esi);
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EDI, &edi);
    printf("edx 0x%8x         esi 0x%8x      edi 0x%8x\n", edx, esi, edi);
    seL4_Word ebp;
    vm_get_thread_context_reg(vcpu, VCPU_CONTEXT_EBP, &ebp);
    printf("ebp 0x%8x\n", ebp);

    printf("cr0 0x%x      cr3 0x%x   cr4 0x%x\n", vm_guest_state_get_cr0(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr),
           vm_guest_state_get_cr3(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr),
           vm_guest_state_get_cr4(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr));
}

#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
