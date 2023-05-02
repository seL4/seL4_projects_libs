/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <stdio.h>
#include <stdlib.h>

#include <sel4/sel4.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/arch/vmcs_fields.h>

#include "guest_state.h"
#include "vmcs.h"

#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
#define CS_ACCESS_RIGHTS AR(G) | AR(L) | AR(P) | AR(S) | AR_T(BUSY_TSS)
#define LDTR_ACCESS_RIGHTS AR(P) | AR_T(LDT)
#else
#define CS_ACCESS_RIGHTS AR(G) | AR(DB) | AR(P) | AR(S) | AR_T(BUSY_TSS)
#define LDTR_ACCESS_RIGHTS ACCESS_RIGHTS_UNUSABLE
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */

/*init the vmcs structure for a 32-bit guest os thread*/
static void vm_vmcs_init_32_bit_guest(vm_vcpu_t *vcpu)
{
    assert(vcpu);

    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_FS_LIMIT, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_GS_LIMIT, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_TR_LIMIT, 0x0);

    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_FS_ACCESS_RIGHTS, ACCESS_RIGHTS_UNUSABLE);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_GS_ACCESS_RIGHTS, ACCESS_RIGHTS_UNUSABLE);

    vcpu->vcpu_arch.guest_state->machine.control_ppc = VMX_CONTROL_PPC_HLT_EXITING | VMX_CONTROL_PPC_CR3_LOAD_EXITING |
                                                       VMX_CONTROL_PPC_CR3_STORE_EXITING;
}

/*init the vmcs structure for a 32-bit guest os thread*/
static void vm_vmcs_init_64_bit_guest(vm_vcpu_t *vcpu)
{
    assert(vcpu);

    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_FS_LIMIT, ~0u);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_GS_LIMIT, ~0u);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_TR_LIMIT, 0xFF);

    /* Allows guest to use FS and GS registers */
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_FS_ACCESS_RIGHTS, DEFAULT_ACCESS_RIGHTS);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_GS_ACCESS_RIGHTS, DEFAULT_ACCESS_RIGHTS);

    vcpu->vcpu_arch.guest_state->machine.control_ppc = VMX_CONTROL_PPC_HLT_EXITING;
}

/*init the vmcs structure for a guest os thread*/
void vm_vmcs_init_guest(vm_vcpu_t *vcpu)
{
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_ES_SELECTOR, 2 << 3);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_CS_SELECTOR, BIT(3));
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_SS_SELECTOR, 2 << 3);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_DS_SELECTOR, 2 << 3);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_FS_SELECTOR, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_GS_SELECTOR, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_LDTR_SELECTOR, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_TR_SELECTOR, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_ES_LIMIT, ~0u);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_CS_LIMIT, ~0u);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_SS_LIMIT, ~0u);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_DS_LIMIT, ~0u);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_LDTR_LIMIT, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_GDTR_LIMIT, 0x0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_IDTR_LIMIT, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_ES_ACCESS_RIGHTS, DEFAULT_ACCESS_RIGHTS);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_CS_ACCESS_RIGHTS, CS_ACCESS_RIGHTS);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_SS_ACCESS_RIGHTS, DEFAULT_ACCESS_RIGHTS);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_DS_ACCESS_RIGHTS, DEFAULT_ACCESS_RIGHTS);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_LDTR_ACCESS_RIGHTS, LDTR_ACCESS_RIGHTS);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_TR_ACCESS_RIGHTS, DEFAULT_TR_ACCESS_RIGHTS);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_SYSENTER_CS, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_CONTROL_CR0_MASK, vcpu->vcpu_arch.guest_state->virt.cr.cr0_mask);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_CONTROL_CR4_MASK, vcpu->vcpu_arch.guest_state->virt.cr.cr4_mask);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_CONTROL_CR0_READ_SHADOW, vcpu->vcpu_arch.guest_state->virt.cr.cr0_shadow);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_CONTROL_CR4_READ_SHADOW, vcpu->vcpu_arch.guest_state->virt.cr.cr4_shadow);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_ES_BASE, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_CS_BASE, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_SS_BASE, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_DS_BASE, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_FS_BASE, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_GS_BASE, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_LDTR_BASE, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_TR_BASE, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_GDTR_BASE, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_IDTR_BASE, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_RFLAGS, BIT(1));
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_SYSENTER_ESP, 0);
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_SYSENTER_EIP, 0);

#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
    vm_vmcs_init_64_bit_guest(vcpu);
#else
    vm_vmcs_init_32_bit_guest(vcpu);
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */

    vm_vmcs_write(vcpu->vcpu.cptr, VMX_CONTROL_PRIMARY_PROCESSOR_CONTROLS,
                  vcpu->vcpu_arch.guest_state->machine.control_ppc);
    vm_vmcs_read(vcpu->vcpu.cptr, VMX_CONTROL_ENTRY_INTERRUPTION_INFO, &vcpu->vcpu_arch.guest_state->machine.control_entry);

#ifdef CONFIG_LIB_VM_VMX_TIMER_DEBUG
    /* Enable pre-emption timer */
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_CONTROL_PIN_EXECUTION_CONTROLS, BIT(6));
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_CONTROL_EXIT_CONTROLS, BIT(22));
    vm_vmcs_write(vcpu->vcpu.cptr, VMX_GUEST_VMX_PREEMPTION_TIMER_VALUE, CONFIG_LIB_VM_VMX_TIMER_TIMEOUT);
#endif
}
