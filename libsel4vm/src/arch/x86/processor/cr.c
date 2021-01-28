/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

/*vm exit handler for control register access*/

#include <stdio.h>
#include <stdlib.h>

#include <sel4/sel4.h>
#include <utils/util.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/arch/guest_x86_context.h>
#include <sel4vm/arch/vmcs_fields.h>

#include "vm.h"
#include "guest_state.h"
#include "vmcs.h"
#include "processor/platfeature.h"

static inline seL4_Word apply_cr_bits(seL4_Word cr, seL4_Word mask, seL4_Word host_bits)
{
    /* force any bit in the mask to be the value from the shadow (both enabled and disabled) */
    cr |= (mask & host_bits);
    cr &= ~(mask & (~host_bits));
    return cr;
}

static int vm_cr_set_cr0(vm_vcpu_t *vcpu, seL4_Word value)
{
    int err;
    if (value & CR0_RESERVED_BITS) {
        return -1;
    }

    /* check if paging is being enabled */
    if ((value & X86_CR0_PG) && !(vcpu->vcpu_arch.guest_state->virt.cr.cr0_shadow & X86_CR0_PG)) {
        /* guest is taking over paging. So we can no longer care about some of our CR4 values, and
         * we don't need cr3 load/store exiting anymore */
        seL4_Word new_mask = vcpu->vcpu_arch.guest_state->virt.cr.cr4_mask & ~(X86_CR4_PSE | X86_CR4_PAE);
        seL4_Word cr4_value = vm_guest_state_get_cr4(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        /* for any bits that have changed in the mask, grab them from the shadow */
        cr4_value = apply_cr_bits(cr4_value, new_mask ^ vcpu->vcpu_arch.guest_state->virt.cr.cr4_mask,
                                  vcpu->vcpu_arch.guest_state->virt.cr.cr4_shadow);
        /* update mask and cr4 value */
        vcpu->vcpu_arch.guest_state->virt.cr.cr4_mask = new_mask;
        err = vm_vmcs_write(vcpu->vcpu.cptr, VMX_CONTROL_CR4_MASK, new_mask);
        if (err) {
            return -1;
        }
        vm_guest_state_set_cr4(vcpu->vcpu_arch.guest_state, cr4_value);
        /* now turn of cr3 load/store exiting */
        seL4_Word ppc = vm_guest_state_get_control_ppc(vcpu->vcpu_arch.guest_state);
        ppc &= ~(VMX_CONTROL_PPC_CR3_LOAD_EXITING | VMX_CONTROL_PPC_CR3_STORE_EXITING);
        vm_guest_state_set_control_ppc(vcpu->vcpu_arch.guest_state, ppc);
        /* load the cached cr3 value */
        vm_guest_state_set_cr3(vcpu->vcpu_arch.guest_state, vcpu->vcpu_arch.guest_state->virt.cr.cr3_guest);
    }

    /* check if paging is being disabled */
    if (!(value & X86_CR0_PG) && (vcpu->vcpu_arch.guest_state->virt.cr.cr0_shadow & X86_CR0_PG)) {
        ZF_LOGE("Do not support paging being disabled");
        /* if we did support disabling paging we should re-enable cr3 load/store exiting,
         * watch the pae bit in cr4 again and then */
        return -1;
    }

    /* update the guest shadow */
    vcpu->vcpu_arch.guest_state->virt.cr.cr0_shadow = value;
    err = vm_vmcs_write(vcpu->vcpu.cptr, VMX_CONTROL_CR0_READ_SHADOW, value);
    if (err) {
        return -1;
    }
    value = apply_cr_bits(value, vcpu->vcpu_arch.guest_state->virt.cr.cr0_mask,
                          vcpu->vcpu_arch.guest_state->virt.cr.cr0_host_bits);

    vm_guest_state_set_cr0(vcpu->vcpu_arch.guest_state, value);

    return 0;
}

static int vm_cr_set_cr3(vm_vcpu_t *vcpu, seL4_Word value)
{
    /* if the guest hasn't turned on paging then just cache this */
    vcpu->vcpu_arch.guest_state->virt.cr.cr3_guest = value;
    if (vcpu->vcpu_arch.guest_state->virt.cr.cr0_shadow & X86_CR0_PG) {
        vm_guest_state_set_cr3(vcpu->vcpu_arch.guest_state, value);
    }
    return 0;
}

static int vm_cr_get_cr3(vm_vcpu_t *vcpu, seL4_Word *value)
{
    if (vcpu->vcpu_arch.guest_state->virt.cr.cr0_shadow & X86_CR0_PG) {
        *value = vm_guest_state_get_cr3(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
    } else {
        *value = vcpu->vcpu_arch.guest_state->virt.cr.cr3_guest;
    }
    return 0;

}

static int vm_cr_set_cr4(vm_vcpu_t *vcpu, seL4_Word value)
{

    if (value & CR4_RESERVED_BITS) {
        return -1;
    }

    /* update the guest shadow */
    vcpu->vcpu_arch.guest_state->virt.cr.cr4_shadow = value;
    int err = vm_vmcs_write(vcpu->vcpu.cptr, VMX_CONTROL_CR4_READ_SHADOW, value);
    if (err) {
        return -1;
    }

    value = apply_cr_bits(value, vcpu->vcpu_arch.guest_state->virt.cr.cr4_mask,
                          vcpu->vcpu_arch.guest_state->virt.cr.cr4_host_bits);

    vm_guest_state_set_cr4(vcpu->vcpu_arch.guest_state, value);

    return 0;
}

static int vm_cr_clts(vm_vcpu_t *vcpu)
{
    ZF_LOGI("Ignoring call of clts");

    return -1;
}

static int vm_cr_lmsw(vm_vcpu_t *vcpu, seL4_Word value)
{
    ZF_LOGI("Ignoring call of lmsw");

    return -1;

}

/* Convert exit regs to seL4 user context */
static int crExitRegs[] = {
    VCPU_CONTEXT_EAX,
    VCPU_CONTEXT_ECX,
    VCPU_CONTEXT_EDX,
    VCPU_CONTEXT_EBX,
    /*VCPU_CONTEXT_ESP*/ -1,
    VCPU_CONTEXT_EBP,
    VCPU_CONTEXT_ESI,
    VCPU_CONTEXT_EDI,
#ifdef CONFIG_ARCH_X86_64
    VCPU_CONTEXT_R8,
    VCPU_CONTEXT_R9,
    VCPU_CONTEXT_R10,
    VCPU_CONTEXT_R11,
    VCPU_CONTEXT_R12,
    VCPU_CONTEXT_R13,
    VCPU_CONTEXT_R14,
    VCPU_CONTEXT_R15,
#endif
};

int vm_cr_access_handler(vm_vcpu_t *vcpu)
{

    seL4_Word exit_qualification, val;
    int cr, reg, ret = -1;

    exit_qualification = vm_guest_exit_get_qualification(vcpu->vcpu_arch.guest_state);
    cr = exit_qualification & 15;
    reg = (exit_qualification >> 8) & 15;

    switch ((exit_qualification >> 4) & 3) {
    case 0: /* mov to cr */
        if (crExitRegs[reg] < 0) {
            return -1;
        }
        vm_get_thread_context_reg(vcpu, crExitRegs[reg], &val);

        switch (cr) {
        case 0:
            ret = vm_cr_set_cr0(vcpu, val);
            break;
        case 3:
            ret = vm_cr_set_cr3(vcpu, val);
            break;
        case 4:
            ret = vm_cr_set_cr4(vcpu, val);
            break;
        case 8:

#if 0

            u8 cr8_prev = kvm_get_cr8(vcpu);
            u8 cr8 = kvm_register_read(vcpu, reg);
            err = kvm_set_cr8(vcpu, cr8);
            kvm_complete_insn_gp(vcpu, err);
            if (irqchip_in_kernel(vcpu->kvm)) {
                return 1;
            }
            if (cr8_prev <= cr8) {
                return 1;
            }
            vcpu->run->exit_reason = KVM_EXIT_SET_TPR;
            return 0;
#endif

        default:
            ZF_LOGD("unhandled control register: op %d cr %d\n",
                    (int)(exit_qualification >> 4) & 3, cr);
            break;

        }
        break;

    case 1: /*mov from cr*/
        switch (cr) {
        case 3:

            ret = vm_cr_get_cr3(vcpu, &val);
            if (!ret) {
                ret = vm_set_thread_context_reg(vcpu, crExitRegs[reg], val);
            }

            break;
        case 8:
#if 0
            val = kvm_get_cr8(vcpu);
            kvm_register_write(vcpu, reg, val);
            trace_kvm_cr_read(cr, val);
            skip_emulated_instruction(vcpu);
            return 1;
#endif
        default:
            ZF_LOGD("unhandled control register: op %d cr %d\n",
                    (int)(exit_qualification >> 4) & 3, cr);
            break;

        }
        break;
    case 2: /* clts */
        ret = vm_cr_clts(vcpu);
        break;

    case 3: /* lmsw */
        val = (exit_qualification >> LMSW_SOURCE_DATA_SHIFT) & 0x0f;
        ret = vm_cr_lmsw(vcpu, val);
        break;

    default:
        ZF_LOGD("unhandled control register: op %d cr %d\n",
                (int)(exit_qualification >> 4) & 3, cr);
        break;
    }

    if (!ret) {
        vm_guest_exit_next_instruction(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
        return VM_EXIT_HANDLED;
    }

    return VM_EXIT_HANDLE_ERROR;
}
