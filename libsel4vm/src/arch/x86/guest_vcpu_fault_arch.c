/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vm/guest_vcpu_fault.h>

#include <sel4vm/arch/vmcs_fields.h>

#include "guest_state.h"
#include "processor/decode.h"

seL4_Word get_vcpu_fault_address(vm_vcpu_t *vcpu)
{
    return vm_guest_exit_get_physical(vcpu->vcpu_arch.guest_state);
}

seL4_Word get_vcpu_fault_ip(vm_vcpu_t *vcpu)
{
    return vm_guest_state_get_eip(vcpu->vcpu_arch.guest_state);
}

seL4_Word get_vcpu_fault_data(vm_vcpu_t *vcpu)
{
    int reg;
    seL4_Word imm;
    int size;
    vm_decode_ept_violation(vcpu, &reg, &imm, &size);
    int vcpu_reg = vm_decoder_reg_mapw[reg];
    seL4_Word data;
    vm_get_thread_context_reg(vcpu, vcpu_reg, &data);
    return data;
}

size_t get_vcpu_fault_size(vm_vcpu_t *vcpu)
{
    int reg;
    seL4_Word imm;
    int size;
    vm_decode_ept_violation(vcpu, &reg, &imm, &size);
    return size;
}

seL4_Word get_vcpu_fault_data_mask(vm_vcpu_t *vcpu)
{
    size_t size = get_vcpu_fault_size(vcpu);
    seL4_Word addr = get_vcpu_fault_address(vcpu);
    seL4_Word mask = 0;
    switch (size) {
    case sizeof(uint8_t):
        mask = 0x000000ff;
        break;
    case sizeof(uint16_t):
        mask = 0x0000ffff;
        break;
    case sizeof(uint32_t):
        mask = 0xffffffff;
        break;
    default:
        ZF_LOGE("Invalid fault size");
        return 0;
    }
    mask <<= (addr & 0x3) * 8;
    return mask;
}

bool is_vcpu_read_fault(vm_vcpu_t *vcpu)
{
    unsigned int qualification = vm_guest_exit_get_qualification(vcpu->vcpu_arch.guest_state);
    return true ? qualification & BIT(0) : false;
}

int set_vcpu_fault_data(vm_vcpu_t *vcpu, seL4_Word data)
{
    int reg;
    seL4_Word imm;
    int size;
    vm_decode_ept_violation(vcpu, &reg, &imm, &size);
    int vcpu_reg = vm_decoder_reg_mapw[reg];
    return vm_set_thread_context_reg(vcpu, vcpu_reg, data);
}

void advance_vcpu_fault(vm_vcpu_t *vcpu)
{
    vm_guest_exit_next_instruction(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
}

void restart_vcpu_fault(vm_vcpu_t *vcpu)
{
    return;
}
