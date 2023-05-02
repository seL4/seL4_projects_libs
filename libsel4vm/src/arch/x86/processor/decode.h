/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */
#pragma once

#include <sel4vm/guest_vm.h>
#include <sel4vm/arch/guest_x86_context.h>

#define MAX_INSTR_OPCODES 255
#define OP_ESCAPE 0xf

int vm_fetch_instruction(vm_vcpu_t *vcpu, uintptr_t eip, uintptr_t cr3, int len, uint8_t *buf);

int vm_decode_instruction(uint8_t *instr, int instr_len, int *reg, seL4_Word *imm, int *op_len);

void vm_decode_ept_violation(vm_vcpu_t *vcpu, int *reg, seL4_Word *imm, int *size);

/* Interpret just enough virtual 8086 instructions to run trampoline code.
   Returns the final jump address */
uintptr_t vm_emulate_realmode(vm_vcpu_t *vcpu, uint8_t *instr_buf,
                              uint16_t *segment, uintptr_t eip, uint32_t len, guest_state_t *gs,
                              int m66_set);

// TODO don't have these in a header, make them inline functions
const static int vm_decoder_reg_mapw[] = {
    VCPU_CONTEXT_EAX,
    VCPU_CONTEXT_ECX,
    VCPU_CONTEXT_EDX,
    VCPU_CONTEXT_EBX,
    /*VCPU_CONTEXT_ESP*/ -1,
    VCPU_CONTEXT_EBP,
    VCPU_CONTEXT_ESI,
    VCPU_CONTEXT_EDI,
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
    VCPU_CONTEXT_R8,
    VCPU_CONTEXT_R9,
    VCPU_CONTEXT_R10,
    VCPU_CONTEXT_R11,
    VCPU_CONTEXT_R12,
    VCPU_CONTEXT_R13,
    VCPU_CONTEXT_R14,
    VCPU_CONTEXT_R15,
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
};

const static int vm_decoder_reg_mapb[] = {
    VCPU_CONTEXT_EAX,
    VCPU_CONTEXT_ECX,
    VCPU_CONTEXT_EDX,
    VCPU_CONTEXT_EBX,
    VCPU_CONTEXT_EAX,
    VCPU_CONTEXT_ECX,
    VCPU_CONTEXT_EDX,
    VCPU_CONTEXT_EBX
};

