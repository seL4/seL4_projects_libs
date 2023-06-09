/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/***
 * @module guest_x86_context.h
 * The libsel4vm x86 context interface provides a set of useful getters and setters on x86 vcpu thread contexts.
 * This interface is commonly leveraged by VMM's to initialise a vcpu state, process a vcpu fault and
 * accordingly update its state.
 */

#include <sel4/sel4.h>
#include <sel4vm/guest_vm.h>

typedef enum vcpu_context_reg {
    VCPU_CONTEXT_EAX = 0,
    VCPU_CONTEXT_EBX,
    VCPU_CONTEXT_ECX,
    VCPU_CONTEXT_EDX,
    VCPU_CONTEXT_ESI,
    VCPU_CONTEXT_EDI,
    VCPU_CONTEXT_EBP,
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
    VCPU_CONTEXT_R8,
    VCPU_CONTEXT_R9,
    VCPU_CONTEXT_R10,
    VCPU_CONTEXT_R11,
    VCPU_CONTEXT_R12,
    VCPU_CONTEXT_R13,
    VCPU_CONTEXT_R14,
    VCPU_CONTEXT_R15
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
} vcpu_context_reg_t;

/***
 * @function vm_set_thread_context(vcpu, context)
 * Set a VCPU's thread registers given a seL4_VCPUContext
 * @param {vm_vcpu_t *} vcpu            Handle to the vcpu
 * @param {seL4_VCPUContext} context    seL4_VCPUContext applied to VCPU Registers
 * @return                              0 on success, otherwise -1 for error
 */
int vm_set_thread_context(vm_vcpu_t *vcpu, seL4_VCPUContext context);

/***
 * @function vm_set_thread_context_reg(vcpu, reg, value)
 * Set a single VCPU's thread register in a seL4_VCPUContext
 * @param {vm_vcpu_t *} vcpu            Handle to the vcpu
 * @param {vcpu_context_reg_t} reg      Register enumerated by vcpu_context_reg
 * @param {seL4_Word} value             Value to set register with
 * @return                              0 on success, otherwise -1 for error
 */
int vm_set_thread_context_reg(vm_vcpu_t *vcpu, vcpu_context_reg_t reg, seL4_Word value);

/***
 * @function vm_get_thread_context(vcpu, context)
 * Get a VCPU's thread context
 * @param {vm_vcpu_t *} vcpu                Handle to the vcpu
 * @param {seL4_VCPUContext *} context      Pointer to user supplied seL4_VCPUContext to populate with VCPU's current context
 * @return                                  0 on success, otherwise -1 for error
 */
int vm_get_thread_context(vm_vcpu_t *vcpu, seL4_VCPUContext *context);

/***
 * @function vm_get_thread_context_reg(vcpu, reg, value)
 * Get a single VCPU's thread register
 * @param {vm_vcpu_t *} vcpu            Handle to the vcpu
 * @param {vcpu_context_reg_t} reg      Register enumerated by vcpu_context_reg
 * @param {seL4_Word *} value           Pointer to user supplied variable to populate register value with
 * @return                              0 on success, otherwise -1 for error
 */
int vm_get_thread_context_reg(vm_vcpu_t *vcpu, vcpu_context_reg_t reg, seL4_Word *value);

/* VMCS Getters and Setters */

/***
 * @function vm_set_vmcs_field(vcpu, field, value)
 * Set a VMCS field
 * @param {vm_vcpu_t *} vcpu        Handle to the vcpu
 * @param {seL4_Word} reg           VMCS field
 * @param {uint32_t} value          Value to set VMCS field with
 * @return                          0 on success, otherwise -1 for error
 */
int vm_set_vmcs_field(vm_vcpu_t *vcpu, seL4_Word field, seL4_Word value);

/***
 * @function vm_get_vmcs_field(vcpu, field, value)
 * Get a VMCS register
 * @param {vm_vcpu_t *} vcpu        Handle to the vcpu
 * @param {seL4_Word} reg           VMCS field
 * @param {uint32_t *} value        Pointer to user supplied variable to populate VMCS field value with
 * @return                          0 on success, otherwise -1 for error
 */
int vm_get_vmcs_field(vm_vcpu_t *vcpu, seL4_Word field, seL4_Word *value);
