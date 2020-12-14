/*
 * Copyright 2023, DornerWorks
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4vm/guest_vm.h>

/* SMC Helpers */
seL4_Word smc_get_function_id(seL4_UserContext *u);
seL4_Word smc_set_return_value(seL4_UserContext *u, seL4_Word val);
seL4_Word smc_get_arg(seL4_UserContext *u, seL4_Word arg);
void smc_set_arg(seL4_UserContext *u, seL4_Word arg, seL4_Word val);

/***
 * @function smc_forward(vm_vcpu_t *vcpu, seL4_UserContext *regs, seL4_ARM_SMC smc_cap)
 * Forward an SMC call using the appropriate capability
 * @param {vm_vcpu_t *} vcpu           A handle to the VCPU
 * @param {seL4_UserContext *} regs    A handle to the registers from the calling thread that want to make an SMC call
 * @param {seL4_ARM_SMC} smc_cap       The SMC capability for the requested call
 * @return                             0 on success, -1 on error
 */
int smc_forward(vm_vcpu_t *vcpu, seL4_UserContext *regs, seL4_ARM_SMC smc_cap);

/***
 * @function vm_smc_handle_default(vm_vcpu_t *vcpu, seL4_UserContext *regs)
 * The default handler SMC faults. Will be called if a custom handler is not set for any given VM.
 * @param {vm_vcpu_t *} vcpu           A handle to the VCPU
 * @param {seL4_UserContext *} regs    A handle to the registers from the calling thread that want to make an SMC call
 * @return                             0 on success, -1 on error
 */
int vm_smc_handle_default(vm_vcpu_t *vcpu, seL4_UserContext *regs);
