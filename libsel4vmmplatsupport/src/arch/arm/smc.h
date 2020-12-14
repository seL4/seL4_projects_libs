/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4vm/guest_vm.h>
#include <sel4vmmplatsupport/arch/smc.h>

/* Values in this file are taken from:
 * SMC CALLING CONVENTION
 * System Software on ARM (R) Platforms
 * Issue B
 */

#define SMC_CALLING_CONVENTION BIT(31)
#define SMC_CALLING_CONVENTION_32 0
#define SMC_CALLING_CONVENTION_64 1
#define SMC_FAST_CALL BIT(31)

#define SMC_SERVICE_CALL_MASK 0x3F
#define SMC_SERVICE_CALL_SHIFT 24

#define SMC_FUNC_ID_MASK 0xFFFF

/* SMC and HVC function identifiers */
typedef enum {
    SMC_CALL_ARM_ARCH = 0,
    SMC_CALL_CPU_SERVICE = 1,
    SMC_CALL_SIP_SERVICE = 2,
    SMC_CALL_OEM_SERVICE = 3,
    SMC_CALL_STD_SERVICE = 4,
    SMC_CALL_STD_HYP_SERVICE = 5,
    SMC_CALL_VENDOR_HYP_SERVICE = 6,
    SMC_CALL_TRUSTED_APP = 48,
    SMC_CALL_TRUSTED_OS = 50,
    SMC_CALL_RESERVED = 64,
} smc_call_id_t;

/* SMC VCPU fault handler */
int handle_smc(vm_vcpu_t *vcpu, uint32_t hsr);
