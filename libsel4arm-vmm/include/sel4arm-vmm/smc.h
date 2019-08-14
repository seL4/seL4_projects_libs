/*
 * Copyright 2019, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */
#pragma once

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

static inline smc_call_id_t smc_get_call(uintptr_t func_id)
{
    seL4_Word service = ((func_id >> SMC_SERVICE_CALL_SHIFT) & SMC_SERVICE_CALL_MASK);
    assert(service >= 0 && service <= 0xFFFF);

    if (service <= SMC_CALL_VENDOR_HYP_SERVICE) {
        return service;
    } else if (service < SMC_CALL_TRUSTED_APP) {
        return SMC_CALL_RESERVED;
    } else if (service < SMC_CALL_TRUSTED_OS) {
        return SMC_CALL_TRUSTED_APP;
    } else if (service < SMC_CALL_RESERVED) {
        return SMC_CALL_TRUSTED_OS;
    } else {
        return SMC_CALL_RESERVED;
    }
}

static inline bool smc_call_is_32(uintptr_t func_id)
{
    return !!(func_id & SMC_CALLING_CONVENTION) == SMC_CALLING_CONVENTION_32;
}

static inline bool smc_call_is_atomic(uintptr_t func_id)
{
    return !!(func_id & SMC_FAST_CALL);
}

static inline uintptr_t smc_get_function_number(uintptr_t func_id)
{
    return (func_id & SMC_FUNC_ID_MASK);
}
