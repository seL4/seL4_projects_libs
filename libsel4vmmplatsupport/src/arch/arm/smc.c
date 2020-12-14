/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */


#include <sel4vm/guest_vm.h>
#include <sel4vm/arch/guest_arm_context.h>
#include <sel4vm/guest_vcpu_fault.h>

#include "smc.h"
#include "psci.h"

static smc_call_id_t smc_get_call(uintptr_t func_id)
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

static bool smc_call_is_32(uintptr_t func_id)
{
    return !!(func_id & SMC_CALLING_CONVENTION) == SMC_CALLING_CONVENTION_32;
}

static bool smc_call_is_atomic(uintptr_t func_id)
{
    return !!(func_id & SMC_FAST_CALL);
}

static uintptr_t smc_get_function_number(uintptr_t func_id)
{
    return (func_id & SMC_FUNC_ID_MASK);
}

int handle_smc(vm_vcpu_t *vcpu, uint32_t hsr)
{
    int err;
    seL4_UserContext regs;

    err = vm_get_thread_context(vcpu, &regs);
    if (err) {
        ZF_LOGE("Failed to get vcpu registers to decode smc fault");
        return -1;
    }

    err = vcpu->vm->arch.vm_smc_handler(vcpu, &regs);
    if (err) {
        ZF_LOGE("SMC Handler failed");
        return -1;
    }

    err = vm_set_thread_context(vcpu, regs);
    if (err) {
        ZF_LOGE("Failed to set vcpu context registers");
        return -1;
    }
    advance_vcpu_fault(vcpu);

    return 0;
}

int vm_smc_handle_default(vm_vcpu_t *vcpu, seL4_UserContext *regs)
{
    int err;

    seL4_Word id = smc_get_function_id(regs);
    seL4_Word fn_number = smc_get_function_number(id);
    smc_call_id_t service = smc_get_call(id);

    switch (service) {
    case SMC_CALL_ARM_ARCH:
        ZF_LOGE("Unhandled SMC: arm architecture call %lu\n", fn_number);
        break;
    case SMC_CALL_CPU_SERVICE:
        ZF_LOGE("Unhandled SMC: CPU service call %lu\n", fn_number);
        break;
    case SMC_CALL_SIP_SERVICE:
        ZF_LOGE("Unhandled SMC: Got SiP service call %lu\n", fn_number);
        break;
    case SMC_CALL_OEM_SERVICE:
        ZF_LOGE("Unhandled SMC: OEM service call %lu\n", fn_number);
        break;
    case SMC_CALL_STD_SERVICE:
        if (fn_number < PSCI_MAX) {
            return handle_psci(vcpu, regs, fn_number, smc_call_is_32(id));
        }
        ZF_LOGE("Unhandled SMC: standard service call %lu\n", fn_number);
        break;
    case SMC_CALL_STD_HYP_SERVICE:
        ZF_LOGE("Unhandled SMC: standard hyp service call %lu\n", fn_number);
        break;
    case SMC_CALL_VENDOR_HYP_SERVICE:
        ZF_LOGE("Unhandled SMC: vendor hyp service call %lu\n", fn_number);
        break;
    case SMC_CALL_TRUSTED_APP:
        ZF_LOGE("Unhandled SMC: trusted app call %lu\n", fn_number);
        break;
    case SMC_CALL_TRUSTED_OS:
        ZF_LOGE("Unhandled SMC: trusted os call %lu\n", fn_number);
        break;
    default:
        ZF_LOGE("Unhandle SMC: unknown value service: %lu fn_number: %lu\n",
                (unsigned long) service, fn_number);
        break;
    }
    return -1;
}
