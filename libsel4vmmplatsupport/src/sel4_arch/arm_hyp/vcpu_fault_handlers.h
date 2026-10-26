/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_vcpu_fault.h>
#include <sel4vm/arch/processor.h>
#include <sel4vm/sel4_arch/processor.h>
#include <sel4vmmplatsupport/arch/guest_vcpu_fault.h>

#include "vcpu_fault.h"
#include "smc.h"

/*
 * Trapped MCR/MRC and MCRR/MRRC accesses to CP14 and CP15. Ignore all trapped
 * writes to these registers for now and leave Rt unchanged on reads. These
 * accesses are mostly used by Linux during boot to set up registers seL4
 * traps and already sets up itself.
 */
static inline int ignore_coproc_exception(vm_vcpu_t *vcpu, uint32_t hsr)
{
    advance_vcpu_fault(vcpu);
    return 0;
}

static vcpu_exception_handler_fn vcpu_exception_handlers[] = {
    [0 ... HSR_MAX_EXCEPTION] = unknown_vcpu_exception_handler,
    [HSR_WFx_EXCEPTION] = ignore_exception,
    [HSR_CP14_32_EXCEPTION] = ignore_coproc_exception,
    [HSR_CP14_64_EXCEPTION] = ignore_coproc_exception,
    [HSR_CP15_32_EXCEPTION] = ignore_coproc_exception,
    [HSR_CP15_64_EXCEPTION] = ignore_coproc_exception,
    [HSR_SMC_EXCEPTION] = handle_smc,
};
