/*
 * Copyright 2018, Data61
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

#include <stdbool.h>

#include <utils/util.h>
#include <sel4arm-vmm/fault.h>

#define SRT_MASK    0x1f

static inline bool sel4arch_fault_is_thumb(fault_t *f)
{
    return CPSR_IS_THUMB(fault_get_ctx(f)->spsr);
}

static inline seL4_Word smc_get_function_id(seL4_UserContext *u)
{
    return u->x0;
}

static inline seL4_Word smc_set_return_value(seL4_UserContext *u, seL4_Word val)
{
    u->x0 = val;
}
