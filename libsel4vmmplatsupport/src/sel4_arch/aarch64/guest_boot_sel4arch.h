/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <sel4/sel4.h>
#include <sel4vm/sel4_arch/processor.h>

static inline void sel4arch_set_bootargs(seL4_UserContext *regs, seL4_Word pc,
                                         seL4_Word mach_type, seL4_Word atags)
{
    regs->x0 = atags;
    regs->pc = pc;
    regs->spsr = PMODE_EL1h;
}
