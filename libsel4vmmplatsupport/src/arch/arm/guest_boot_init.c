/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vm/guest_vm.h>
#include <sel4vm/boot.h>
#include <sel4vm/arch/guest_arm_context.h>

#include "guest_boot_sel4arch.h"

int vcpu_set_bootargs(vm_vcpu_t *vcpu, seL4_Word pc, seL4_Word mach_type, seL4_Word atags)
{
    seL4_UserContext regs = {0};
    if (!vcpu) {
        ZF_LOGE("Failed to set bootargs: NULL VCPU handle");
        return -1;
    }
    sel4arch_set_bootargs(&regs, pc, mach_type, atags);

    /* Write VCPU thread registers */
    int err = vm_set_thread_context(vcpu, regs);
    if (err) {
        ZF_LOGE("Failed to set VCPU thread context");
    }
    return err;
}
