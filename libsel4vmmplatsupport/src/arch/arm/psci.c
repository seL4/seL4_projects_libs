/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vm/guest_vm.h>
#include <sel4vm/boot.h>
#include <sel4vm/guest_vm_util.h>
#include <sel4vm/guest_vcpu_fault.h>
#include <sel4vm/arch/guest_arm_context.h>

#include <sel4vmmplatsupport/guest_vcpu_util.h>
#include <sel4vmmplatsupport/arch/guest_boot_init.h>

#include "psci.h"
#include "smc.h"

static int start_new_vcpu(vm_vcpu_t *vcpu,  uintptr_t entry_address, uintptr_t context_id, int target_cpu)
{
    int err;
    err = vm_assign_vcpu_target(vcpu, target_cpu);
    if (err) {
        return -1;
    }
    err = vcpu_set_bootargs(vcpu, entry_address, 0, context_id);
    if (err) {
        vm_assign_vcpu_target(vcpu, -1);
        return -1;
    }
    err = vcpu_start(vcpu);
    if (err) {
        vm_assign_vcpu_target(vcpu, -1);
        return -1;
    }
    return 0;
}

int handle_psci(vm_vcpu_t *vcpu, seL4_UserContext *regs, seL4_Word fn_number, bool convention)
{
    int err;
    switch (fn_number) {
    case PSCI_VERSION:
        smc_set_return_value(regs, 0x00010000); /* version 1 */
        break;
    case PSCI_CPU_ON: {
        uintptr_t target_cpu = smc_get_arg(regs, 1);
        uintptr_t entry_point_address = smc_get_arg(regs, 2);
        uintptr_t context_id = smc_get_arg(regs, 3);
        vm_vcpu_t *target_vcpu = vm_vcpu_for_target_cpu(vcpu->vm, target_cpu);
        if (target_vcpu == NULL) {
            target_vcpu = vm_find_free_unassigned_vcpu(vcpu->vm);
            if (target_vcpu && start_new_vcpu(target_vcpu, entry_point_address, context_id, target_cpu) == 0) {
                smc_set_return_value(regs, PSCI_SUCCESS);
            } else {
                smc_set_return_value(regs, PSCI_INTERNAL_FAILURE);
            }
        } else {
            if (is_vcpu_online(target_vcpu)) {
                smc_set_return_value(regs, PSCI_ALREADY_ON);
            } else {
                smc_set_return_value(regs, PSCI_INTERNAL_FAILURE);
            }
        }

        break;
    }
    case PSCI_MIGRATE_INFO_TYPE:
        /* trusted OS does not require migration */
        smc_set_return_value(regs, 2);
        break;
    case PSCI_FEATURES:
        /* TODO Not sure if required */
        smc_set_return_value(regs, PSCI_NOT_SUPPORTED);
        break;
    case PSCI_SYSTEM_RESET:
        smc_set_return_value(regs, PSCI_SUCCESS);
        break;
    default:
        ZF_LOGE("Unhandled PSCI function id %lu\n", fn_number);
        return -1;
    }
    return 0;
}
