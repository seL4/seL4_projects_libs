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

int handle_psci(vm_vcpu_t *vcpu, seL4_Word fn_number, bool convention)
{
    int err;
    seL4_UserContext regs;
    err = vm_get_thread_context(vcpu, &regs);
    if (err) {
        ZF_LOGE("Failed to get vcpu registers");
        return -1;
    }
    switch (fn_number) {
    case PSCI_VERSION:
        smc_set_return_value(&regs, 0x00010000); /* version 1 */
        break;
    case PSCI_CPU_ON: {
        uintptr_t requested_cpu = smc_get_arg(&regs, 1);
        uintptr_t entry_point_address = smc_get_arg(&regs, 2);
        uintptr_t context_id = smc_get_arg(&regs, 3);
        vm_vcpu_t *target_vcpu = NULL;
        if ((requested_cpu >= 0) && (requested_cpu < vcpu->vm->num_vcpus)) {
            target_vcpu = vcpu->vm->vcpus[requested_cpu];
        } else {
            smc_set_return_value(&regs, PSCI_INTERNAL_FAILURE);
            break;
        }

        /* Automatically assign vcpu to an unassigned physical cpu */
        if (target_vcpu->target_cpu == -1) {
            int selected_cpu = vm_find_free_unassigned_cpu(vcpu->vm);
            if ((selected_cpu >= 0) && start_new_vcpu(target_vcpu, entry_point_address, context_id, selected_cpu) == 0) {
                smc_set_return_value(&regs, PSCI_SUCCESS);
            } else {
                ZF_LOGE("[vCPU %u] no unused physical core left", vcpu->vcpu_id);
                smc_set_return_value(&regs, PSCI_INTERNAL_FAILURE);
            }
        } else {
            if (is_vcpu_online(target_vcpu)) {
                smc_set_return_value(&regs, PSCI_ALREADY_ON);
            } else {
                smc_set_return_value(&regs, PSCI_INTERNAL_FAILURE);
            }
        }

        break;
    }
    case PSCI_MIGRATE_INFO_TYPE:
        /* trusted OS does not require migration */
        smc_set_return_value(&regs, 2);
        break;
    case PSCI_FEATURES:
        /* TODO Not sure if required */
        smc_set_return_value(&regs, PSCI_NOT_SUPPORTED);
        break;
    case PSCI_SYSTEM_RESET:
        smc_set_return_value(&regs, PSCI_SUCCESS);
        break;
    default:
        ZF_LOGE("Unhandled PSCI function id %lu\n", fn_number);
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
