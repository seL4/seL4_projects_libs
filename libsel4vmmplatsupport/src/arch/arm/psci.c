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
        ZF_LOGE("Failed to assign vcpu target");
        return -1;
    }
    err = vcpu_set_bootargs(vcpu, entry_address, 0, context_id);
    if (err) {
        ZF_LOGE("Failed to set bootargs");
        vm_assign_vcpu_target(vcpu, -1);
        return -1;
    }
    err = vcpu_start(vcpu);
    if (err) {
        ZF_LOGE("Failed to start vcpu. Assigning -1 target");
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
        ZF_LOGE("target_cpu: %d", target_cpu);
        vm_vcpu_t *target_vcpu = vm_vcpu_for_target_cpu(vcpu->vm, target_cpu);
        if (target_vcpu == NULL) {
            ZF_LOGE("Null VCPU");
            target_vcpu = vm_find_free_unassigned_vcpu(vcpu->vm);
            ZF_LOGE("Found free unassigned VCPU: %p", target_vcpu);
            if (target_vcpu && start_new_vcpu(target_vcpu, entry_point_address, context_id, target_cpu) == 0) {
                ZF_LOGE("Started found VCPU");
                smc_set_return_value(regs, PSCI_SUCCESS);
            } else {
                ZF_LOGE("Failed to start found VCPU");
                smc_set_return_value(regs, PSCI_INTERNAL_FAILURE);
            }
        } else if ((target_vcpu->target_cpu >= 0) && (target_vcpu->target_cpu < CONFIG_MAX_NUM_NODES)) {
            ZF_LOGE("Found existing VCPU");
            /* Assign vcpu to physical cpu specified in config */
            if (is_vcpu_online(target_vcpu)) {
                ZF_LOGE("its already on...");
                smc_set_return_value(regs, PSCI_ALREADY_ON);
            } else if (start_new_vcpu(target_vcpu, entry_point_address, context_id, target_vcpu->target_cpu) == 0) {
                ZF_LOGE("started VCPU that we already found!");
                smc_set_return_value(regs, PSCI_SUCCESS);
            } else {
                ZF_LOGE("[vCPU %u] could not start vCPU", vcpu->vcpu_id);
                smc_set_return_value(regs, PSCI_INTERNAL_FAILURE);
            }
        } else {
            ZF_LOGE("[vCPU %u] invalid target CPU %d", vcpu->vcpu_id, target_vcpu->target_cpu);
            smc_set_return_value(regs, PSCI_INTERNAL_FAILURE);
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
