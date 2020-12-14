/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */


#include <sel4vm/guest_vm.h>

#include "smc.h"

seL4_Word smc_get_function_id(seL4_UserContext *u)
{
    return u->x0;
}

seL4_Word smc_set_return_value(seL4_UserContext *u, seL4_Word val)
{
    u->x0 = val;
}

seL4_Word smc_get_arg(seL4_UserContext *u, seL4_Word arg)
{
    switch (arg) {
    case 1:
        return u->x1;
    case 2:
        return u->x2;
    case 3:
        return u->x3;
    case 4:
        return u->x4;
    case 5:
        return u->x5;
    case 6:
        return u->x6;
    case 7:
        return u->x7;
    default:
        ZF_LOGF("SMC only has 7 argument registers");
    }
}

void smc_set_arg(seL4_UserContext *u, seL4_Word arg, seL4_Word val)
{
    switch (arg) {
    case 1:
        u->x1 = val;
        break;
    case 2:
        u->x2 = val;
        break;
    case 3:
        u->x3 = val;
        break;
    case 4:
        u->x4 = val;
        break;
    case 5:
        u->x5 = val;
        break;
    case 6:
        u->x6 = val;
        break;
    case 7:
        u->x7 = val;
        break;
    default:
        ZF_LOGF("SMC only has 7 argument registers");
    }
}

int smc_forward(vm_vcpu_t *vcpu, seL4_UserContext *regs, seL4_ARM_SMC smc_cap)
{
    int err = 0;
    seL4_ARM_SMCContext smc_args;
    seL4_ARM_SMCContext smc_results;

    /* Get function and arguments from guest */
    smc_args.x0 = regs->x0;
    smc_args.x1 = regs->x1;
    smc_args.x2 = regs->x2;
    smc_args.x3 = regs->x3;
    smc_args.x4 = regs->x4;
    smc_args.x5 = regs->x5;
    smc_args.x6 = regs->x6;
    smc_args.x7 = regs->x7;

    /* Make systemcall */
    err = seL4_ARM_SMC_Call(smc_cap, &smc_args, &smc_results);
    if (err) {
        ZF_LOGE("Failure during seL4_ARM_SMC_Call function %lu\n", smc_args.x0);
        return -1;
    }

    /* Send SMC results back to guest */
    regs->x0 = smc_results.x0;
    regs->x1 = smc_results.x1;
    regs->x2 = smc_results.x2;
    regs->x3 = smc_results.x3;
    regs->x4 = smc_results.x4;
    regs->x5 = smc_results.x5;
    regs->x6 = smc_results.x6;
    regs->x7 = smc_results.x7;

    return 0;
}
