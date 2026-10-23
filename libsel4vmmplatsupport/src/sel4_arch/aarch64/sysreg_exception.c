/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_vcpu_fault.h>
#include <sel4vm/arch/processor.h>
#include <sel4vm/arch/guest_arm_context.h>
#include <sel4vm/sel4_arch/processor.h>
#include <sel4vmmplatsupport/arch/guest_vcpu_fault.h>

#include "vcpu_fault_handlers.h"

#include "sysreg_exception.h"

static int ignore_sysreg_exception(vm_vcpu_t *vcpu, sysreg_t *sysreg, bool is_read);

sysreg_entry_t sysreg_table[] = {
#ifdef CONFIG_ARM_CORTEX_A57
    /* S3_1_c15_c2_0: Write EL1 CPU Auxiliary Control Register */
    {
        .sysreg = { .params.op0 = 3, .params.op1 = 1, .params.op2 = 0, .params.crn = 15, .params.crm = 2 },
        .sysreg_match_mask = { .hsr_val  = SYSREG_MATCH_ALL_MASK },
        .handler = ignore_sysreg_exception
    },
#endif
    /* Debug and Trace Register Operations */
    {
        .sysreg = { .params.op0 = 2 },
        .sysreg_match_mask = { .hsr_val = SYSREG_OP0_MASK },
        .handler = ignore_sysreg_exception
    },
    /* Performance Monitors registers, S3_<op1>_c9_c<crm>_<op2> */
    {
        .sysreg = { .params.op0 = 3, .params.crn = 9 },
        .sysreg_match_mask = { .hsr_val = SYSREG_OP0_MASK | SYSREG_CRn_MASK },
        .handler = ignore_sysreg_exception
    },
    /* PMEVCNTR<n>_EL0, PMEVTYPER<n>_EL0, PMCCFILTR_EL0: S3_3_c14_c<8..15>_<op2> */
    {
        .sysreg = { .params.op0 = 3, .params.op1 = 3, .params.crn = 14, .params.crm = 0b1000 },
        .sysreg_match_mask = { .hsr_val = SYSREG_OP0_MASK | SYSREG_OP1_MASK | SYSREG_CRn_MASK | (0b1000 << SYSREG_CRm_SHIFT) },
        .handler = ignore_sysreg_exception
    },
};

static int ignore_sysreg_exception(vm_vcpu_t *vcpu, sysreg_t *sysreg, bool is_read)
{
    advance_vcpu_fault(vcpu);
    return 0;
}

static bool is_sysreg_match(sysreg_t *sysreg, sysreg_entry_t *sysreg_entry)
{
    sysreg_t match_a = *sysreg;
    match_a.hsr_val &= sysreg_entry->sysreg_match_mask.hsr_val;
    sysreg_t match_b = sysreg_entry->sysreg;
    return (
               (match_a.params.op0 == match_b.params.op0) &&
               (match_a.params.op1 == match_b.params.op1) &&
               (match_a.params.op2 == match_b.params.op2) &&
               (match_a.params.crn == match_b.params.crn) &&
               (match_a.params.crm == match_b.params.crm)
           );
}

static sysreg_entry_t *find_sysreg_entry(vm_vcpu_t *vcpu, sysreg_t *sysreg_op)
{
    for (int i = 0; i < ARRAY_SIZE(sysreg_table); i++) {
        sysreg_entry_t *sysreg_entry = &sysreg_table[i];
        sysreg_t match_sysreg_op = *sysreg_op;
        if (is_sysreg_match(sysreg_op, sysreg_entry)) {
            return sysreg_entry;
        }
    }
    return NULL;
}

int sysreg_exception_handler(vm_vcpu_t *vcpu, uint32_t hsr)
{
    sysreg_t sysreg_op;
    sysreg_op.hsr_val = hsr;
    sysreg_entry_t *entry = find_sysreg_entry(vcpu, &sysreg_op);
    if (!entry) {
        return -1;
    }
    return entry->handler(vcpu, &entry->sysreg, sysreg_op.params.direction);
}
