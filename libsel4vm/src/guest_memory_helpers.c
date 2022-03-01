/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_memory.h>

memory_fault_result_t default_error_fault_callback(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                   size_t fault_length, void *cookie)
{
    ZF_LOGE("Failed to handle fault addr: 0x%"PRIxPTR, fault_addr);
    return FAULT_ERROR;
}
