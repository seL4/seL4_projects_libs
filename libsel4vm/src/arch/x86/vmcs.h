/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <sel4/sel4.h>

#include <sel4vm/guest_vm.h>

int vm_vmcs_read(seL4_CPtr vcpu, seL4_Word field, seL4_Word *value);
int vm_vmcs_write(seL4_CPtr vcpu, seL4_Word field, seL4_Word value);
void vm_vmcs_init_guest(vm_vcpu_t *vcpu);
