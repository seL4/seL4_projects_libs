/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vm/guest_vm.h>

typedef struct {
    uintptr_t paddr;
    size_t size;
    vm_memory_reservation_t *vm_res;
} vm_mapping_t;

typedef struct vgic vgic_t;

int vm_install_vgic(vm_t *vm);
int vm_vgic_maintenance_handler(vm_vcpu_t *vcpu);

static inline vgic_t *get_vgic_from_vm(vm_t *vm)
{
    assert(vm);
    vgic_t *vgic = (typeof(vgic))(vm->arch.vgic_context);
    assert(vgic);
    return vgic;
}
