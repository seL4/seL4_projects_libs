/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vm/guest_vm.h>

typedef struct vgic vgic_t;

struct vgic_dist_device {
    uintptr_t pstart;
    size_t size;
    vgic_t *vgic;
};

extern const struct vgic_dist_device dev_vgic_dist;

int vm_install_vgic(vm_t *vm);
int vm_vgic_maintenance_handler(vm_vcpu_t *vcpu);
