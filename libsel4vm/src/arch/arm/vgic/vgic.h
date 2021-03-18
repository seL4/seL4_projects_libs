/*
 * Copyright 2019, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */

#pragma once

#include <sel4vm/guest_vm.h>

struct vgic_dist_device {
    uintptr_t pstart;
    size_t size;
    void *priv;
};

extern const struct vgic_dist_device dev_vgic_dist;

int vm_install_vgic(vm_t *vm);
int vm_vgic_maintenance_handler(vm_vcpu_t *vcpu);
