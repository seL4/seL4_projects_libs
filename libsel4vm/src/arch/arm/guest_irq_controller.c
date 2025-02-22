/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "vgic/vgic.h"

int vm_create_default_irq_controller(vm_t *vm)
{
    if (!vm) {
        ZF_LOGE("Failed to initialise default irq controller: Invalid vm");
        return -1;
    }

    if (!vm->gic_path) {
        ZF_LOGE("Failed to find GIC path in VM structure");
        return -1;
    }

    if (!vm->fdt_ori) {
        ZF_LOGE("Failed to find reference FDT");
        return -1;
    }

    return vm_install_vgic(vm);
}
