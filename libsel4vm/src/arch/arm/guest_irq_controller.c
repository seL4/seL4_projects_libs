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

    #if defined(GIC_VERSION_2)
        ZF_LOGE("Installing VGICv2");
        return vm_install_vgic_v2(vm);
    #elif defined(GIC_VERSION_3)
        ZF_LOGE("Installing VGICv3");
        return vm_install_vgic_v3(vm);
    #else
        #error "GIC version is not defined."
    #endif
}
