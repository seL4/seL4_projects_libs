/*
 * Copyright 2018, Data61
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

#include <sel4arm-vmm/plat/device_map.h>
#include <sel4arm-vmm/vm.h>

#define GIC_DIST_PADDR    0x38800000
#define GIC_REDIST_PADDR  0x38880000
#define MAX_VIRQS         512
extern const struct device dev_vram;
