/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <sel4vm/gen_config.h>
#include <sel4vm/guest_vm.h>

/* FIXME these should be defined in a way that is friendlier to extension. */
#if defined(CONFIG_PLAT_EXYNOS5)
#define GIC_PADDR   0x10480000
#elif defined(CONFIG_PLAT_TK1) || defined(CONFIG_PLAT_TX1)
#define GIC_PADDR   0x50040000
#elif defined(CONFIG_PLAT_TX2)
#define GIC_PADDR   0x03880000
#elif defined(CONFIG_PLAT_QEMU_ARM_VIRT)
#define GIC_PADDR   0x8000000
#elif defined(CONFIG_PLAT_ODROIDC2)
#define GIC_PADDR   0xc4300000
#elif defined(CONFIG_PLAT_IMX8MM_EVK) || defined(CONFIG_PLAT_IMX8MQ_EVK)
#define GIC_PADDR   0x38800000
#else
#error "Unsupported platform for GIC"
#endif

#if defined(CONFIG_PLAT_IMX8MM_EVK) || defined(CONFIG_PLAT_IMX8MQ_EVK)
#define GIC_VERSION_3
#else
#define GIC_VERSION_2
#endif

#ifdef CONFIG_PLAT_QEMU_ARM_VIRT
#define GIC_DIST_PADDR       (GIC_PADDR)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x00010000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x00030000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x00040000)
#elif defined(GIC_VERSION_2)
#define GIC_DIST_PADDR       (GIC_PADDR + 0x1000)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x2000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x4000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x6000)
#else
#define GIC_DIST_PADDR (GIC_PADDR)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x2000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x4000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x6000)
#endif

struct vgic_device {
    uintptr_t pstart;
    size_t size;
    void *priv;
};

extern const struct vgic_device dev_vgic_dist;

int vm_install_vgic_v2(vm_t *vm);
int vm_install_vgic_v3(vm_t *vm);
int vm_vgic_maintenance_handler(vm_vcpu_t *vcpu);
