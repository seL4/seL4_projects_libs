/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <autoconf.h>
#include <assert.h>
#include <stdint.h>


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
#elif defined(CONFIG_PLAT_ZYNQMP)
#define GIC_PADDR   0xf9000000
#else
#error "Unsupported platform for GIC"
#endif

#ifdef CONFIG_PLAT_QEMU_ARM_VIRT
#define GIC_DIST_PADDR       (GIC_PADDR)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x00010000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x00030000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x00040000)
#elif defined(CONFIG_PLAT_ZYNQMP)
#define GIC_DIST_PADDR       (GIC_PADDR + 0x10000)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x20000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x40000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x60000)
#else
#define GIC_DIST_PADDR       (GIC_PADDR + 0x1000)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x2000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x4000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x6000)
#endif

/* Memory map for GIC distributor */
struct gic_dist_map {
    uint32_t enable;                                    /* 0x000 */
    uint32_t ic_type;                                   /* 0x004 */
    uint32_t dist_ident;                                /* 0x008 */

    uint32_t res1[29];                                  /* [0x00C, 0x080) */

    uint32_t irq_group0[CONFIG_MAX_NUM_NODES];          /* [0x080, 0x84) */
    uint32_t irq_group[31];                             /* [0x084, 0x100) */
    uint32_t enable_set0[CONFIG_MAX_NUM_NODES];         /* [0x100, 0x104) */
    uint32_t enable_set[31];                            /* [0x104, 0x180) */
    uint32_t enable_clr0[CONFIG_MAX_NUM_NODES];         /* [0x180, 0x184) */
    uint32_t enable_clr[31];                            /* [0x184, 0x200) */
    uint32_t pending_set0[CONFIG_MAX_NUM_NODES];        /* [0x200, 0x204) */
    uint32_t pending_set[31];                           /* [0x204, 0x280) */
    uint32_t pending_clr0[CONFIG_MAX_NUM_NODES];        /* [0x280, 0x284) */
    uint32_t pending_clr[31];                           /* [0x284, 0x300) */
    uint32_t active0[CONFIG_MAX_NUM_NODES];             /* [0x300, 0x304) */
    uint32_t active[31];                                /* [0x300, 0x380) */
    uint32_t active_clr0[CONFIG_MAX_NUM_NODES];         /* [0x380, 0x384) */
    uint32_t active_clr[31];                            /* [0x384, 0x400) */
    uint32_t priority0[CONFIG_MAX_NUM_NODES][8];        /* [0x400, 0x420) */
    uint32_t priority[247];                             /* [0x420, 0x7FC) */
    uint32_t res3;                                      /* 0x7FC */

    uint32_t targets0[CONFIG_MAX_NUM_NODES][8];         /* [0x800, 0x820) */
    uint32_t targets[247];                              /* [0x820, 0xBFC) */
    uint32_t res4;                                      /* 0xBFC */

    uint32_t config[64];                                /* [0xC00, 0xD00) */

    uint32_t spi[32];                                   /* [0xD00, 0xD80) */
    uint32_t res5[20];                                  /* [0xD80, 0xDD0) */
    uint32_t res6;                                      /* 0xDD0 */
    uint32_t legacy_int;                                /* 0xDD4 */
    uint32_t res7[2];                                   /* [0xDD8, 0xDE0) */
    uint32_t match_d;                                   /* 0xDE0 */
    uint32_t enable_d;                                  /* 0xDE4 */
    uint32_t res8[70];                                  /* [0xDE8, 0xF00) */

    uint32_t sgi_control;                               /* 0xF00 */
    uint32_t res9[3];                                   /* [0xF04, 0xF10) */

    uint32_t sgi_pending_clr[CONFIG_MAX_NUM_NODES][4];  /* [0xF10, 0xF20) */
    uint32_t sgi_pending_set[CONFIG_MAX_NUM_NODES][4];  /* [0xF20, 0xF30) */
    uint32_t res10[40];                                 /* [0xF30, 0xFC0) */

    uint32_t periph_id[12];                             /* [0xFC0, 0xFF0) */
    uint32_t component_id[4];                           /* [0xFF0, 0xFFF] */
};
