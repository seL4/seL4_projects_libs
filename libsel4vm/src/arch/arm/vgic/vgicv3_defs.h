/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

/*
 * GIC Distributor Register Map
 * ARM Generic Interrupt Controller (Architecture version 3.0)
 * Architecture Specification (Issue C)
 * Chapter 8 Programmers' Model - Table 8-25
 */
#define GIC_DIST_CTLR           0x000
#define GIC_DIST_TYPER          0x004
#define GIC_DIST_IIDR           0x008
#define GIC_DIST_STATUSR        0x010
#define GIC_DIST_SETSPI_NSR     0x040
#define GIC_DIST_CLRSPI_NSR     0x048
#define GIC_DIST_SETSPI_SR      0x050
#define GIC_DIST_CLRSPI_SR      0x058
#define GIC_DIST_IGROUPR0       0x080
#define GIC_DIST_IGROUPR1       0x084
#define GIC_DIST_IGROUPRN       0x0FC
#define GIC_DIST_ISENABLER0     0x100
#define GIC_DIST_ISENABLER1     0x104
#define GIC_DIST_ISENABLERN     0x17C
#define GIC_DIST_ICENABLER0     0x180
#define GIC_DIST_ICENABLER1     0x184
#define GIC_DIST_ICENABLERN     0x1fC
#define GIC_DIST_ISPENDR0       0x200
#define GIC_DIST_ISPENDR1       0x204
#define GIC_DIST_ISPENDRN       0x27C
#define GIC_DIST_ICPENDR0       0x280
#define GIC_DIST_ICPENDR1       0x284
#define GIC_DIST_ICPENDRN       0x2FC
#define GIC_DIST_ISACTIVER0     0x300
#define GIC_DIST_ISACTIVER1     0x304
#define GIC_DIST_ISACTIVERN     0x37C
#define GIC_DIST_ICACTIVER0     0x380
#define GIC_DIST_ICACTIVER1     0x384
#define GIC_DIST_ICACTIVERN     0x3FC
#define GIC_DIST_IPRIORITYR0    0x400
#define GIC_DIST_IPRIORITYR7    0x41C
#define GIC_DIST_IPRIORITYR8    0x420
#define GIC_DIST_IPRIORITYRN    0x7F8
#define GIC_DIST_ITARGETSR0     0x800
#define GIC_DIST_ITARGETSR7     0x81C
#define GIC_DIST_ITARGETSR8     0x820
#define GIC_DIST_ITARGETSRN     0xBF8
#define GIC_DIST_ICFGR0         0xC00
#define GIC_DIST_ICFGRN         0xCFC
#define GIC_DIST_IGRPMODR0      0xD00
#define GIC_DIST_IGRPMODRN      0xD7C
#define GIC_DIST_NSACR0         0xE00
#define GIC_DIST_NSACRN         0xEFC
#define GIC_DIST_SGIR           0xF00
#define GIC_DIST_CPENDSGIR0     0xF10
#define GIC_DIST_CPENDSGIRN     0xF1C
#define GIC_DIST_SPENDSGIR0     0xF20
#define GIC_DIST_SPENDSGIRN     0xF2C
#define GIC_DIST_IROUTER0      0x6100
#define GIC_DIST_IROUTERN      0x7FD8

/*
 * ARM Generic Interrupt Controller (Architecture version 3.0)
 * Architecture Specification (Issue C)
 * 8.9.21: Software Generated Interrupt Register, GICD_SGIR
 */
#define GIC_DIST_SGI_TARGET_LIST_FILTER_SHIFT   24
#define GIC_DIST_SGI_TARGET_LIST_FILTER_MASK    0x3 << GIC_DIST_SGI_TARGET_LIST_FILTER_SHIFT
#define GIC_DIST_SGI_TARGET_LIST_SPEC           0
#define GIC_DIST_SGI_TARGET_LIST_OTHERS         1
#define GIC_DIST_SGI_TARGET_SELF                2

#define GIC_DIST_SGI_CPU_TARGET_LIST_SHIFT      16
#define GIC_DIST_SGI_CPU_TARGET_LIST_MASK       0xFF << GIC_DIST_SGI_CPU_TARGET_LIST_SHIFT

#define GIC_DIST_SGI_INTID_MASK                 0xF

#define GICR_CTLR           0x000
#define GICR_IIDR           0x004
#define GICR_TYPER          0x008
#define GICR_WAKER          0x014




#define GICR_IGROUPR0       0x10080
#define GICR_ISENABLER0     0x10100
#define GICR_ICENABLER0     0x10180
#define GICR_ICACTIVER0     0x10380
#define GICR_IPRIORITYR0    0x10400
#define GICR_IPRIORITYRN    0x1041C
#define GICR_ICFGR1         0x10c04