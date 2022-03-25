
/*
 * Copyright 2017, Data61
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

#include <sel4vm/guest_vm_util.h>

//#define DEBUG_IRQ
//#define DEBUG_DIST

#ifdef DEBUG_IRQ
#define DIRQ(...) do{ printf("VIRQ: "); printf(__VA_ARGS__); }while(0)
#else
#define DIRQ(...) do{}while(0)
#endif

#ifdef DEBUG_DIST
#define DDIST(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DDIST(...) do{}while(0)
#endif

#define IRQ_IDX(irq) ((irq) / 32)
#define IRQ_BIT(irq) (1U << ((irq) % 32))

#define not_pending(...) !is_pending(__VA_ARGS__)
#define not_active(...)  !is_active(__VA_ARGS__)
#define not_enabled(...) !is_enabled(__VA_ARGS__)

/* GIC Distributor register access utilities */
#define GIC_DIST_REGN(offset, reg) ((offset-reg)/sizeof(uint32_t))
#define RANGE32(a, b) a ... b + (sizeof(uint32_t)-1)

/*
 * GIC Distributor Register Map
 * ARM Generic Interrupt Controller (Architecture version 2.0)
 * Architecture Specification (Issue B.b)
 * Chapter 4 Programmers' Model - Table 4.1
 */
#define GIC_DIST_CTLR           0x000
#define GIC_DIST_TYPER          0x004
#define GIC_DIST_IIDR           0x008
#define GIC_DIST_IGROUPR0       0x080
#define GIC_DIST_IGROUPR1       0x084
#define GIC_DIST_IGROUPRN       0x0FC
#define GIC_DIST_ISENABLER0     0x100
#define GIC_DIST_ISENABLER1     0x104
#define GIC_DIST_ISENABLERN     0x17C
#define GIC_DIST_ICENABLER0     0x180
#define GIC_DIST_ICENABLER1     0x184
#define GIC_DIST_ICENABLERN     0x1FC
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
#define GIC_DIST_NSACR0         0xE00
#define GIC_DIST_NSACRN         0xEFC
#define GIC_DIST_SGIR           0xF00
#define GIC_DIST_CPENDSGIR0     0xF10
#define GIC_DIST_CPENDSGIRN     0xF1C
#define GIC_DIST_SPENDSGIR0     0xF20
#define GIC_DIST_SPENDSGIRN     0xF2C

#define GIC_DIST_PIDR2         0xFFE8

/*
 * ARM Generic Interrupt Controller (Architecture version 2.0)
 * Architecture Specification (Issue B.b)
 * 4.3.15: Software Generated Interrupt Register, GICD_SGI
 * Values defined as per Table 4-21 (GICD_SGIR bit assignments)
 */
#define GIC_DIST_SGI_TARGET_LIST_FILTER_SHIFT   24
#define GIC_DIST_SGI_TARGET_LIST_FILTER_MASK    0x3 << GIC_DIST_SGI_TARGET_LIST_FILTER_SHIFT
#define GIC_DIST_SGI_TARGET_LIST_SPEC           0
#define GIC_DIST_SGI_TARGET_LIST_OTHERS         1
#define GIC_DIST_SGI_TARGET_SELF                2

#define GIC_DIST_SGI_CPU_TARGET_LIST_SHIFT      16
#define GIC_DIST_SGI_CPU_TARGET_LIST_MASK       0xFF << GIC_DIST_SGI_CPU_TARGET_LIST_SHIFT

#define GIC_DIST_SGI_INTID_MASK                 0xF

static inline void set_sgi_ppi_pending(struct vgic_dist_map *gic_dist, int irq, int value, int vcpu_id)
{
    if (value) {
        gic_dist->pending_set0[vcpu_id] |= IRQ_BIT(irq);
        gic_dist->pending_clr0[vcpu_id] |= IRQ_BIT(irq);
    } else {
        gic_dist->pending_set0[vcpu_id] &= ~IRQ_BIT(irq);
        gic_dist->pending_clr0[vcpu_id] &= ~IRQ_BIT(irq);
    }
}

static inline void set_spi_pending(struct vgic_dist_map *gic_dist, int irq, int value, int vcpu_id)
{
    if (value) {
        gic_dist->pending_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_dist->pending_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->pending_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_dist->pending_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline void set_pending(struct vgic_dist_map *gic_dist, int irq, int value, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        set_sgi_ppi_pending(gic_dist, irq, value, vcpu_id);
    } else {
        set_spi_pending(gic_dist, irq, value, vcpu_id);
    }
}

static inline int is_sgi_ppi_pending(struct vgic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->pending_set0[vcpu_id] & IRQ_BIT(irq));
}

static inline int is_spi_pending(struct vgic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->pending_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline int is_pending(struct vgic_dist_map *gic_dist, int irq, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        return is_sgi_ppi_pending(gic_dist, irq, vcpu_id);

    }
    return is_spi_pending(gic_dist, irq, vcpu_id);
}

static inline void set_sgi_ppi_enable(struct vgic_dist_map *gic_dist, int irq, int value, int vcpu_id)
{
    if (value) {
        gic_dist->enable_set0[vcpu_id] |= IRQ_BIT(irq);
        gic_dist->enable_clr0[vcpu_id] |= IRQ_BIT(irq);
    } else {
        gic_dist->enable_set0[vcpu_id] &= ~IRQ_BIT(irq);
        gic_dist->enable_clr0[vcpu_id] &= ~IRQ_BIT(irq);
    }
}


static inline void set_spi_enable(struct vgic_dist_map *gic_dist, int irq, int value, int vcpu_id)
{
    if (value) {
        gic_dist->enable_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_dist->enable_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->enable_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_dist->enable_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline void set_enable(struct vgic_dist_map *gic_dist, int irq, int value, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        set_sgi_ppi_enable(gic_dist, irq, value, vcpu_id);
        return;
    }
    set_spi_enable(gic_dist, irq, value, vcpu_id);
}

static inline int is_sgi_ppi_enabled(struct vgic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->enable_set0[vcpu_id] & IRQ_BIT(irq));
}

static inline int is_spi_enabled(struct vgic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->enable_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline int is_enabled(struct vgic_dist_map *gic_dist, int irq, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        return is_sgi_ppi_enabled(gic_dist, irq, vcpu_id);
    }
    return is_spi_enabled(gic_dist, irq, vcpu_id);
}

static inline void set_sgi_ppi_active(struct vgic_dist_map *gic_dist, int irq, int value, int vcpu_id)
{
    if (value) {
        gic_dist->active_set0[vcpu_id] |= IRQ_BIT(irq);
    } else {
        gic_dist->active_set0[vcpu_id] &= ~IRQ_BIT(irq);
    }
}

static inline void set_spi_active(struct vgic_dist_map *gic_dist, int irq, int value, int vcpu_id)
{
    if (value) {
        gic_dist->active_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->active_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline void set_active(struct vgic_dist_map *gic_dist, int irq, int value, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        set_sgi_ppi_active(gic_dist, irq, value, vcpu_id);
        return;
    }
    set_spi_active(gic_dist, irq, value, vcpu_id);
}

static inline int is_sgi_ppi_active(struct vgic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->active_set0[vcpu_id] & IRQ_BIT(irq));
}

static inline int is_spi_active(struct vgic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->active_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline int is_active(struct vgic_dist_map *gic_dist, int irq, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        return is_sgi_ppi_active(gic_dist, irq, vcpu_id);
    }
    return is_spi_active(gic_dist, irq, vcpu_id);
}

static struct vgic_dist_map *vgic_get_dist_map(struct vgic_device *d) {
    vgic_t *vgic = vgic_device_get_vgic(d);
    return priv_get_dist(vgic->registers);
}

static int vgic_dist_enable_irq(struct vgic_device *d, vm_vcpu_t *vcpu, int irq)
{
    struct vgic_dist_map *gic_dist = vgic_get_dist_map(d);
    vgic_t *vgic = vgic_device_get_vgic(d);
    DDIST("enabling irq %d\n", irq);
    set_enable(gic_dist, irq, true, vcpu->vcpu_id);
    struct virq_handle *virq_data = virq_find_irq_data(vgic, vcpu, irq);
    if (virq_data) {
        /* STATE b) */
        if (not_pending(gic_dist, virq_data->virq, vcpu->vcpu_id)) {
            virq_ack(vcpu, virq_data);
        }
    } else {
        DDIST("enabled irq %d has no handle", irq);
    }
    return 0;
}

static int vgic_dist_disable_irq(struct vgic_device *d, vm_vcpu_t *vcpu, int irq)
{
    /* STATE g) */
    struct vgic_dist_map *gic_dist = vgic_get_dist_map(d);
    if (irq >= 16) {
        DDIST("disabling irq %d\n", irq);
        set_enable(gic_dist, irq, false, vcpu->vcpu_id);
    }
    return 0;
}

static int vgic_dist_set_pending_irq(struct vgic_device *d, vm_vcpu_t *vcpu, int irq)
{
    /* STATE c) */
    struct vgic_dist_map *gic_dist = vgic_get_dist_map(d);
    vgic_t *vgic = vgic_device_get_vgic(d);

    struct virq_handle *virq_data = virq_find_irq_data(vgic, vcpu, irq);
    /* If the distributor and the IRQ is enabled, inject the IRQ */
    if (virq_data && gic_dist_is_enabled(gic_dist) && is_enabled(gic_dist, irq, vcpu->vcpu_id)) {
        int err;
        DDIST("Pending set: Inject IRQ from pending set (%d)\n", irq);

        set_pending(gic_dist, virq_data->virq, true, vcpu->vcpu_id);
        err = vgic_vcpu_inject_irq(vgic, vcpu, virq_data);
        assert(!err);

        return err;
    } else {
        /* No further action */
        DDIST("IRQ not enabled (%d) on vcpu %d\n", irq, vcpu->vcpu_id);
        return -1;
    }

    return 0;
}

static void vgic_dist_clr_pending_irq(struct vigc_dist_map *gic_dist, vm_vcpu_t *vcpu, int irq)
{
    DDIST("clr pending irq %d\n", irq);
    set_pending(gic_dist, irq, false, vcpu->vcpu_id);
}

static memory_fault_result_t handle_vgic_dist_read_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                         size_t fault_length,
                                                         void *cookie)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    struct vgic_device *d = (struct vgic_device *)cookie;
    struct vgic_dist_map *gic_dist = vgic_get_dist_map(d);
    int offset = fault_get_address(fault) - d->pstart;
    // ZF_LOGE("offset is: %x, d->pstart is %x", d->pstart);
    // ZF_LOGE("fault address is : 0x%x, fault_get_address == 0x%x", fault_addr, fault_get_address(fault));
    uint32_t reg = 0;
    int reg_offset = 0;
    uintptr_t base_reg;
    uint32_t *reg_ptr;
    switch (offset) {
    case RANGE32(GIC_DIST_CTLR, GIC_DIST_CTLR):
        reg = gic_dist_is_enabled(gic_dist);
        break;
    case RANGE32(GIC_DIST_TYPER, GIC_DIST_TYPER):
        reg = gic_dist->typer;
        break;
    case RANGE32(GIC_DIST_IIDR, GIC_DIST_IIDR):
        reg = gic_dist->iidr;
        break;
    case RANGE32(0x00C, 0x01C):
        /* Reserved */
        break;
    case RANGE32(0x020, 0x03C):
        /* Implementation defined */
        break;
    case RANGE32(0x040, 0x07C):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_IGROUPR0, GIC_DIST_IGROUPR0):
        reg = gic_dist->irq_group0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_IGROUPR1, GIC_DIST_IGROUPRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IGROUPR1);
        reg = gic_dist->irq_group[reg_offset];
        break;
    case RANGE32(GIC_DIST_ISENABLER0, GIC_DIST_ISENABLER0):
        reg = gic_dist->enable_set0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ISENABLER1, GIC_DIST_ISENABLERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISENABLER1);
        reg = gic_dist->enable_set[reg_offset];
        break;
    case RANGE32(GIC_DIST_ICENABLER0, GIC_DIST_ICENABLER0):
        reg = gic_dist->enable_clr0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ICENABLER1, GIC_DIST_ICENABLERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICENABLER1);
        reg = gic_dist->enable_clr[reg_offset];
        break;
    case RANGE32(GIC_DIST_ISPENDR0, GIC_DIST_ISPENDR0):
        reg = gic_dist->pending_set0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ISPENDR1, GIC_DIST_ISPENDRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISPENDR1);
        reg = gic_dist->pending_set[reg_offset];
        break;
    case RANGE32(GIC_DIST_ICPENDR0, GIC_DIST_ICPENDR0):
        reg = gic_dist->pending_clr0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ICPENDR1, GIC_DIST_ICPENDRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICPENDR1);
        reg = gic_dist->pending_clr[reg_offset];
        break;
    case RANGE32(GIC_DIST_ISACTIVER0, GIC_DIST_ISACTIVER0):
        reg = gic_dist->active_set0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ISACTIVER1, GIC_DIST_ISACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISACTIVER1);
        reg = gic_dist->active_set[reg_offset];
        break;
    case RANGE32(GIC_DIST_ICACTIVER0, GIC_DIST_ICACTIVER0):
        reg = gic_dist->active_clr0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ICACTIVER1, GIC_DIST_ICACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICACTIVER1);
        reg = gic_dist->active_clr[reg_offset];
        break;
    case RANGE32(GIC_DIST_IPRIORITYR0, GIC_DIST_IPRIORITYR7):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IPRIORITYR0);
        reg = gic_dist->priority0[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(GIC_DIST_IPRIORITYR8, GIC_DIST_IPRIORITYRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IPRIORITYR8);
        reg = gic_dist->priority[reg_offset];
        break;
    case RANGE32(0x7FC, 0x7FC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ITARGETSR0, GIC_DIST_ITARGETSR7):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ITARGETSR0);
        reg = gic_dist->targets0[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(GIC_DIST_ITARGETSR8, GIC_DIST_ITARGETSRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ITARGETSR8);
        reg = gic_dist->targets[reg_offset];
        break;
    case RANGE32(0xBFC, 0xBFC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ICFGR0, GIC_DIST_ICFGRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICFGR0);
        reg = gic_dist->config[reg_offset];
        break;
    case RANGE32(0xDE8, 0xEFC):
        /* Reserved [0xDE8 - 0xE00) */
        /* GIC_DIST_NSACR [0xE00 - 0xF00) - Not supported */
        break;
    case RANGE32(GIC_DIST_SGIR, GIC_DIST_SGIR):
        reg = gic_dist->sgir;
        break;
    case RANGE32(0xF04, 0xF0C):
        /* Implementation defined */
        break;
    case RANGE32(GIC_DIST_CPENDSGIR0, GIC_DIST_CPENDSGIRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_CPENDSGIR0);
        reg = gic_dist->sgi_pending_clr[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(GIC_DIST_SPENDSGIR0, GIC_DIST_SPENDSGIRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_SPENDSGIR0);
        reg = gic_dist->sgi_pending_set[vcpu->vcpu_id][reg_offset];
        break;
    case RANGE32(0xF30, 0xFBC):
        /* Reserved */
        break;
#if defined(GIC_VERSION_3)
    case RANGE32(GIC_DIST_PIDR2, GIC_DIST_PIDR2):
        reg = gic_dist->pidr2;
        break;
#endif
    default:
        ZF_LOGE("Unknown register offset 0x%x\n", offset);
        err = ignore_fault(fault);
        goto fault_return;
    }
    uint32_t mask = fault_get_data_mask(fault);
    fault_set_data(fault, reg & mask);
    err = advance_fault(fault);

fault_return:
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}

static memory_fault_result_t handle_vgic_dist_write_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                          size_t fault_length,
                                                          void *cookie)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    struct vgic_device *d = (struct vgic_device *)cookie;
    struct vgic_dist_map *gic_dist = vgic_get_dist_map(d);
    int offset = fault_get_address(fault) - d->pstart;
    uint32_t reg = 0;
    uint32_t mask = fault_get_data_mask(fault);
    uint32_t reg_offset = 0;
    uint32_t data;
    switch (offset) {
    case RANGE32(GIC_DIST_CTLR, GIC_DIST_CTLR):
#if defined(GIC_VERSION_3)
        data = (fault_get_data(fault) >> 4) & 0x1;
#else
        data = fault_get_data(fault);
#endif
        if (data == 1) {
            gic_dist_enable(gic_dist);
        } else if (data == 0) {
            gic_dist_disable(gic_dist);
        } else {
            ZF_LOGE("Unknown enable register encoding: 0x%x", data);
        }
        break;
    case RANGE32(GIC_DIST_TYPER, GIC_DIST_TYPER):
        break;
    case RANGE32(GIC_DIST_IIDR, GIC_DIST_IIDR):
        break;
    case RANGE32(0x00C, 0x01C):
        /* Reserved */
        break;
    case RANGE32(0x020, 0x03C):
        /* Implementation defined */
        break;
    case RANGE32(0x040, 0x07C):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_IGROUPR0, GIC_DIST_IGROUPR0):
        gic_dist->irq_group0[vcpu->vcpu_id] = fault_emulate(fault, gic_dist->irq_group0[vcpu->vcpu_id]);
        break;
    case RANGE32(GIC_DIST_IGROUPR1, GIC_DIST_IGROUPRN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_IGROUPR1);
        gic_dist->irq_group[reg_offset] = fault_emulate(fault, gic_dist->irq_group[reg_offset]);
        break;
    case RANGE32(GIC_DIST_ISENABLER0, GIC_DIST_ISENABLERN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - GIC_DIST_ISENABLER0) * 8;
            vgic_dist_enable_irq(d, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ICENABLER0, GIC_DIST_ICENABLERN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - GIC_DIST_ICENABLER0) * 8;
            vgic_dist_disable_irq(d, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ISPENDR0, GIC_DIST_ISPENDRN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - GIC_DIST_ISPENDR0) * 8;
            vgic_dist_set_pending_irq(d, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ICPENDR0, GIC_DIST_ICPENDRN):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            irq += (offset - 0x280) * 8;
            vgic_dist_clr_pending_irq(gic_dist, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ISACTIVER0, GIC_DIST_ISACTIVER0):
        gic_dist->active_set0[vcpu->vcpu_id] = fault_emulate(fault, gic_dist->active_set0[vcpu->vcpu_id]);
        break;
    case RANGE32(GIC_DIST_ISACTIVER1, GIC_DIST_ISACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISACTIVER1);
        gic_dist->active_set[reg_offset] = fault_emulate(fault, gic_dist->active_set[reg_offset]);
        break;
    case RANGE32(GIC_DIST_ICACTIVER0, GIC_DIST_ICACTIVER0):
        gic_dist->active_clr0[vcpu->vcpu_id] = fault_emulate(fault, gic_dist->active_clr0[vcpu->vcpu_id]);
        break;
    case RANGE32(GIC_DIST_ICACTIVER1, GIC_DIST_ICACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ICACTIVER1);
        gic_dist->active_clr[reg_offset] = fault_emulate(fault, gic_dist->active_clr[reg_offset]);
        break;
    case RANGE32(GIC_DIST_IPRIORITYR0, GIC_DIST_IPRIORITYRN):
        break;
    case RANGE32(0x7FC, 0x7FC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ITARGETSR0, GIC_DIST_ITARGETSRN):
        break;
    case RANGE32(0xBFC, 0xBFC):
        /* Reserved */
        break;
    case RANGE32(GIC_DIST_ICFGR0, GIC_DIST_ICFGRN):
        /* Not supported */
        break;
    case RANGE32(0xD00, 0xDE4):
        break;
    case RANGE32(0xDE8, 0xEFC):
        /* Reserved [0xDE8 - 0xE00) */
        /* GIC_DIST_NSACR [0xE00 - 0xF00) - Not supported */
        break;
    case RANGE32(GIC_DIST_SGIR, GIC_DIST_SGIR):
        data = fault_get_data(fault);
        int mode = (data & GIC_DIST_SGI_TARGET_LIST_FILTER_MASK) >> GIC_DIST_SGI_TARGET_LIST_FILTER_SHIFT;
        int virq = (data & GIC_DIST_SGI_INTID_MASK);
        uint16_t target_list = 0;
        switch (mode) {
        case GIC_DIST_SGI_TARGET_LIST_SPEC:
            /* Forward virq to vcpus specified in CPUTargetList */
            target_list = (data & GIC_DIST_SGI_CPU_TARGET_LIST_MASK) >> GIC_DIST_SGI_CPU_TARGET_LIST_SHIFT;
            break;
        case GIC_DIST_SGI_TARGET_LIST_OTHERS:
            /* Forward virq to all vcpus but the requesting vcpu */
            target_list = (1 << vcpu->vm->num_vcpus) - 1;
            target_list = target_list & ~(1 << vcpu->vcpu_id);
            break;
        case GIC_DIST_SGI_TARGET_SELF:
            /* Forward to virq to only the requesting vcpu */
            target_list = (1 << vcpu->vcpu_id);
            break;
        default:
            ZF_LOGE("Unknow SGIR Target List Filter mode");
            goto ignore_fault;
        }
        /* Find the VCPUs the IRQ targets and inject it, assuming the VCPU is online. */
        for (int i = 0; i < vcpu->vm->num_vcpus; i++) {
            vm_vcpu_t *target_vcpu = vcpu->vm->vcpus[i];
            if (!(target_list & (1 << i)) || !is_vcpu_online(target_vcpu)) {
                continue;
            }
            vm_inject_irq(target_vcpu, virq);
        }
        break;
    case RANGE32(0xF04, 0xF0C):
        break;
    case RANGE32(GIC_DIST_CPENDSGIR0, GIC_DIST_SPENDSGIRN):
        ZF_LOGE("SGI reg not implemented, disregarding write to 0x%x!\n", offset);
        break;
    case RANGE32(0xF30, 0xFBC):
        /* Reserved */
        break;
    case RANGE32(0xFC0, 0xFFB):
        break;
    case RANGE32(0x6100, 0x7FD8):
        break;
    default:
        ZF_LOGE("Unknown register offset: 0x%x", offset);
    }
ignore_fault:
    err = ignore_fault(fault);
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}

static memory_fault_result_t handle_vgic_dist_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                    size_t fault_length,
                                                    void *cookie)
{
    if (fault_is_read(vcpu->vcpu_arch.fault)) {
        return handle_vgic_dist_read_fault(vm, vcpu, fault_addr, fault_length, cookie);
    }
    return handle_vgic_dist_write_fault(vm, vcpu, fault_addr, fault_length, cookie);
}
