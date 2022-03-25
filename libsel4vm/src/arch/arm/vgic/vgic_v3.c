/*
* For GICv3, need memory carve out in DTS. (?)
* Might have to add a `device_map.h` for the imx8mm as well as a `devices.h` file for passthrough stuff?.
* Various TODOs
* Do we need to do anything about SMC and WFI faults?
    * So in the current code we ignore WFI faults, Anna does create new fault type. Probably should figure out what WFI is.
* Need to document this file properly, something like the state machine in vgic_v2.c too.
* priority regs are implementation defined for dist map, have a look.
* Checkout every code base that Anna was using, and try run her imx8mq example
* Check that GICv2 still works fine
* Understnad the LR_NEXT functions etc.
* Understand how overflow works in virq.h

*/

#include <assert.h>
#include <stdint.h>

#include "vm.h"
#include "../fault.h"
#include "virq.h"
#include "vgic_v3.h"
#include "vdist.h"

#define GICR_ISENABLER0     0x0100
#define GICR_ICENABLER0     0x0180

#define GIC_RDIST_PADDR     0x38880000
#define GIC_RDIST_SGI_PADDR (GIC_RDIST_PADDR + 0x10000)

#define GIC_SGI_PPI_IRQ_MAX 31

static struct vgic_device *vgic_dist;

struct vgic_device *vgic_get_vgic_dist() {
    return vgic_dist;
}

static inline void sgi_set_pending(struct vgic_rdist_sgi_map *gic_rdist_sgi, int irq, int v)
{
    if (v) {
        gic_rdist_sgi->ispend[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_rdist_sgi->icpend[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_rdist_sgi->ispend[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_rdist_sgi->icpend[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline int sgi_is_pending(struct vgic_rdist_sgi_map *gic_rdist_sgi, int irq)
{
    return !!(gic_rdist_sgi->ispend[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline void sgi_set_enable(struct vgic_rdist_sgi_map *gic_rdist_sgi, int irq, int v)
{
    if (v) {
        gic_rdist_sgi->isenable[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_rdist_sgi->icenable[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_rdist_sgi->isenable[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_rdist_sgi->icenable[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline int sgi_is_enabled(struct vgic_rdist_sgi_map *gic_rdist_sgi, int irq)
{
    return !!(gic_rdist_sgi->isenable[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline int sgi_is_active(struct vgic_rdist_sgi_map *gic_rdist_sgi, int irq)
{
    return !!(gic_rdist_sgi->isactive[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static void vgic_sgi_disable_irq(struct vgic_rdist_sgi_map *gic_rdist_sgi, int irq)
{
    /* STATE: g) */
    assert(irq <= GIC_SGI_PPI_IRQ_MAX);
    DDIST("SGI disabling IRQ: %d\n", irq);
    sgi_set_enable(gic_rdist_sgi, irq, false);
}

static void vgic_sgi_enable_irq(vgic_t *vgic, vm_vcpu_t *vcpu, struct vgic_rdist_sgi_map *gic_rdist_sgi, int irq)
{
    assert(irq <= GIC_SGI_PPI_IRQ_MAX);

    DDIST("SGI enabling IRQ: %d\n", irq);
    sgi_set_enable(gic_rdist_sgi, irq, true);
    struct virq_handle *virq_data = virq_find_irq_data(vgic, vcpu, irq);
    if (virq_data) {
        /* STATE b) */
        if (sgi_is_pending(gic_rdist_sgi, virq_data->virq)) {
            DDIST("IRQ not pending\n");
            virq_ack(vcpu, virq_data);
        }
    } else {
        DDIST("enabled irq %d has no handle\n", irq);
    }
}

static inline struct vgic_rdist_map *vgic_get_rdist_map(vgic_t *vgic, size_t vcpu_id)
{
    assert(vgic);
    assert(vgic->registers);
    assert(vcpu_id < CONFIG_MAX_NUM_NODES);

    vgic_reg_t *registers = vgic->registers;
    return &registers->rdist[vcpu_id];
}

static inline struct vgic_rdist_sgi_map *vgic_get_rdist_sgi_map(vgic_t *vgic, size_t vcpu_id)
{
    assert(vgic);
    assert(vgic->registers);
    assert(vcpu_id < CONFIG_MAX_NUM_NODES);

    vgic_reg_t *registers = vgic->registers;
    return &registers->rdist_sgi[vcpu_id];
}

static void vgic_dist_reset(struct vgic_device *d)
{
    struct vgic_dist_map *gic_dist = vgic_get_dist_map(d);
    memset(gic_dist, 0, sizeof(*gic_dist));

    gic_dist->typer            = 0x7B04B0; /* RO */
    gic_dist->iidr             = 0x1043B ; /* RO */

    gic_dist->enable_set[0]    = 0x0000ffff; /* 16bit RO */
    gic_dist->enable_clr[0]    = 0x0000ffff; /* 16bit RO */

    gic_dist->config[0]        = 0xaaaaaaaa; /* RO */

    gic_dist->pidr2            = 0x30;     /* RO */

    gic_dist->pidrn[0]         = 0x44;     /* RO */
    gic_dist->pidrn[4]         = 0x92;     /* RO */
    gic_dist->pidrn[5]         = 0xB4;     /* RO */
    gic_dist->pidrn[6]         = 0x3B;     /* RO */

    gic_dist->cidrn[0]         = 0x0D;     /* RO */
    gic_dist->cidrn[1]         = 0xF0;     /* RO */
    gic_dist->cidrn[2]         = 0x05;     /* RO */
    gic_dist->cidrn[3]         = 0xB1;     /* RO */
}

static void vgic_rdist_reset(struct vgic_rdist_map *gic_rdist)
{
    memset(gic_rdist, 0, sizeof(*gic_rdist));

    gic_rdist->iidr            = 0x1143B;  /* RO */
    gic_rdist->typer           = 0x1;      /* RO */

    gic_rdist->pidr0           = 0x93;     /* RO */
    gic_rdist->pidr1           = 0xB4;     /* RO */
    gic_rdist->pidr2           = 0x30;     /* RO */
    gic_rdist->pidr4           = 0x44;     /* RO */

    gic_rdist->cidr0           = 0x0D;     /* RO */
    gic_rdist->cidr1           = 0xF0;     /* RO */
    gic_rdist->cidr2           = 0x05;     /* RO */
    gic_rdist->cidr3           = 0xB1;     /* RO */
}

static void vgic_rdist_sgi_reset(struct vgic_rdist_sgi_map *gic_rdist_sgi)
{
    assert(gic_rdist_sgi);
    memset(gic_rdist_sgi, 0, sizeof(*gic_rdist_sgi));
    gic_rdist_sgi->isactive[0]       = 0xaaaaaaaa;
}

static memory_fault_result_t handle_vgic_rdist_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                          size_t fault_length,
                                                          void *cookie)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    struct vgic_device *d = (struct vgic_device *) cookie;
    vgic_t *vgic = d->priv;
    int offset = fault_get_address(fault) - d->pstart;
    struct vgic_rdist_map *gic_rdist = vgic_get_rdist_map(vgic, vcpu->vcpu_id);

    if (offset < 0 || offset > 0xFFFC) {
        ZF_LOGE("Unknown register offset 0x%x\n", offset);
        err = ignore_fault(fault);
        goto fault_return;
    }

    if (fault_is_read(fault)) {
        uint32_t reg = *(uint32_t *)((uintptr_t)gic_rdist + (offset - (offset % 4)));
        uint32_t mask = fault_get_data_mask(fault);
        fault_set_data(fault, reg & mask);
        return advance_fault(fault);
    } else {
        uint32_t data = fault_get_data(fault);
        ZF_LOGE("Unsupported register offset 0x%x, data: 0x%x", offset, data);
        err = ignore_fault(fault);
        goto fault_return;
    }

fault_return:
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}

static memory_fault_result_t handle_vgic_rdist_sgi_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                          size_t fault_length,
                                                          void *cookie)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    struct vgic_device *d = (struct vgic_device *) cookie;
    vgic_t *vgic = d->priv;

    struct vgic_rdist_sgi_map *gic_rdist_sgi = vgic_get_rdist_sgi_map(vgic, vcpu->vcpu_id);

    int offset = fault_get_address(fault) - d->pstart;
    if (offset < 0x0080 || offset > 0xFFFC) {
        ZF_LOGE("Unknown register offset 0x%x\n", offset);
        err = ignore_fault(fault);
        goto fault_return;
    }

    uint32_t reg = *(uint32_t *)((uintptr_t)gic_rdist_sgi + (offset - (offset % 4)));
    uint32_t mask = fault_get_data_mask(fault);
    if (fault_is_read(fault)) {
        fault_set_data(fault, reg & mask);
        err = advance_fault(fault);
    } else {
        uint32_t data = fault_get_data(fault);
        switch (offset) {
            case RANGE32(GICR_ISENABLER0, GICR_ISENABLER0):
                /* Mask the data to write */
                data &= mask;
                /* Mask bits that are already set */
                // data &= ~reg;

                while (data) {
                    int irq;
                    irq = CTZ(data);
                    data &= ~(1U << irq);
                    int old_irq = irq;
                    irq += (offset - GICR_ISENABLER0) * 8;
                    // ZF_LOGE("irq was: %d, now is: %d", old_irq, irq);
                    vgic_sgi_enable_irq(vgic, vcpu, gic_rdist_sgi, irq);
                }
                break;
            case RANGE32(GICR_ICENABLER0, GICR_ICENABLER0):
                /* Mask the data to write */
                data &= mask;
                /* Mask bits that are already clear */
                // data &= reg;
                while (data) {
                    int irq;
                    irq = CTZ(data);
                    int old_irq = irq;
                    data &= ~(1U << irq);
                    irq += (offset - 0x180) * 8;
                    vgic_sgi_disable_irq(gic_rdist_sgi, irq);
                }
                break;
            default:
                ZF_LOGE("Unsupported register offset 0x%x, data: 0x%x", offset, data);
                break;
        }

        err = ignore_fault(fault);
    }

fault_return:
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}

int vgic_vcpu_inject_irq(vgic_t *vgic, vm_vcpu_t *inject_vcpu, struct virq_handle *irq)
{
    seL4_CPtr vcpu = inject_vcpu->vcpu.cptr;
    int irq_idx = vgic_find_free_irq(vgic, inject_vcpu);

    int err = seL4_ARM_VCPU_InjectIRQ(vcpu, irq->virq, 0, 1, irq_idx);
    assert((irq_idx < 4) || err);
    if (!err) {
        /* Shadow */
        vgic->irq[inject_vcpu->vcpu_id][irq_idx] = irq;
        return err;
    } else {
        /* Add to overflow list */
        int ret_overflow = vgic_add_overflow(vgic, irq, inject_vcpu);
        ZF_LOGF_IF(ret_overflow, "too many overflow irqs");
        return ret_overflow;
    }
}

static int vgic_sgi_set_pending_irq(vgic_t *vgic, vm_vcpu_t *vcpu, int irq)
{
    /* STATE c) */
    struct vgic_dist_map *gic_dist = vgic_get_dist_map(vgic_get_vgic_dist());
    struct virq_handle *virq_data = virq_find_irq_data(vgic, vcpu, irq);

    struct vgic_rdist_sgi_map *gic_rdist_sgi = vgic_get_rdist_sgi_map(vgic, vcpu->vcpu_id);

    /* If it is enabled, inject the IRQ */
    if (virq_data && gic_dist_is_enabled(gic_dist) && sgi_is_enabled(gic_rdist_sgi, irq)) {
        int err;
        DDIST("Pending set: Inject IRQ from pending set (%d)\n", irq);

        sgi_set_pending(gic_rdist_sgi, virq_data->virq, true);
        err = vgic_vcpu_inject_irq(vgic, vcpu, virq_data);
        assert(!err);

        return err;
    } else {
        /* No further action */
        DDIST("IRQ not enabled (%d) on vcpu %d\n", irq, vcpu->vcpu_id);
    }

    return 0;
}

int vm_inject_irq(vm_vcpu_t *vcpu, int irq)
{
    assert(irq);
    DIRQ("VM received IRQ %d\n", irq);

    /* Grab a handle to the VGIC */
    struct vgic_device *gic_dist = vgic_get_vgic_dist();
    vgic_t *vgic = vgic_device_get_vgic(gic_dist);
    if (irq >= GIC_SGI_PPI_IRQ_MAX) {
        vgic_dist_set_pending_irq(gic_dist, vcpu, irq);
    } else {
        vgic_sgi_set_pending_irq(vgic, vcpu, irq);
    }

    if (!fault_handled(vcpu->vcpu_arch.fault) && fault_is_wfi(vcpu->vcpu_arch.fault)) {
        ignore_fault(vcpu->vcpu_arch.fault);
    }

    return 0;
}

int vm_install_vgic_v3(vm_t *vm)
{
    vgic_t *vgic = calloc(1, sizeof(*vgic));
    if (!vgic) {
        ZF_LOGE("Unable to malloc memory for VGIC");
        return -1;
    }

    // Init IRQ and LR overflow fields.
    vgic_virq_init(vgic);

    vgic_reg_t *registers = calloc(1, sizeof(vgic_reg_t));
    vgic->registers = registers;
    if (!vgic->registers) {
        free(vgic);
        ZF_LOGE("Unable to malloc memory for VGIC registers");
        return -1;
    }

    /* Distributor */
    vgic_dist = calloc(1, sizeof(struct vgic_device));
    assert(vgic_dist);
    memcpy(vgic_dist, &dev_vgic_dist, sizeof(struct vgic_device));

    vm_memory_reservation_t *res_vgic_dist = vm_reserve_memory_at(vm, vgic_dist->pstart, vgic_dist->size,
                                                                       handle_vgic_dist_fault, (void *)vgic_dist);
    if (!res_vgic_dist) {
        ZF_LOGE("Could not reserve memory at 0x%lu", dev_vgic_dist.pstart);
        return -1;
    }
    assert(res_vgic_dist);
    vgic_dist->priv = (void *) vgic;
    vgic_dist_reset(vgic_dist);

    /* Redistributor */
    struct vgic_device *vgic_rdist = calloc(1, sizeof(struct vgic_device));
    assert(vgic_rdist);
    memcpy(vgic_rdist, &dev_vgic_rdist, sizeof(struct vgic_device));
    vgic_rdist->priv = (void *) vgic;

    for (size_t i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
        vgic_rdist_reset(&registers->rdist[i]);
    }

    vm_memory_reservation_t *res_vgic_rdist = vm_reserve_memory_at(vm, vgic_rdist->pstart, vgic_rdist->size,
                                                                       handle_vgic_rdist_fault, (void *)vgic_rdist);
    assert(res_vgic_rdist);

    /* Redistributor SGI/PPI */
    struct vgic_device *vgic_rdist_sgi = calloc(1, sizeof(struct vgic_device));
    assert(vgic_rdist_sgi);
    memcpy(vgic_rdist_sgi, &dev_vgic_rdist_sgi, sizeof(struct vgic_device));
    vgic_rdist_sgi->priv = (void *) vgic;

    for (size_t i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
        vgic_rdist_sgi_reset(&registers->rdist_sgi[i]);
    }

    vm_memory_reservation_t *res_vgic_rdist_sgi = vm_reserve_memory_at(vm, vgic_rdist_sgi->pstart, vgic_rdist_sgi->size,
                                                                       handle_vgic_rdist_sgi_fault, (void *)vgic_rdist_sgi);
    assert(res_vgic_rdist_sgi);

    registers->rdist[CONFIG_MAX_NUM_NODES - 1].typer |= 1 << 4;

    ZF_LOGE("finished installing vgic3");

    return 0;
}

int handle_vgic_maintenance(vm_vcpu_t *vcpu, int idx)
{
    struct vgic_dist_map *gic_dist = vgic_get_dist_map(vgic_dist);
    vgic_t *vgic = vgic_device_get_vgic(vgic_dist);
    struct virq_handle **lr;

    assert(gic_dist);
    lr = vgic_priv_get_lr(vgic_dist, vcpu);
    assert(lr[idx]);

    if (lr[idx]->virq >= GIC_SPI_IRQ_MIN) {
        vgic_dist_clr_pending_irq(gic_dist, vcpu, lr[idx]->virq);
    } else {
        // vgic_sgi_set_pending_irq(vgic, vcpu, lr[idx]->virq);
        // set_pending(gic_dist, lr[idx]->virq, false, vcpu->vcpu_id);
        sgi_set_pending(vgic_get_rdist_sgi_map(vgic, vcpu->vcpu_id), lr[idx]->virq, false);
    }


    virq_ack(vcpu, lr[idx]);
    lr[idx] = NULL;
    vgic_handle_overflow(vgic, vcpu);

    return 0;
}

const struct vgic_device dev_vgic_dist = {
    .pstart = GIC_DIST_PADDR,
    .size = 0x10000,
    .priv = NULL,
};

const struct vgic_device dev_vgic_rdist = {
    .pstart = GIC_RDIST_PADDR,
    .size = 0x10000,
    .priv = NULL,
};

const struct vgic_device dev_vgic_rdist_sgi = {
    .pstart = GIC_RDIST_SGI_PADDR,
    .size = 0x10000,
    .priv = NULL,
};
