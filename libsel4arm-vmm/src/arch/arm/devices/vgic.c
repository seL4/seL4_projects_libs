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
/*
 * This component controls and maintains the GIC for the VM.
 * IRQs must be registered at init time with vm_virq_new(...)
 * This function creates and registers an IRQ data structure which will be used for IRQ maintenance
 * b) ENABLING: When the VM enables the IRQ, it checks the pending flag for the VM.
 *   - If the IRQ is not pending, we either
 *        1) have not received an IRQ so it is still enabled in seL4
 *        2) have received an IRQ, but ignored it because the VM had disabled it.
 *     In either case, we simply ACK the IRQ with seL4. In case 1), the IRQ will come straight through,
       in case 2), we have ACKed an IRQ that was not yet pending anyway.
 *   - If the IRQ is already pending, we can assume that the VM has yet to ACK the IRQ and take no further
 *     action.
 *   Transitions: b->c
 * c) PIRQ: When an IRQ is received from seL4, seL4 disables the IRQ and sends an async message. When the VMM
 *    receives the message.
 *   - If the IRQ is enabled, we set the pending flag in the VM and inject the appropriate IRQ
 *     leading to state d)
 *   - If the IRQ is disabled, the VMM takes no further action, leading to state b)
 *   Transitions: (enabled)? c->d :  c->b
 * d) When the VM acknowledges the IRQ, an exception is raised and delivered to the VMM. When the VMM
 *    receives the exception, it clears the pending flag and acks the IRQ with seL4, leading back to state c)
 *    Transition: d->c
 * g) When/if the VM disables the IRQ, we may still have an IRQ resident in the GIC. We allow
 *    this IRQ to be delivered to the VM, but subsequent IRQs will not be delivered as seen by state c)
 *    Transitions g->c
 *
 *   NOTE: There is a big assumption that the VM will not manually manipulate our pending flags and
 *         destroy our state. The affects of this will be an IRQ that is never acknowledged and hence,
 *         will never occur again.
 */

#include "vgic.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <utils/arith.h>
#include <vka/vka.h>
#include <vka/capops.h>

#include "../../../devices.h"
#include "../../../vm.h"
#include "vdist.h"

#define GIC_DIST_PADDR       (GIC_PADDR + 0x1000)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x2000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x4000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x6000)

struct virq_handle {
    int virq;
    void (*ack)(void *token);
    void *token;
    vm_t *vm;
};

static inline vm_t *virq_get_vm(struct virq_handle *irq)
{
    return irq->vm;
}

static inline void virq_ack(struct virq_handle *irq)
{
    irq->ack(irq->token);
}

/* Memory map for GIC distributer */
struct gic_dist_map {
    uint32_t enable;                /* 0x000 */
    uint32_t ic_type;               /* 0x004 */
    uint32_t dist_ident;            /* 0x008 */
    uint32_t res1[29];              /* [0x00C, 0x080) */

    uint32_t security[32];          /* [0x080, 0x100) */

    uint32_t enable_set[32];        /* [0x100, 0x180) */
    uint32_t enable_clr[32];        /* [0x180, 0x200) */
    uint32_t pending_set[32];       /* [0x200, 0x280) */
    uint32_t pending_clr[32];       /* [0x280, 0x300) */
    uint32_t active[32];            /* [0x300, 0x380) */
    uint32_t res2[32];              /* [0x380, 0x400) */

    uint32_t priority[255];         /* [0x400, 0x7FC) */
    uint32_t res3;                  /* 0x7FC */

    uint32_t targets[255];            /* [0x800, 0xBFC) */
    uint32_t res4;                  /* 0xBFC */

    uint32_t config[64];             /* [0xC00, 0xD00) */

    uint32_t spi[32];               /* [0xD00, 0xD80) */
    uint32_t res5[20];              /* [0xD80, 0xDD0) */
    uint32_t res6;                  /* 0xDD0 */
    uint32_t legacy_int;            /* 0xDD4 */
    uint32_t res7[2];               /* [0xDD8, 0xDE0) */
    uint32_t match_d;               /* 0xDE0 */
    uint32_t enable_d;              /* 0xDE4 */
    uint32_t res8[70];               /* [0xDE8, 0xF00) */

    uint32_t sgi_control;           /* 0xF00 */
    uint32_t res9[3];               /* [0xF04, 0xF10) */
    uint32_t sgi_pending_clr[4];    /* [0xF10, 0xF20) */
    uint32_t res10[40];             /* [0xF20, 0xFC0) */

    uint32_t periph_id[12];         /* [0xFC0, 0xFF0) */
    uint32_t component_id[4];       /* [0xFF0, 0xFFF] */
};

#define MAX_LR_OVERFLOW 64
#define LR_OF_NEXT(_i) (((_i) == MAX_LR_OVERFLOW - 1) ? 0 : ((_i) + 1))

struct lr_of {
    struct virq_handle irqs[MAX_LR_OVERFLOW]; /* circular buffer */
    size_t head;
    size_t tail;
    bool full;
};

typedef struct vgic {
/// Mirrors the vcpu list registers
    struct virq_handle *irq[63];
/// IRQs that would not fit in the vcpu list registers
    struct lr_of lr_overflow;
/// Complete set of virtual irqs
    struct virq_handle *virqs[MAX_VIRQS];
/// Virtual distributer registers
    struct gic_dist_map *dist;
} vgic_t;

static struct virq_handle *virq_find_irq_data(vgic_t *vgic, int virq)
{
    int i;
    for (i = 0; i < MAX_VIRQS; i++) {
        if (vgic->virqs[i] && vgic->virqs[i]->virq == virq) {
            return vgic->virqs[i];
        }
    }
    return NULL;
}

static int virq_add(vgic_t *vgic, struct virq_handle *virq_data)
{
    int i;
    for (i = 0; i < MAX_VIRQS; i++) {
        if (vgic->virqs[i] == NULL) {
            vgic->virqs[i] = virq_data;
            return 0;
        }
    }
    return -1;
}

static int virq_init(vgic_t *vgic)
{
    memset(vgic->irq, 0, sizeof(vgic->irq));
    memset(vgic->virqs, 0, sizeof(vgic->virqs));
    vgic->lr_overflow.head = 0;
    vgic->lr_overflow.tail = 0;
    vgic->lr_overflow.full = false;
    return 0;
}

static inline vgic_t *vgic_device_get_vgic(struct device *d)
{
    assert(d);
    assert(d->priv);
    return (vgic_t *)d->priv;
}

static inline struct gic_dist_map *vgic_priv_get_dist(struct device *d)
{
    assert(d);
    assert(d->priv);
    return vgic_device_get_vgic(d)->dist;
}

static inline struct virq_handle **vgic_priv_get_lr(struct device *d)
{
    assert(d);
    assert(d->priv);
    return vgic_device_get_vgic(d)->irq;
}



static inline void set_pending(struct gic_dist_map *gic_dist, int irq, int v)
{
    if (v) {
        gic_dist->pending_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_dist->pending_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->pending_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_dist->pending_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline int is_pending(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->pending_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline void set_enable(struct gic_dist_map *gic_dist, int irq, int v)
{
    if (v) {
        gic_dist->enable_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_dist->enable_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->enable_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_dist->enable_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline int is_enabled(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->enable_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline void set_active(struct gic_dist_map *gic_dist, int irq, int v)
{
    if (v) {
        gic_dist->active[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->active[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline int is_active(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->active[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static int vgic_vcpu_inject_irq(struct device *d, vm_t *vm, struct virq_handle *irq)
{
    vgic_t *vgic;
    int err;
    int i;

    vgic = vgic_device_get_vgic(d);

    seL4_CPtr vcpu;
    vcpu = vm->vcpu.cptr;
    for (i = 0; i < 64; i++) {
        if (vgic->irq[i] == NULL) {
            break;
        }
    }
    err = seL4_ARM_VCPU_InjectIRQ(vcpu, irq->virq, 0, 0, i);
    assert((i < 4) || err);
    if (!err) {
        /* Shadow */
        vgic->irq[i] = irq;
        return err;
    } else {
        /* Add to overflow list */
        int idx = vgic->lr_overflow.tail;
        ZF_LOGF_IF(vgic->lr_overflow.full, "too many overflow irqs");
        vgic->lr_overflow.irqs[idx] = *irq;
        vgic->lr_overflow.full = (vgic->lr_overflow.head == LR_OF_NEXT(vgic->lr_overflow.tail));
        if (!vgic->lr_overflow.full) {
            vgic->lr_overflow.tail = LR_OF_NEXT(idx);
        }
        return 0;
    }
}

int handle_vgic_maintenance(vm_t *vm, int idx)
{
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->lock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    /* STATE d) */
    struct device *d;
    struct gic_dist_map *gic_dist;
    struct virq_handle **lr;

    d = vm_find_device_by_id(vm, DEV_VGIC_DIST);
    assert(d);
    gic_dist = vgic_priv_get_dist(d);
    lr = vgic_priv_get_lr(d);
    assert(lr[idx]);

    /* Clear pending */
    DIRQ("Maintenance IRQ %d\n", lr[idx]->virq);
    set_pending(gic_dist, lr[idx]->virq, false);
    virq_ack(lr[idx]);

    /* Check the overflow list for pending IRQs */
    lr[idx] = NULL;
    vgic_t *vgic = vgic_device_get_vgic(d);
    /* copy tail, as vgic_vcpu_inject_irq can mutate it, and we do
     * not want to process any new overflow irqs */
    size_t tail = vgic->lr_overflow.tail;
    for (size_t i = vgic->lr_overflow.head; i != tail; i = LR_OF_NEXT(i)) {
        if (vgic_vcpu_inject_irq(d, vm, &vgic->lr_overflow.irqs[i]) == 0) {
            vgic->lr_overflow.head = LR_OF_NEXT(i);
            vgic->lr_overflow.full = (vgic->lr_overflow.head == LR_OF_NEXT(vgic->lr_overflow.tail));
        } else {
            break;
        }
    }
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->unlock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    return 0;
}



static int vgic_dist_enable(struct device *d, vm_t *vm)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    DDIST("enabling gic distributer\n");
    gic_dist->enable = 1;
    return 0;
}

static int vgic_dist_disable(struct device *d, vm_t *vm)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    DDIST("disabling gic distributer\n");
    gic_dist->enable = 0;
    return 0;
}

static int vgic_dist_enable_irq(struct device *d, vm_t *vm, int irq)
{
    struct gic_dist_map *gic_dist;
    struct virq_handle *virq_data;
    vgic_t *vgic;
    gic_dist = vgic_priv_get_dist(d);
    vgic = vgic_device_get_vgic(d);
    DDIST("enabling irq %d\n", irq);
    set_enable(gic_dist, irq, true);
    virq_data = virq_find_irq_data(vgic, irq);
    if (virq_data) {
        /* STATE b) */
        if (not_pending(gic_dist, virq_data->virq)) {
            virq_ack(virq_data);
        }
    } else {
        DDIST("enabled irq %d has no handle", irq);
    }
    return 0;
}

static int vgic_dist_disable_irq(struct device *d, vm_t *vm, int irq)
{
    /* STATE g) */
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    if (irq >= 16) {
        DDIST("disabling irq %d\n", irq);
        set_enable(gic_dist, irq, false);
    }
    return 0;
}

static int vgic_dist_set_pending_irq(struct device *d, vm_t *vm, int irq)
{
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->lock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    /* STATE c) */
    struct gic_dist_map *gic_dist;
    vgic_t *vgic;
    struct virq_handle *virq_data;

    gic_dist = vgic_priv_get_dist(d);
    vgic = vgic_device_get_vgic(d);

    virq_data = virq_find_irq_data(vgic, irq);
    /* If it is enables, inject the IRQ */
    if (virq_data && gic_dist->enable && is_enabled(gic_dist, irq)) {
        int err;
        DDIST("Pending set: Inject IRQ from pending set (%d)\n", irq);

        set_pending(gic_dist, virq_data->virq, true);
        err = vgic_vcpu_inject_irq(d, vm, virq_data);
        assert(!err);

#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
        vm->unlock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
        return err;
    } else {
        /* No further action */
        DDIST("IRQ not enabled (%d)\n", irq);
    }

#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->unlock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    return 0;
}

static int vgic_dist_clr_pending_irq(struct device *d, vm_t *vm, int irq)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    DDIST("clr pending irq %d\n", irq);
    set_pending(gic_dist, irq, false);
    return 0;
}

static int handle_vgic_dist_fault(struct device *d, vm_t *vm, fault_t *fault)
{
    struct gic_dist_map *gic_dist;
    int offset;
    enum gic_dist_action act;
    uint32_t mask;
    uint32_t *reg;

    gic_dist = vgic_priv_get_dist(d);
    mask = fault_get_data_mask(fault);
    offset = fault_get_address(fault) - d->pstart;

    offset = ALIGN_DOWN(offset, sizeof(uint32_t));

    reg = (uint32_t *)((uintptr_t)gic_dist + offset);
    act = gic_dist_get_action(offset);

    assert(offset >= 0 && offset < d->size);

    /* Out of range */
    if (offset < 0 || offset >= sizeof(struct gic_dist_map)) {
        DDIST("offset out of range %x %x\n", offset, sizeof(struct gic_dist_map));
        return ignore_fault(fault);

        /* Read fault */
    } else if (fault_is_read(fault)) {
        fault_set_data(fault, *reg);
        return ignore_fault(fault);
    } else {
        uint32_t data;
        switch (act) {
        case ACTION_READONLY:
            return ignore_fault(fault);

        case ACTION_PASSTHROUGH:
            *reg = fault_emulate(fault, *reg);
            return advance_fault(fault);

        case ACTION_ENABLE:
            *reg = fault_emulate(fault, *reg);
            data = fault_get_data(fault);
            if (data == 1) {
                vgic_dist_enable(d, vm);
            } else if (data == 0) {
                vgic_dist_disable(d, vm);
            } else {
                assert(!"Unknown enable register encoding\n");
            }
            return advance_fault(fault);

        case ACTION_ENABLE_SET:
            data = fault_get_data(fault);
            /* Mask the data to write */
            data &= mask;
            /* Mask bits that are already set */
            data &= ~(*reg);
            while (data) {
                int irq;
                irq = CTZ(data);
                data &= ~(1U << irq);
                irq += (offset - 0x100) * 8;
                vgic_dist_enable_irq(d, vm, irq);
            }
            return ignore_fault(fault);

        case ACTION_ENABLE_CLR:
            data = fault_get_data(fault);
            /* Mask the data to write */
            data &= mask;
            /* Mask bits that are already clear */
            data &= *reg;
            while (data) {
                int irq;
                irq = CTZ(data);
                data &= ~(1U << irq);
                irq += (offset - 0x180) * 8;
                vgic_dist_disable_irq(d, vm, irq);
            }
            return ignore_fault(fault);

        case ACTION_PENDING_SET:
            data = fault_get_data(fault);
            /* Mask the data to write */
            data &= mask;
            /* Mask bits that are already set */
            data &= ~(*reg);
            while (data) {
                int irq;
                irq = CTZ(data);
                data &= ~(1U << irq);
                irq += (offset - 0x200) * 8;
                vgic_dist_set_pending_irq(d, vm, irq);
            }
            return ignore_fault(fault);

        case ACTION_PENDING_CLR:
            data = fault_get_data(fault);
            /* Mask the data to write */
            data &= mask;
            /* Mask bits that are already clear */
            data &= *reg;
            while (data) {
                int irq;
                irq = CTZ(data);
                data &= ~(1U << irq);
                irq += (offset - 0x280) * 8;
                vgic_dist_clr_pending_irq(d, vm, irq);
            }
            return ignore_fault(fault);

        case ACTION_SGI:
            assert(!"vgic SGI not implemented!\n");
            return ignore_fault(fault);

        case ACTION_UNKNOWN:
        default:
            DDIST("Unknown action on offset 0x%x\n", offset);
            return ignore_fault(fault);
        }
    }
    abandon_fault(fault);
    return -1;
}

static void vgic_dist_reset(struct device *d)
{
    struct gic_dist_map *gic_dist;
    gic_dist = vgic_priv_get_dist(d);
    memset(gic_dist, 0, sizeof(*gic_dist));
    gic_dist->ic_type         = 0x0000fce7; /* RO */
    gic_dist->dist_ident      = 0x0200043b; /* RO */
    gic_dist->enable_set[0]   = 0x0000ffff; /* 16bit RO */
    gic_dist->enable_clr[0]   = 0x0000ffff; /* 16bit RO */
    gic_dist->config[0]       = 0xaaaaaaaa; /* RO */
    /* Reset value depends on GIC configuration */
    gic_dist->config[1]       = 0x55540000;
    gic_dist->config[2]       = 0x55555555;
    gic_dist->config[3]       = 0x55555555;
    gic_dist->config[4]       = 0x55555555;
    gic_dist->config[5]       = 0x55555555;
    gic_dist->config[6]       = 0x55555555;
    gic_dist->config[7]       = 0x55555555;
    gic_dist->config[8]       = 0x55555555;
    gic_dist->config[9]       = 0x55555555;
    gic_dist->config[10]      = 0x55555555;
    gic_dist->config[11]      = 0x55555555;
    gic_dist->config[12]      = 0x55555555;
    gic_dist->config[13]      = 0x55555555;
    gic_dist->config[14]      = 0x55555555;
    gic_dist->config[15]      = 0x55555555;
    /* identification */
    gic_dist->periph_id[4]    = 0x00000004; /* RO */
    gic_dist->periph_id[8]    = 0x00000090; /* RO */
    gic_dist->periph_id[9]    = 0x000000b4; /* RO */
    gic_dist->periph_id[10]   = 0x0000002b; /* RO */
    gic_dist->component_id[0] = 0x0000000d; /* RO */
    gic_dist->component_id[1] = 0x000000f0; /* RO */
    gic_dist->component_id[2] = 0x00000005; /* RO */
    gic_dist->component_id[3] = 0x000000b1; /* RO */

    /* This tells Linux that all IRQs are routed to CPU0.
     * When we eventually support multiple vCPUs per guest,
     * this will need to be updated.
     */
    for (int i = 0; i < ARRAY_SIZE(gic_dist->targets); i++) {
        gic_dist->targets[i] = 0x01010101;
    }
}

virq_handle_t vm_virq_new(vm_t *vm, int virq, void (*ack)(void *), void *token)
{
    struct virq_handle *virq_data;
    struct device *vgic_device;
    vgic_t *vgic;
    int err;
    vgic_device = vm_find_device_by_id(vm, DEV_VGIC_DIST);
    assert(vgic_device);
    if (!vgic_device) {
        return NULL;
    }
    vgic = vgic_device_get_vgic(vgic_device);
    assert(vgic);

    virq_data = malloc(sizeof(*virq_data));
    if (!virq_data) {
        return NULL;
    }
    virq_data->virq = virq;
    virq_data->token = token;
    virq_data->ack = ack;
    virq_data->vm = vm;
    err = virq_add(vgic, virq_data);
    if (err) {
        free(virq_data);
        return NULL;
    }
    return virq_data;
}

int vm_inject_IRQ(virq_handle_t virq)
{
    struct device *vgic_device;
    vm_t *vm;
    assert(virq);
    vm = virq->vm;

    // vm->lock();

    DIRQ("VM received IRQ %d\n", virq->virq);

    /* Grab a handle to the VGIC */
    vgic_device = vm_find_device_by_id(vm, DEV_VGIC_DIST);
    if (vgic_device == NULL) {
        return -1;
    }

    vgic_dist_set_pending_irq(vgic_device, vm, virq->virq);

    if (!fault_handled(vm->fault) && fault_is_wfi(vm->fault)) {
        ignore_fault(vm->fault);
    }

    // vm->unlock();

    return 0;
}

/*
 * 1) completely virtual the distributor
 * 2) remap vcpu to cpu. Full access
 */
int vm_install_vgic(vm_t *vm)
{
    struct device dist, vcpu;
    vgic_t *vgic;
    void *addr;
    int err;

    vgic = malloc(sizeof(*vgic));
    if (!vgic) {
        assert(!"Unable to malloc memory for VGIC");
        return -1;
    }
    err = virq_init(vgic);
    if (err) {
        free(vgic);
        return -1;
    }

    /* Distributor */
    dist = dev_vgic_dist;
    vgic->dist = map_emulated_device(vm, &dev_vgic_dist);
    assert(vgic->dist);
    if (vgic->dist == NULL) {
        return -1;
    }

    dist.priv = (void *)vgic;
    vgic_dist_reset(&dist);
    err = vm_add_device(vm, &dist);
    if (err) {
        free(dist.priv);
        return -1;
    }

    /* Remap VCPU to CPU */
    vcpu = dev_vgic_vcpu;
    addr = map_vm_device(vm, vcpu.pstart, dev_vgic_cpu.pstart, seL4_AllRights);
    assert(addr);
    if (!addr) {
        free(dist.priv);
        return -1;
    }
    vcpu.pstart = dev_vgic_cpu.pstart;
    err = vm_add_device(vm, &vcpu);
    assert(!err);
    if (err) {
        free(dist.priv);
        return -1;
    }

    return 0;
}

const struct device dev_vgic_dist = {
    .devid = DEV_VGIC_DIST,
    .name = "vgic.distributor",
    .pstart = GIC_DIST_PADDR,
    .size = 0x1000,
    .handle_page_fault = &handle_vgic_dist_fault,
    .priv = NULL,
};

const struct device dev_vgic_cpu = {
    .devid = DEV_VGIC_CPU,
    .name = "vgic.cpu_interface",
    .pstart = GIC_CPU_PADDR,
    .size = 0x1000,
    .handle_page_fault = NULL,
    .priv = NULL,
};

const struct device dev_vgic_vcpu = {
    .devid = DEV_VGIC_VCPU,
    .name = "vgic.vcpu_interface",
    .pstart = GIC_VCPU_PADDR,
    .size = 0x1000,
    .handle_page_fault = NULL,
    .priv = NULL,
};
