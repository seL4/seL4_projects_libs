/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
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

#include <sel4vm/gen_config.h>
#include <sel4vm/guest_vm.h>
#include <sel4vm/boot.h>
#include <sel4vm/guest_memory.h>
#include <sel4vm/guest_irq_controller.h>
#include <sel4vm/guest_vm_util.h>

#include "vgicv2_defs.h"
#include "vm.h"
#include "../fault.h"

//#define DEBUG_IRQ
//#define DEBUG_DIST

#ifdef DEBUG_IRQ
#define DIRQ(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DIRQ(...) do{}while(0)
#endif

#ifdef DEBUG_DIST
#define DDIST(...) do{ printf("VDIST: "); printf(__VA_ARGS__); }while(0)
#else
#define DDIST(...) do{}while(0)
#endif

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
#else
#error "Unsupported platform for GIC"
#endif

#ifdef CONFIG_PLAT_QEMU_ARM_VIRT
#define GIC_DIST_PADDR       (GIC_PADDR)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x00010000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x00030000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x00040000)
#else
#define GIC_DIST_PADDR       (GIC_PADDR + 0x1000)
#define GIC_CPU_PADDR        (GIC_PADDR + 0x2000)
#define GIC_VCPU_CNTR_PADDR  (GIC_PADDR + 0x4000)
#define GIC_VCPU_PADDR       (GIC_PADDR + 0x6000)
#endif

#define MAX_VIRQS   200
#define NUM_SGI_VIRQS   16
#define NUM_PPI_VIRQS   16
#define GIC_SPI_IRQ_MIN      (NUM_SGI_VIRQS + NUM_PPI_VIRQS)

/* GIC Distributor register access utilities */
#define GIC_DIST_REGN(offset, reg) ((offset-reg)/sizeof(uint32_t))
#define RANGE32(a, b) a ... b + (sizeof(uint32_t)-1)

#define IRQ_IDX(irq) ((irq) / 32)
#define IRQ_BIT(irq) (1U << ((irq) % 32))

struct virq_handle {
    int virq;
    irq_ack_fn_t ack;
    void *token;
};

typedef struct virq_handle *virq_handle_t;

static inline void virq_ack(vm_vcpu_t *vcpu, struct virq_handle *irq)
{
    irq->ack(vcpu, irq->virq, irq->token);
}

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

/* TODO: A typical number of list registers supported by GIC is four, but not
 * always. One particular way to probe the number of registers is to inject a
 * dummy IRQ with seL4_ARM_VCPU_InjectIRQ(), using LR index high enough to be
 * not supported by any target; the kernel will reply with the supported range
 * of LR indexes.
 */
#define NUM_LIST_REGS 4
/* This is a rather abritrary number, increase if needed. */
#define MAX_IRQ_QUEUE_LEN 64
#define IRQ_QUEUE_NEXT(_i) (((_i) + 1) & (MAX_IRQ_QUEUE_LEN - 1))

static_assert((MAX_IRQ_QUEUE_LEN & (MAX_IRQ_QUEUE_LEN - 1)) == 0,
              "IRQ ring buffer size must be power of two");

struct irq_queue {
    struct virq_handle *irqs[MAX_IRQ_QUEUE_LEN]; /* circular buffer */
    size_t head;
    size_t tail;
};

typedef struct vgic {
/// Mirrors the vcpu list registers
    struct virq_handle *lr_shadow[CONFIG_MAX_NUM_NODES][NUM_LIST_REGS];
/// IRQs that would not fit in the vcpu list registers
    struct irq_queue irq_queue[CONFIG_MAX_NUM_NODES];
/// Complete set of virtual irqs
    struct virq_handle *sgi_ppi_irq[CONFIG_MAX_NUM_NODES][NUM_SGI_VIRQS + NUM_PPI_VIRQS];
    struct virq_handle *virqs[MAX_VIRQS];
/// Virtual distributor registers
    struct gic_dist_map *dist;
} vgic_t;

static struct vgic_dist_device *vgic_dist;

static struct virq_handle *virq_get_sgi_ppi(vgic_t *vgic, vm_vcpu_t *vcpu, int virq)
{
    assert(vcpu->vcpu_id < CONFIG_MAX_NUM_NODES);
    return vgic->sgi_ppi_irq[vcpu->vcpu_id][virq];
}

static struct virq_handle *virq_find_spi_irq_data(struct vgic *vgic, int virq)
{
    for (int i = 0; i < MAX_VIRQS; i++) {
        if (vgic->virqs[i] && vgic->virqs[i]->virq == virq) {
            return vgic->virqs[i];
        }
    }
    return NULL;
}

static struct virq_handle *virq_find_irq_data(struct vgic *vgic, vm_vcpu_t *vcpu, int virq)
{
    if (virq < GIC_SPI_IRQ_MIN)  {
        return virq_get_sgi_ppi(vgic, vcpu, virq);
    }
    return virq_find_spi_irq_data(vgic, virq);
}

static int virq_spi_add(vgic_t *vgic, struct virq_handle *virq_data)
{
    for (int i = 0; i < MAX_VIRQS; i++) {
        if (vgic->virqs[i] == NULL) {
            vgic->virqs[i] = virq_data;
            return 0;
        }
    }
    return -1;
}

static int virq_sgi_ppi_add(vm_vcpu_t *vcpu, vgic_t *vgic, struct virq_handle *virq_data)
{
    if (vgic->sgi_ppi_irq[vcpu->vcpu_id][virq_data->virq] != NULL) {
        ZF_LOGE("VIRQ %d already registered for VCPU %u\n", virq_data->virq, vcpu->vcpu_id);
        return -1;
    }
    vgic->sgi_ppi_irq[vcpu->vcpu_id][virq_data->virq] = virq_data;
    return 0;
}

static int virq_add(vm_vcpu_t *vcpu, vgic_t *vgic, struct virq_handle *virq_data)
{
    int virq = virq_data->virq;
    if (virq < GIC_SPI_IRQ_MIN) {
        return virq_sgi_ppi_add(vcpu, vgic, virq_data);
    }
    return virq_spi_add(vgic, virq_data);
}

static void vgic_virq_init(vgic_t *vgic)
{
    memset(vgic->virqs, 0, sizeof(vgic->virqs));
    for (int i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
        memset(vgic->lr_shadow[i], 0, sizeof(vgic->lr_shadow[i]));
        vgic->irq_queue[i].head = 0;
        vgic->irq_queue[i].tail = 0;
        memset(vgic->irq_queue[i].irqs, 0, sizeof(vgic->irq_queue[i].irqs));
    }
}

static inline void virq_init(virq_handle_t virq, int irq, irq_ack_fn_t ack_fn, void *token)
{
    virq->virq = irq;
    virq->token = token;
    virq->ack = ack_fn;
}

static inline struct vgic *vgic_device_get_vgic(struct vgic_dist_device *d)
{
    assert(d);
    assert(d->priv);
    return (vgic_t *)d->priv;
}

static inline struct gic_dist_map *vgic_priv_get_dist(struct vgic_dist_device *d)
{
    assert(d);
    assert(d->priv);
    return vgic_device_get_vgic(d)->dist;
}

static inline void set_sgi_ppi_pending(struct gic_dist_map *gic_dist, int irq, bool set_pending, int vcpu_id)
{
    if (set_pending) {
        gic_dist->pending_set0[vcpu_id] |= IRQ_BIT(irq);
        gic_dist->pending_clr0[vcpu_id] |= IRQ_BIT(irq);
    } else {
        gic_dist->pending_set0[vcpu_id] &= ~IRQ_BIT(irq);
        gic_dist->pending_clr0[vcpu_id] &= ~IRQ_BIT(irq);
    }
}

static inline void set_spi_pending(struct gic_dist_map *gic_dist, int irq, bool set_pending)
{
    if (set_pending) {
        gic_dist->pending_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_dist->pending_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->pending_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_dist->pending_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline void set_pending(struct gic_dist_map *gic_dist, int irq, bool set_pending, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        set_sgi_ppi_pending(gic_dist, irq, set_pending, vcpu_id);
        return;
    }
    set_spi_pending(gic_dist, irq, set_pending);
}

static inline bool is_sgi_ppi_pending(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->pending_set0[vcpu_id] & IRQ_BIT(irq));
}

static inline bool is_spi_pending(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->pending_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline bool is_pending(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        return is_sgi_ppi_pending(gic_dist, irq, vcpu_id);

    }
    return is_spi_pending(gic_dist, irq);
}

static inline void set_sgi_ppi_enable(struct gic_dist_map *gic_dist, int irq, bool set_enable, int vcpu_id)
{
    if (set_enable) {
        gic_dist->enable_set0[vcpu_id] |= IRQ_BIT(irq);
        gic_dist->enable_clr0[vcpu_id] |= IRQ_BIT(irq);
    } else {
        gic_dist->enable_set0[vcpu_id] &= ~IRQ_BIT(irq);
        gic_dist->enable_clr0[vcpu_id] &= ~IRQ_BIT(irq);
    }
}

static inline void set_spi_enable(struct gic_dist_map *gic_dist, int irq, bool set_enable)
{
    if (set_enable) {
        gic_dist->enable_set[IRQ_IDX(irq)] |= IRQ_BIT(irq);
        gic_dist->enable_clr[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->enable_set[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
        gic_dist->enable_clr[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline void set_enable(struct gic_dist_map *gic_dist, int irq, bool set_enable, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        set_sgi_ppi_enable(gic_dist, irq, set_enable, vcpu_id);
        return;
    }
    set_spi_enable(gic_dist, irq, set_enable);
}

static inline bool is_sgi_ppi_enabled(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->enable_set0[vcpu_id] & IRQ_BIT(irq));
}

static inline bool is_spi_enabled(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->enable_set[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline bool is_enabled(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        return is_sgi_ppi_enabled(gic_dist, irq, vcpu_id);
    }
    return is_spi_enabled(gic_dist, irq);
}

static inline void set_sgi_ppi_active(struct gic_dist_map *gic_dist, int irq, bool set_active, int vcpu_id)
{
    if (set_active) {
        gic_dist->active0[vcpu_id] |= IRQ_BIT(irq);
    } else {
        gic_dist->active0[vcpu_id] &= ~IRQ_BIT(irq);
    }
}

static inline void set_spi_active(struct gic_dist_map *gic_dist, int irq, bool set_active)
{
    if (set_active) {
        gic_dist->active[IRQ_IDX(irq)] |= IRQ_BIT(irq);
    } else {
        gic_dist->active[IRQ_IDX(irq)] &= ~IRQ_BIT(irq);
    }
}

static inline void set_active(struct gic_dist_map *gic_dist, int irq, bool set_active, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        set_sgi_ppi_active(gic_dist, irq, set_active, vcpu_id);
        return;
    }
    set_spi_active(gic_dist, irq, set_active);
}

static inline bool is_sgi_ppi_active(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    return !!(gic_dist->active0[vcpu_id] & IRQ_BIT(irq));
}

static inline bool is_spi_active(struct gic_dist_map *gic_dist, int irq)
{
    return !!(gic_dist->active[IRQ_IDX(irq)] & IRQ_BIT(irq));
}

static inline bool is_active(struct gic_dist_map *gic_dist, int irq, int vcpu_id)
{
    if (irq < GIC_SPI_IRQ_MIN) {
        return is_sgi_ppi_active(gic_dist, irq, vcpu_id);
    }
    return is_spi_active(gic_dist, irq);
}

static inline int vgic_irq_enqueue(vgic_t *vgic, vm_vcpu_t *vcpu, struct virq_handle *irq)
{
    struct irq_queue *q = &vgic->irq_queue[vcpu->vcpu_id];

    if (unlikely(IRQ_QUEUE_NEXT(q->tail) == q->head)) {
        return -1;
    }

    q->irqs[q->tail] = irq;
    q->tail = IRQ_QUEUE_NEXT(q->tail);

    return 0;
}

static inline struct virq_handle *vgic_irq_dequeue(vgic_t *vgic, vm_vcpu_t *vcpu)
{
    struct irq_queue *q = &vgic->irq_queue[vcpu->vcpu_id];
    struct virq_handle *virq = NULL;

    if (q->head != q->tail) {
        virq = q->irqs[q->head];
        q->head = IRQ_QUEUE_NEXT(q->head);
    }

    return virq;
}

static int vgic_find_empty_list_reg(vgic_t *vgic, vm_vcpu_t *vcpu)
{
    for (int i = 0; i < NUM_LIST_REGS; i++) {
        if (vgic->lr_shadow[vcpu->vcpu_id][i] == NULL) {
            return i;
        }
    }

    return -1;
}

static int vgic_vcpu_load_list_reg(vgic_t *vgic, vm_vcpu_t *vcpu, int idx, struct virq_handle *irq)
{
    int err = seL4_ARM_VCPU_InjectIRQ(vcpu->vcpu.cptr, irq->virq, 0, 0, idx);
    if (err) {
        ZF_LOGF("Failure loading vGIC list register (error %d)", err);
        return err;
    }

    vgic->lr_shadow[vcpu->vcpu_id][idx] = irq;

    return 0;
}

int handle_vgic_maintenance(vm_vcpu_t *vcpu, int idx)
{
    /* STATE d) */
    assert(vgic_dist);
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(vgic_dist);
    vgic_t *vgic = vgic_device_get_vgic(vgic_dist);
    struct virq_handle **lr_shadow = vgic->lr_shadow[vcpu->vcpu_id];
    assert(lr_shadow[idx]);

    /* Clear pending */
    DIRQ("Maintenance IRQ %d\n", lr_shadow[idx]->virq);
    set_pending(gic_dist, lr_shadow[idx]->virq, false, vcpu->vcpu_id);
    virq_ack(vcpu, lr_shadow[idx]);
    lr_shadow[idx] = NULL;

    /* Check the overflow list for pending IRQs */
    struct virq_handle *virq = vgic_irq_dequeue(vgic, vcpu);
    if (virq) {
        return vgic_vcpu_load_list_reg(vgic, vcpu, idx, virq);
    }

    return 0;
}

static int vgic_dist_enable(struct vgic_dist_device *d, vm_t *vm)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    DDIST("enabling gic distributor\n");
    gic_dist->enable = 1;
    return 0;
}

static int vgic_dist_disable(struct vgic_dist_device *d, vm_t *vm)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    DDIST("disabling gic distributor\n");
    gic_dist->enable = 0;
    return 0;
}

static int vgic_dist_enable_irq(struct vgic_dist_device *d, vm_vcpu_t *vcpu, int irq)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    vgic_t *vgic = vgic_device_get_vgic(d);
    DDIST("enabling irq %d\n", irq);
    set_enable(gic_dist, irq, true, vcpu->vcpu_id);
    struct virq_handle *virq_data = virq_find_irq_data(vgic, vcpu, irq);
    if (virq_data) {
        /* STATE b) */
        if (!is_pending(gic_dist, virq_data->virq, vcpu->vcpu_id)) {
            virq_ack(vcpu, virq_data);
        }
    } else {
        DDIST("enabled irq %d has no handle", irq);
    }
    return 0;
}

static int vgic_dist_disable_irq(struct vgic_dist_device *d, vm_vcpu_t *vcpu, int irq)
{
    /* STATE g) */
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);

    /* It is IMPLEMENTATION DEFINED if a GIC allows disabling SGIs. Our vGIC
     * implementation does not allows it, such requests are simply ignored.
     * Since it is not uncommon that a guest OS tries disabling SGIs, e.g. as
     * part of the platform initialization, no dedicated messages are logged
     * here to avoid bloating the logs.
     */
    if (irq >= NUM_SGI_VIRQS) {
        DDIST("disabling irq %d\n", irq);
        set_enable(gic_dist, irq, false, vcpu->vcpu_id);
    }
    return 0;
}

static int vgic_dist_set_pending_irq(struct vgic_dist_device *d, vm_vcpu_t *vcpu, int irq)
{
    /* STATE c) */
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    vgic_t *vgic = vgic_device_get_vgic(d);

    struct virq_handle *virq_data = virq_find_irq_data(vgic, vcpu, irq);

    if (!virq_data || !gic_dist->enable || !is_enabled(gic_dist, irq, vcpu->vcpu_id)) {
        DDIST("IRQ not enabled (%d) on vcpu %d\n", irq, vcpu->vcpu_id);
        return -1;
    }

    if (is_pending(gic_dist, virq_data->virq, vcpu->vcpu_id)) {
        return 0;
    }

    DDIST("Pending set: Inject IRQ from pending set (%d)\n", irq);
    set_pending(gic_dist, virq_data->virq, true, vcpu->vcpu_id);

    /* Enqueueing an IRQ and dequeueing it right after makes little sense
     * now, but in the future this is needed to support IRQ priorities.
     */
    int err = vgic_irq_enqueue(vgic, vcpu, virq_data);
    if (err) {
        ZF_LOGF("Failure enqueueing IRQ, increase MAX_IRQ_QUEUE_LEN");
        return -1;
    }

    int idx = vgic_find_empty_list_reg(vgic, vcpu);
    if (idx < 0) {
        /* There were no empty list registers available, but that's not a big
         * deal -- we have already enqueued this IRQ and eventually the vGIC
         * maintenance code will load it to a list register from the queue.
         */
        return 0;
    }

    struct virq_handle *virq = vgic_irq_dequeue(vgic, vcpu);
    assert(virq);

    return vgic_vcpu_load_list_reg(vgic, vcpu, idx, virq);
}

static int vgic_dist_clr_pending_irq(struct vgic_dist_device *d, vm_vcpu_t *vcpu, int irq)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    DDIST("clr pending irq %d\n", irq);
    set_pending(gic_dist, irq, false, vcpu->vcpu_id);
    /* TODO: remove from IRQ queue and list registers as well */
    return 0;
}

static memory_fault_result_t handle_vgic_dist_read_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                         size_t fault_length,
                                                         void *cookie)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    struct vgic_dist_device *d = (struct vgic_dist_device *)cookie;
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    int offset = fault_get_address(fault) - d->pstart;
    uint32_t reg = 0;
    int reg_offset = 0;
    uintptr_t base_reg;
    uint32_t *reg_ptr;
    switch (offset) {
    case RANGE32(GIC_DIST_CTLR, GIC_DIST_CTLR):
        reg = gic_dist->enable;
        break;
    case RANGE32(GIC_DIST_TYPER, GIC_DIST_TYPER):
        reg = gic_dist->ic_type;
        break;
    case RANGE32(GIC_DIST_IIDR, GIC_DIST_IIDR):
        reg = gic_dist->dist_ident;
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
        reg = gic_dist->active0[vcpu->vcpu_id];
        break;
    case RANGE32(GIC_DIST_ISACTIVER1, GIC_DIST_ISACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISACTIVER1);
        reg = gic_dist->active[reg_offset];
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
    case RANGE32(0xD00, 0xDE4):
        base_reg = (uintptr_t) & (gic_dist->spi[0]);
        reg_ptr = (uint32_t *)(base_reg + (offset - 0xD00));
        reg = *reg_ptr;
        break;
    case RANGE32(0xDE8, 0xEFC):
        /* Reserved [0xDE8 - 0xE00) */
        /* GIC_DIST_NSACR [0xE00 - 0xF00) - Not supported */
        break;
    case RANGE32(GIC_DIST_SGIR, GIC_DIST_SGIR):
        reg = gic_dist->sgi_control;
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
    case RANGE32(0xFC0, 0xFFB):
        base_reg = (uintptr_t) & (gic_dist->periph_id[0]);
        reg_ptr = (uint32_t *)(base_reg + (offset - 0xFC0));
        reg = *reg_ptr;
        break;
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
    struct vgic_dist_device *d = (struct vgic_dist_device *)cookie;
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    int offset = fault_get_address(fault) - d->pstart;
    uint32_t reg = 0;
    uint32_t mask = fault_get_data_mask(fault);
    uint32_t reg_offset = 0;
    uint32_t data;
    switch (offset) {
    case RANGE32(GIC_DIST_CTLR, GIC_DIST_CTLR):
        data = fault_get_data(fault);
        if (data == 1) {
            vgic_dist_enable(d, vm);
        } else if (data == 0) {
            vgic_dist_disable(d, vm);
        } else {
            ZF_LOGE("Unknown enable register encoding");
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
            vgic_dist_clr_pending_irq(d, vcpu, irq);
        }
        break;
    case RANGE32(GIC_DIST_ISACTIVER0, GIC_DIST_ISACTIVER0):
        gic_dist->active0[vcpu->vcpu_id] = fault_emulate(fault, gic_dist->active0[vcpu->vcpu_id]);
        break;
    case RANGE32(GIC_DIST_ISACTIVER1, GIC_DIST_ISACTIVERN):
        reg_offset = GIC_DIST_REGN(offset, GIC_DIST_ISACTIVER1);
        gic_dist->active[reg_offset] = fault_emulate(fault, gic_dist->active[reg_offset]);
        break;
    case RANGE32(GIC_DIST_ICACTIVER0, GIC_DIST_ICACTIVER0):
        gic_dist->active_clr0[vcpu->vcpu_id] = fault_emulate(fault, gic_dist->active0[vcpu->vcpu_id]);
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
        assert(!"vgic SGI reg not implemented!\n");
        break;
    case RANGE32(0xF30, 0xFBC):
        /* Reserved */
        break;
    case RANGE32(0xFC0, 0xFFB):
        break;
    default:
        ZF_LOGE("Unknown register offset 0x%x\n", offset);
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

static void vgic_dist_reset(struct vgic_dist_device *d)
{
    struct gic_dist_map *gic_dist;
    gic_dist = vgic_priv_get_dist(d);
    memset(gic_dist, 0, sizeof(*gic_dist));
    gic_dist->ic_type         = 0x0000fce7; /* RO */
    gic_dist->dist_ident      = 0x0200043b; /* RO */

    for (int i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
        gic_dist->enable_set0[i]   = 0x0000ffff; /* 16bit RO */
        gic_dist->enable_clr0[i]   = 0x0000ffff; /* 16bit RO */
    }

    /* Reset value depends on GIC configuration */
    gic_dist->config[0]       = 0xaaaaaaaa; /* RO */
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

    /* Configure per-processor SGI/PPI target registers */
    for (int i = 0; i < CONFIG_MAX_NUM_NODES; i++) {
        for (int j = 0; j < ARRAY_SIZE(gic_dist->targets0[i]); j++) {
            for (int irq = 0; irq < sizeof(uint32_t); irq++) {
                gic_dist->targets0[i][j] |= ((1 << i) << (irq * 8));
            }
        }
    }
    /* Deliver the SPI interrupts to the first CPU interface */
    for (int i = 0; i < ARRAY_SIZE(gic_dist->targets); i++) {
        gic_dist->targets[i] = 0x1010101;
    }

    /* identification */
    gic_dist->periph_id[4]    = 0x00000004; /* RO */
    gic_dist->periph_id[8]    = 0x00000090; /* RO */
    gic_dist->periph_id[9]    = 0x000000b4; /* RO */
    gic_dist->periph_id[10]   = 0x0000002b; /* RO */
    gic_dist->component_id[0] = 0x0000000d; /* RO */
    gic_dist->component_id[1] = 0x000000f0; /* RO */
    gic_dist->component_id[2] = 0x00000005; /* RO */
    gic_dist->component_id[3] = 0x000000b1; /* RO */
}

int vm_register_irq(vm_vcpu_t *vcpu, int irq, irq_ack_fn_t ack_fn, void *cookie)
{
    struct vgic *vgic = vgic_device_get_vgic(vgic_dist);
    assert(vgic);

    struct virq_handle *virq_data = calloc(1, sizeof(*virq_data));
    if (!virq_data) {
        return -1;
    }

    virq_init(virq_data, irq, ack_fn, cookie);

    int err = virq_add(vcpu, vgic, virq_data);
    if (err) {
        free(virq_data);
        return -1;
    }

    return 0;
}

int vm_inject_irq(vm_vcpu_t *vcpu, int irq)
{
    // vm->lock();

    DIRQ("VM received IRQ %d\n", irq);

    int err = vgic_dist_set_pending_irq(vgic_dist, vcpu, irq);

    if (!fault_handled(vcpu->vcpu_arch.fault) && fault_is_wfi(vcpu->vcpu_arch.fault)) {
        ignore_fault(vcpu->vcpu_arch.fault);
    }

    // vm->unlock();

    return err;
}

static memory_fault_result_t handle_vgic_vcpu_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                    size_t fault_length,
                                                    void *cookie)
{
    /* We shouldn't fault on the vgic vcpu region as it should be mapped in
     * with all rights */
    return FAULT_ERROR;
}

static vm_frame_t vgic_vcpu_iterator(uintptr_t addr, void *cookie)
{
    cspacepath_t frame;
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };
    vm_t *vm = (vm_t *)cookie;

    int err = vka_cspace_alloc_path(vm->vka, &frame);
    if (err) {
        printf("Failed to allocate cslot for vgic vcpu\n");
        return frame_result;
    }
    seL4_Word vka_cookie;
    err = vka_utspace_alloc_at(vm->vka, &frame, kobject_get_type(KOBJECT_FRAME, 12), 12, GIC_VCPU_PADDR, &vka_cookie);
    if (err) {
        err = simple_get_frame_cap(vm->simple, (void *)GIC_VCPU_PADDR, 12, &frame);
        if (err) {
            ZF_LOGE("Failed to find device cap for vgic vcpu\n");
            return frame_result;
        }
    }
    frame_result.cptr = frame.capPtr;
    frame_result.rights = seL4_AllRights;
    frame_result.vaddr = GIC_CPU_PADDR;
    frame_result.size_bits = seL4_PageBits;
    return frame_result;
}

/*
 * 1) completely virtual the distributor
 * 2) remap vcpu to cpu. Full access
 */
int vm_install_vgic(vm_t *vm)
{
    struct vgic *vgic = calloc(1, sizeof(*vgic));
    if (!vgic) {
        assert(!"Unable to calloc memory for VGIC");
        return -1;
    }
    vgic_virq_init(vgic);

    /* Distributor */
    vgic_dist = (struct vgic_dist_device *)calloc(1, sizeof(struct vgic_dist_device));
    if (!vgic_dist) {
        return -1;
    }
    memcpy(vgic_dist, &dev_vgic_dist, sizeof(struct vgic_dist_device));

    vgic->dist = calloc(1, sizeof(struct gic_dist_map));
    assert(vgic->dist);
    if (vgic->dist == NULL) {
        return -1;
    }
    vm_memory_reservation_t *vgic_dist_res = vm_reserve_memory_at(vm, GIC_DIST_PADDR, PAGE_SIZE_4K,
                                                                  handle_vgic_dist_fault, (void *)vgic_dist);
    vgic_dist->priv = (void *)vgic;
    vgic_dist_reset(vgic_dist);

    /* Remap VCPU to CPU */
    vm_memory_reservation_t *vgic_vcpu_reservation = vm_reserve_memory_at(vm, GIC_CPU_PADDR,
                                                                          0x1000, handle_vgic_vcpu_fault, NULL);
    int err = vm_map_reservation(vm, vgic_vcpu_reservation, vgic_vcpu_iterator, (void *)vm);
    if (err) {
        free(vgic_dist->priv);
        return -1;
    }

    return 0;
}

int vm_vgic_maintenance_handler(vm_vcpu_t *vcpu)
{
    int idx = seL4_GetMR(seL4_UnknownSyscall_ARG0);
    /* Currently not handling spurious IRQs */
    assert(idx >= 0);

    int err = handle_vgic_maintenance(vcpu, idx);
    if (!err) {
        seL4_MessageInfo_t reply;
        reply = seL4_MessageInfo_new(0, 0, 0, 0);
        seL4_Reply(reply);
    } else {
        ZF_LOGF("vGIC maintenance handler failed (error %d)", err);
    }
    return VM_EXIT_HANDLED;
}

const struct vgic_dist_device dev_vgic_dist = {
    .pstart = GIC_DIST_PADDR,
    .size = 0x1000,
    .priv = NULL,
};
