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
#include "../../../vm.h"
#include <sel4arm-vmm/plat/devices.h>

#define MAX_LR_OVERFLOW 64
#define LR_OF_NEXT(_i) ((_i) == MAX_LR_OVERFLOW ? 0 : ((_i) + 1))

struct virq_handle {
    int virq;
    void (*ack)(void *token);
    void *token;
    vm_t *vm;
};

struct lr_of {
    struct virq_handle irqs[MAX_LR_OVERFLOW]; /* circular buffer */
    size_t head;
    size_t tail;
};

struct gic_dist_map;

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

static inline vm_t *virq_get_vm(struct virq_handle *irq)
{
    return irq->vm;
}

static inline void virq_ack(struct virq_handle *irq)
{
    irq->ack(irq->token);
}

static inline struct virq_handle *virq_find_irq_data(vgic_t *vgic, int virq)
{
    int i;
    for (i = 0; i < MAX_VIRQS; i++) {
        if (vgic->virqs[i] && vgic->virqs[i]->virq == virq) {
            return vgic->virqs[i];
        }
    }
    return NULL;
}

static inline int virq_add(vgic_t *vgic, struct virq_handle *virq_data)
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

static inline int virq_init(vgic_t *vgic)
{
    memset(vgic->irq, 0, sizeof(vgic->irq));
    memset(vgic->virqs, 0, sizeof(vgic->virqs));
    vgic->lr_overflow.head = 0;
    vgic->lr_overflow.tail = 0;
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

static inline void vgic_add_overflow(vgic_t *vgic, struct virq_handle *irq)
{
    int idx = LR_OF_NEXT(vgic->lr_overflow.tail);
    ZF_LOGF_IF(idx == vgic->lr_overflow.head && idx != vgic->lr_overflow.tail,
               "too many overflow irqs");
    vgic->lr_overflow.irqs[idx] = *irq;
    vgic->lr_overflow.tail = idx;
}

static inline void vgic_pop_overflow(vgic_t *vgic)
{
    vgic->lr_overflow.head = LR_OF_NEXT(vgic->lr_overflow.head);
}

extern const struct device dev_vgic_dist;
extern const struct device dev_vgic_vcpu;
extern const struct device dev_vgic_cpu;


int handle_vgic_maintenance(vm_t *vm, int idx);
