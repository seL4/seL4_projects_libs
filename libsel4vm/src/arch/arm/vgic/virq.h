#pragma once

#include <sel4vm/guest_irq_controller.h>
#include "vgic.h"

#define MAX_LR_OVERFLOW 64
#define LR_OF_NEXT(_i) (((_i) == MAX_LR_OVERFLOW - 1) ? 0 : ((_i) + 1))

struct virq_handle {
    int virq;
    irq_ack_fn_t ack;
    void *token;
};

struct lr_of {
    struct virq_handle irqs[MAX_LR_OVERFLOW]; /* circular buffer */
    size_t head;
    size_t tail;
    bool full;
};

#define MAX_VIRQS       200
#define NUM_SGI_VIRQS   16
#define NUM_PPI_VIRQS   16
#define GIC_SPI_IRQ_MIN NUM_SGI_VIRQS + NUM_PPI_VIRQS

typedef struct vgic {
/// Mirrors the vcpu list registers
    struct virq_handle *irq[CONFIG_MAX_NUM_NODES][MAX_LR_OVERFLOW - 1];
/// IRQs that would not fit in the vcpu list registers
    struct lr_of lr_overflow[CONFIG_MAX_NUM_NODES];
/// Complete set of virtual irqs
    struct virq_handle *sgi_ppi_irq[CONFIG_MAX_NUM_NODES][NUM_SGI_VIRQS + NUM_PPI_VIRQS];
    struct virq_handle *virqs[MAX_VIRQS];
/// Virtual distributor registers
    void *registers;
} vgic_t;

/* Inject interrupt into vcpu */
int vgic_vcpu_inject_irq(vgic_t *vgic, vm_vcpu_t *inject_vcpu, struct virq_handle *irq);

int handle_vgic_maintenance(vm_vcpu_t *vcpu, int idx);

struct vgic_device *vgic_get_vgic_dist();

static inline int vgic_find_free_irq(vgic_t *vgic, vm_vcpu_t *inject_vcpu) {
    for (int i = 0; i < MAX_LR_OVERFLOW - 1; i++) {
        if (vgic->irq[inject_vcpu->vcpu_id][i] == NULL) {
            return i;
        }
    }

    ZF_LOGE("Could not find free IRQ");
    return -1;
}

static inline struct vgic *vgic_device_get_vgic(struct vgic_device *d)
{
    assert(d);
    assert(d->priv);
    return (vgic_t *)d->priv;
}

static inline struct virq_handle **vgic_priv_get_lr(struct vgic_device *d, vm_vcpu_t *vcpu)
{
    assert(d);
    assert(d->priv);
    return vgic_device_get_vgic(d)->irq[vcpu->vcpu_id];
}

static inline int vgic_add_overflow_cpu(struct lr_of *lr_overflow, struct virq_handle *irq)
{
    /* Add to overflow list */
    int idx = lr_overflow->tail;
    if (unlikely(lr_overflow->full)) {
        ZF_LOGF("too many overflow irqs");
        return -1;
    }

    lr_overflow->irqs[idx] = *irq;
    lr_overflow->full = (lr_overflow->head == LR_OF_NEXT(lr_overflow->tail));
    if (!lr_overflow->full) {
        lr_overflow->tail = LR_OF_NEXT(idx);
    }
    return 0;
}

static inline int vgic_add_overflow(vgic_t *vgic, struct virq_handle *irq, vm_vcpu_t *vcpu)
{
    return vgic_add_overflow_cpu(&vgic->lr_overflow[vcpu->vcpu_id], irq);
}

static inline void vgic_handle_overflow_cpu(vgic_t *vgic, struct lr_of *lr_overflow, vm_vcpu_t *vcpu)
{
    /* copy tail, as vgic_vcpu_inject_irq can mutate it, and we do
     * not want to process any new overflow irqs */
    size_t tail = lr_overflow->tail;
    for (size_t i = lr_overflow->head; i != tail; i = LR_OF_NEXT(i)) {
        if (vgic_vcpu_inject_irq(vgic, vcpu, &lr_overflow->irqs[i]) == 0) {
            lr_overflow->head = LR_OF_NEXT(i);
            lr_overflow->full = (lr_overflow->head == LR_OF_NEXT(lr_overflow->tail));
        } else {
            break;
        }
    }
}

static inline void vgic_handle_overflow(vgic_t *vgic, vm_vcpu_t *vcpu)
{
    vgic_handle_overflow_cpu(vgic, &vgic->lr_overflow[vcpu->vcpu_id], vcpu);
}

static inline void virq_ack(vm_vcpu_t *vcpu, struct virq_handle *irq)
{
    irq->ack(vcpu, irq->virq, irq->token);
}

static inline void virq_init(struct virq_handle *virq, int irq, irq_ack_fn_t ack_fn, void *token)
{
    virq->virq = irq;
    virq->token = token;
    virq->ack = ack_fn;
}

static struct virq_handle *virq_find_sgi_ppi_data(vgic_t *vgic, vm_vcpu_t *vcpu, int virq)
{
    assert(vcpu->vcpu_id < CONFIG_MAX_NUM_NODES);
    return vgic->sgi_ppi_irq[vcpu->vcpu_id][virq];
}

static struct virq_handle *virq_find_spi_irq_data(vgic_t *vgic, int virq)
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
        return virq_find_sgi_ppi_data(vgic, vcpu, virq);
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

    ZF_LOGE("Could not add SPI IRQ");
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
        memset(vgic->irq[i], 0, sizeof(vgic->irq[i]));
        vgic->lr_overflow[i].head = 0;
        vgic->lr_overflow[i].tail = 0;
        vgic->lr_overflow[i].full = false;
        memset(vgic->lr_overflow[i].irqs, 0, sizeof(vgic->lr_overflow[i].irqs));
    }
}

int vm_register_irq(vm_vcpu_t *vcpu, int irq, irq_ack_fn_t ack_fn, void *cookie)
{
    vgic_t *vgic = vgic_device_get_vgic(vgic_get_vgic_dist());

    struct virq_handle *virq_data = calloc(1, sizeof(struct virq_handle));
    if (!virq_data) {
        ZF_LOGE("Could not allocate virq_data");
        return -1;
    }

    virq_init(virq_data, irq, ack_fn, cookie);
    int err = virq_add(vcpu, vgic, virq_data);
    if (err) {
        ZF_LOGE("Could not add vIRQ: %d", irq);
        free(virq_data);
        return -1;
    }

    return 0;
}

int vm_vgic_maintenance_handler(vm_vcpu_t *vcpu)
{
    int idx;
    int err;
    idx = seL4_GetMR(seL4_UnknownSyscall_ARG0);
    /* Currently not handling spurious IRQs */
    assert(idx >= 0);

    err = handle_vgic_maintenance(vcpu, idx);
    if (!err) {
        seL4_MessageInfo_t reply;
        reply = seL4_MessageInfo_new(0, 0, 0, 0);
        seL4_Reply(reply);
    }
    return VM_EXIT_HANDLED;
}

memory_fault_result_t handle_vgic_vcpu_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                    size_t fault_length,
                                                    void *cookie)
{
    /* We shouldn't fault on the vgic vcpu region as it should be mapped in
     * with all rights */
    return FAULT_ERROR;
}

static vm_frame_t vgic_vcpu_iterator(uintptr_t addr, void *cookie)
{
    int err;
    cspacepath_t frame;
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };
    vm_t *vm = (vm_t *)cookie;

    err = vka_cspace_alloc_path(vm->vka, &frame);
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
