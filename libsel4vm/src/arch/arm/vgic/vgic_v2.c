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
#include "virq.h"
#include "gicv2.h"
#include "vdist.h"


static struct vgic_dist_device *vgic_dist;

static inline struct gic_dist_map *vgic_priv_get_dist(struct vgic_dist_device *d)
{
    assert(d);
    assert(d->vgic);
    return d->vgic->dist;
}


int handle_vgic_maintenance(vm_vcpu_t *vcpu, int idx)
{
    /* STATE d) */
    assert(vgic_dist);
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(vgic_dist);
    vgic_t *vgic = vgic_dist->vgic;
    assert(vgic);
    vgic_vcpu_t *vgic_vcpu = get_vgic_vcpu(vgic, vcpu->vcpu_id);
    assert(vgic_vcpu);
    assert((idx >= 0) && (idx < ARRAY_SIZE(vgic_vcpu->lr_shadow)));
    virq_handle_t *slot = &vgic_vcpu->lr_shadow[idx];
    assert(*slot);
    virq_handle_t lr_virq = *slot;
    *slot = NULL;
    /* Clear pending */
    DIRQ("Maintenance IRQ %d\n", lr_virq->virq);
    set_pending(gic_dist, lr_virq->virq, false, vcpu->vcpu_id);
    virq_ack(vcpu, lr_virq);

    /* Check the overflow list for pending IRQs */
    struct virq_handle *virq = vgic_irq_dequeue(vgic, vcpu);
    if (virq) {
        return vgic_vcpu_load_list_reg(vgic, vcpu, idx, 0, virq);
    }

    return 0;
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
    struct vgic *vgic = vgic_dist->vgic;
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

    struct vgic *vgic = vgic_dist->vgic;
    assert(vgic);

    DIRQ("VM received IRQ %d\n", irq);

    int err = vgic_dist_set_pending_irq(vgic, vcpu, irq);

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
        ZF_LOGE("Failed to allocate cslot for vgic vcpu");
        return frame_result;
    }
    seL4_Word vka_cookie;
    err = vka_utspace_alloc_at(vm->vka, &frame, kobject_get_type(KOBJECT_FRAME, 12), 12, GIC_VCPU_PADDR, &vka_cookie);
    if (err) {
        err = simple_get_frame_cap(vm->simple, (void *)GIC_VCPU_PADDR, 12, &frame);
        if (err) {
            ZF_LOGE("Failed to find device cap for vgic vcpu");
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
    /* vgic doesn't require further initialization, having all fields set to
     * zero is fine.
     */

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
    vgic_dist->vgic = vgic;
    vgic_dist_reset(vgic_dist);

    /* Remap VCPU to CPU */
    vm_memory_reservation_t *vgic_vcpu_reservation = vm_reserve_memory_at(vm, GIC_CPU_PADDR, PAGE_SIZE_4K,
                                                                          handle_vgic_vcpu_fault, NULL);
    int err = vm_map_reservation(vm, vgic_vcpu_reservation, vgic_vcpu_iterator, (void *)vm);
    if (err) {
        free(vgic_dist->vgic);
        return -1;
    }

    return 0;
}

int vm_vgic_maintenance_handler(vm_vcpu_t *vcpu)
{
    int idx = seL4_GetMR(seL4_VGICMaintenance_IDX);
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
    .vgic = NULL,
};
