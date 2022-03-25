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

#ifdef GIC_VERSION_2

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

#include "vm.h"
#include "../fault.h"
#include "virq.h"
#include "vgic_v2.h"
#include "vdist.h"

static struct vgic_device *vgic_dist;

int vgic_vcpu_inject_irq(vgic_t *vgic, vm_vcpu_t *inject_vcpu, struct virq_handle *irq)
{
    seL4_CPtr vcpu = inject_vcpu->vcpu.cptr;
    int irq_idx = vgic_find_free_irq(vgic, inject_vcpu);

    int err = seL4_ARM_VCPU_InjectIRQ(vcpu, irq->virq, 0, 0, irq_idx);
    assert((irq_idx < 4) || err);
    if (!err) {
        /* Shadow */
        vgic->irq[inject_vcpu->vcpu_id][irq_idx] = irq;
        return err;
    } else {
        /* Add to overflow list */
        return vgic_add_overflow(vgic, irq, inject_vcpu);
    }
}

int handle_vgic_maintenance(vm_vcpu_t *vcpu, int idx)
{
    /* STATE d) */
    struct vgic_dist_map *gic_dist;
    struct virq_handle **lr;

    assert(vgic_dist);
    gic_dist = vgic_get_dist_map(vgic_dist);
    lr = vgic_priv_get_lr(vgic_dist, vcpu);
    assert(lr[idx]);

    /* Clear pending */
    DIRQ("Maintenance IRQ %d\n", lr[idx]->virq);
    set_pending(gic_dist, lr[idx]->virq, false, vcpu->vcpu_id);
    virq_ack(vcpu, lr[idx]);

    /* Check the overflow list for pending IRQs */
    lr[idx] = NULL;
    vgic_t *vgic = vgic_device_get_vgic(vgic_dist);
    vgic_handle_overflow(vgic, vcpu);
    return 0;
}

static void vgic_dist_reset(struct vgic_device *d)
{
    struct vgic_dist_map *gic_dist = vgic_get_dist_map(d);
    memset(gic_dist, 0, sizeof(*gic_dist));
    gic_dist->typer           = 0x0000fce7; /* RO */
    gic_dist->iidr            = 0x0200043b; /* RO */

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

int vm_inject_irq(vm_vcpu_t *vcpu, int irq)
{
    DIRQ("VM received IRQ %d\n", irq);

    int err = vgic_dist_set_pending_irq(vgic_dist, vcpu, irq);

    if (!fault_handled(vcpu->vcpu_arch.fault) && fault_is_wfi(vcpu->vcpu_arch.fault)) {
        ignore_fault(vcpu->vcpu_arch.fault);
    }

    return err;
}

/*
 * 1) completely virtual the distributor
 * 2) remap vcpu to cpu. Full access
 */
int vm_install_vgic_v2(vm_t *vm)
{
    vgic_t *vgic = calloc(1, sizeof(*vgic));
    assert(vgic);

    // Init IRQ and LR overflow fields.
    vgic_virq_init(vgic);

    /* Distributor */
    vgic_dist = calloc(1, sizeof(struct vgic_device));
    assert(vgic_dist);
    memcpy(vgic_dist, &dev_vgic_dist, sizeof(struct vgic_device));

    vgic->registers = calloc(1, sizeof(struct vgic_dist_map));
    assert(vgic->registers);

    vm_memory_reservation_t *vgic_dist_res = vm_reserve_memory_at(vm, GIC_DIST_PADDR, PAGE_SIZE_4K,
                                                                  handle_vgic_dist_fault, (void *)vgic_dist);
    vgic_dist->priv = (void *) vgic;
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

struct vgic_device *vgic_get_vgic_dist() {
    return vgic_dist;
}

const struct vgic_device dev_vgic_dist = {
    .pstart = GIC_DIST_PADDR,
    .size = 0x1000,
    .priv = NULL,
};

#endif // #ifdef GIC_VERSION_2
