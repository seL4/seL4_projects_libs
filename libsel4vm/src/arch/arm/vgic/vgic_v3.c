/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
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

#include "vgicv3_defs.h"
#include "vm.h"
#include "../fault.h"
#include "virq.h"
#include "gicv3.h"
#include "vdist.h"


static struct vgic_dist_device *vgic_dist;
static struct vgic_dist_device *vgic_redist;
static struct vgic_dist_device *vgic_redist_sgi;

static struct gic_dist_map *vgic_priv_get_dist(struct vgic_dist_device *d) {
    vgic_t *vgic = d->vgic;
    return priv_get_dist(vgic->registers);
}

static inline struct gic_rdist_map *vgic_priv_get_rdist(struct vgic_dist_device *d)
{
    assert(d);
    assert(d->vgic);
    vgic_t *vgic = d->vgic;
    vgic_reg_t *reg = (vgic_reg_t *) vgic->registers;
    return reg->rdist;
}


static memory_fault_result_t handle_vgic_rdist_read_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                         size_t fault_length,
                                                         void *cookie)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    struct vgic_dist_device *d = (struct vgic_dist_device *)cookie;
    struct vgic *vgic = d->vgic;
    struct gic_dist_map *gic_dist = priv_get_dist(vgic->registers);
    struct gic_rdist_map *gic_rdist = vgic_priv_get_rdist(d);
    int offset = fault_get_address(fault) - d->pstart;
    int vcpu_id = vcpu->vcpu_id;
    uint32_t reg = 0;
    int reg_offset = 0;
    uintptr_t base_reg;
    uint32_t *reg_ptr;
    switch (offset) {
    case RANGE32(GICR_CTLR, GICR_CTLR):
        reg = gic_rdist->ctlr;
        break;
    case RANGE32(GICR_IIDR, GICR_IIDR):
        reg = gic_rdist->iidr;
        break;
    case RANGE32(GICR_TYPER, GICR_TYPER):
        reg = gic_rdist->typer;
        break;
    case RANGE32(GICR_WAKER, GICR_WAKER):
        reg = gic_rdist->waker;
        break;
    case RANGE32(0xFFD0, 0xFFFC):
        base_reg = (uintptr_t) & (gic_rdist->pidr4);
        reg_ptr = (uint32_t *)(base_reg + (offset - 0xFFD0));
        reg = *reg_ptr;
        break;
    case RANGE32(GICR_IGROUPR0, GICR_IGROUPR0):
        base_reg = (uintptr_t) & (gic_dist->irq_group0[vcpu->vcpu_id]);
        reg_ptr = (uint32_t *)(base_reg + (offset - GICR_IGROUPR0));
        reg = *reg_ptr;
        break;
    case RANGE32(GICR_ICFGR1, GICR_ICFGR1):
        base_reg = (uintptr_t) & (gic_dist->config[1]);
        reg_ptr = (uint32_t *)(base_reg + (offset - GICR_ICFGR1));
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


static memory_fault_result_t handle_vgic_rdist_write_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                          size_t fault_length,
                                                          void *cookie)
{
    int err = 0;
    fault_t *fault = vcpu->vcpu_arch.fault;
    struct vgic_dist_device *d = (struct vgic_dist_device *)cookie;
    struct vgic *vgic = d->vgic;
    struct gic_dist_map *gic_dist = priv_get_dist(vgic->registers);
    int offset = fault_get_address(fault) - d->pstart;
    int vcpu_id = vcpu->vcpu_id;
    uint32_t reg = 0;
    uint32_t mask = fault_get_data_mask(fault);
    uint32_t reg_offset = 0;
    uint32_t data;
    switch (offset) {
    case RANGE32(GICR_WAKER, GICR_WAKER):
        /* Writes are ignored */
        break;
    case RANGE32(GICR_IGROUPR0, GICR_IGROUPR0):
        gic_dist->irq_group0[vcpu->vcpu_id] = fault_emulate(fault, gic_dist->irq_group0[vcpu->vcpu_id]);
        break;
    case RANGE32(GICR_ISENABLER0, GICR_ISENABLER0):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            vgic_dist_enable_irq(vgic, vcpu, irq);
        }
        break;
    case RANGE32(GICR_ICENABLER0, GICR_ICENABLER0):
        data = fault_get_data(fault);
        /* Mask the data to write */
        data &= mask;
        while (data) {
            int irq;
            irq = CTZ(data);
            data &= ~(1U << irq);
            set_enable(gic_dist, irq, false, vcpu->vcpu_id);
        }
        break;
    case RANGE32(GICR_ICACTIVER0, GICR_ICACTIVER0):
    // TODO fix this
        gic_dist->active_clr0[vcpu->vcpu_id] = fault_emulate(fault, gic_dist->active0[vcpu->vcpu_id]);
        break;
    case RANGE32(GICR_IPRIORITYR0, GICR_IPRIORITYRN):
        break;


    default:
        ZF_LOGE("Unknown register offset 0x%x, value: 0x%x\n", offset, fault_get_data(fault));
    }
ignore_fault:
    err = ignore_fault(fault);
    if (err) {
        return FAULT_ERROR;
    }
    return FAULT_HANDLED;
}



static memory_fault_result_t handle_vgic_rdist_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                    size_t fault_length,
                                                    void *cookie)
{
    if (fault_is_read(vcpu->vcpu_arch.fault)) {
        return handle_vgic_rdist_read_fault(vm, vcpu, fault_addr, fault_length, cookie);
    }
    return handle_vgic_rdist_write_fault(vm, vcpu, fault_addr, fault_length, cookie);

}



static void vgic_dist_reset(struct vgic_dist_device *d)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
    ZF_LOGF_IF(!gic_dist, "df");
    memset(gic_dist, 0, sizeof(*gic_dist));

    gic_dist->typer            = 0x7B04B0; /* RO */
    gic_dist->iidr             = 0x1043B ; /* RO */

    gic_dist->enable_set[0]    = 0x0000ffff; /* 16bit RO */
    gic_dist->enable_clr[0]    = 0x0000ffff; /* 16bit RO */

    gic_dist->config[0]        = 0xaaaaaaaa; /* RO */

    gic_dist->pidrn[0]         = 0x44;     /* RO */
    gic_dist->pidrn[4]         = 0x92;     /* RO */
    gic_dist->pidrn[5]         = 0xB4;     /* RO */
    gic_dist->pidrn[6]         = 0x3B;     /* RO */

    gic_dist->cidrn[0]         = 0x0D;     /* RO */
    gic_dist->cidrn[1]         = 0xF0;     /* RO */
    gic_dist->cidrn[2]         = 0x05;     /* RO */
    gic_dist->cidrn[3]         = 0xB1;     /* RO */
}

static void vgic_rdist_reset(struct vgic_dist_device *d)
{
    struct gic_rdist_map *gic_rdist = vgic_priv_get_rdist(d);
    ZF_LOGF_IF(!gic_rdist, "df");

    memset(gic_rdist, 0, sizeof(*gic_rdist));

    gic_rdist->typer           = 0x11;      /* RO */
    gic_rdist->iidr            = 0x1143B;  /* RO */

    gic_rdist->pidr0           = 0x93;     /* RO */
    gic_rdist->pidr1           = 0xB4;     /* RO */
    gic_rdist->pidr2           = 0x3B;     /* RO */
    gic_rdist->pidr4           = 0x44;     /* RO */

    gic_rdist->cidr0           = 0x0D;     /* RO */
    gic_rdist->cidr1           = 0xF0;     /* RO */
    gic_rdist->cidr2           = 0x05;     /* RO */
    gic_rdist->cidr3           = 0xB1;     /* RO */


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
        return vgic_vcpu_load_list_reg(vgic, vcpu, idx, 1, virq);
    }

    return 0;
}

int vm_register_irq(vm_vcpu_t *vcpu, int irq, irq_ack_fn_t ack_fn, void *cookie)
{
    struct virq_handle *virq_data;
    struct vgic *vgic;
    int err;

    vgic = vgic_dist->vgic;
    assert(vgic);

    virq_data = calloc(1, sizeof(*virq_data));
    if (!virq_data) {
        return -1;
    }
    virq_init(virq_data, irq, ack_fn, cookie);
    err = virq_add(vcpu, vgic, virq_data);
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

    vgic_t *vgic = vgic_dist->vgic;
    int err = vgic_dist_set_pending_irq(vgic, vcpu, irq);
    if (!fault_handled(vcpu->vcpu_arch.fault) && fault_is_wfi(vcpu->vcpu_arch.fault)) {
        ignore_fault(vcpu->vcpu_arch.fault);
    }

    // vm->unlock();

    return err;
}

const struct vgic_dist_device dev_vgic_dist = {
    .pstart = GIC_DIST_PADDR,
    .size = 0x10000,
    .vgic = NULL,
};


const struct vgic_dist_device dev_vgic_redist = {
    .pstart = GIC_REDIST_PADDR,
    .size = 0xc0000,
    .vgic = NULL,
};




/*
 * 1) completely virtual the distributor
 * 2) remap vcpu to cpu. Full access
 */
int vm_install_vgic(vm_t *vm)
{
    vgic_t *vgic = calloc(1, sizeof(*vgic));
    if (!vgic) {
        ZF_LOGE("Unable to malloc memory for VGIC");
        return -1;
    }

    vgic_reg_t *registers = calloc(1, sizeof(vgic_reg_t));
    vgic->registers = registers;
    if (!vgic->registers) {
        free(vgic);
        ZF_LOGE("Unable to malloc memory for VGIC");
        return -1;
    }

    registers->dist = calloc(1, sizeof(struct gic_dist_map));
    assert(registers->dist);
    if (registers->dist == NULL) {
        goto out;
    }
    registers->rdist = calloc(1, sizeof(struct gic_rdist_map));
    assert(registers->rdist);
    if (registers->rdist == NULL) {
        goto out;
    }


    /* Distributor */
    vgic_dist = (struct vgic_dist_device *)calloc(1, sizeof(struct vgic_dist_device));
    if (!vgic_dist) {
        goto out;
    }
    memcpy(vgic_dist, &dev_vgic_dist, sizeof(struct vgic_dist_device));

    vm_memory_reservation_t *vgic_dist_res = vm_reserve_memory_at(vm, vgic_dist->pstart, vgic_dist->size,
                                                                  handle_vgic_dist_fault, (void *)vgic_dist);
    vgic_dist->vgic = vgic;

    vgic_redist = (struct vgic_dist_device *)calloc(1, sizeof(struct vgic_dist_device));
    if (!vgic_redist) {
        goto out;
    }
    memcpy(vgic_redist, &dev_vgic_redist, sizeof(struct vgic_dist_device));

    vgic_dist_res = vm_reserve_memory_at(vm, vgic_redist->pstart, vgic_redist->size,
                                                                  handle_vgic_rdist_fault, (void *)vgic_redist);
    vgic_redist->vgic = vgic;

    vgic_dist_reset(vgic_dist);
    vgic_rdist_reset(vgic_redist);

    return 0;

out:
    ZF_LOGF("Fail");
    free(vgic->registers);
    free(vgic);
    return -1;
}

int vm_vgic_maintenance_handler(vm_vcpu_t *vcpu)
{
    int idx;
    int err;
    idx = seL4_GetMR(seL4_VGICMaintenance_IDX);
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



