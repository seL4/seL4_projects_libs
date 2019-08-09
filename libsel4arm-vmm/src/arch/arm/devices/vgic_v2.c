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

#include "virq.h"
#include "gicv2.h"
#include "vdist.h"

static struct gic_dist_map *vgic_priv_get_dist(struct device *d) {
    vgic_t *vgic = vgic_device_get_vgic(d);
    return priv_get_dist(vgic->dist);
}

int vgic_vcpu_inject_irq(vgic_t *vgic, seL4_CPtr vcpu, struct virq_handle *irq)
{
    int i = vgic_find_free_irq(vgic);
    seL4_Error err = seL4_ARM_VCPU_InjectIRQ(vcpu, irq->virq, 0, 0, i);
    assert((i < 4) || err);
    if (!err) {
        /* Shadow */
        vgic_shadow_irq(vgic, i, irq);
        return err;
    } else {
        int error = vgic_add_overflow(vgic, irq);
        ZF_LOGF_IF(error, "too many overflow irqs");
        return 0;
    }
}

int handle_vgic_maintenance(vm_t *vm, int idx)
{
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->lock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    /* STATE d) */
    struct device *d = vm_find_device_by_id(vm, DEV_VGIC_DIST);
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);

    assert(d);
    vgic_t *vgic = vgic_device_get_vgic(d);
    assert(vgic->irq[idx]);

    /* Clear pending */
    DIRQ("Maintenance IRQ %d\n", vgic->irq[idx]->virq);
    vgic_dist_set_pending(gic_dist, vgic->irq[idx]->virq, false);
    virq_ack(vgic->irq[idx]);

    vgic->irq[idx] = NULL;
    vgic_handle_overflow(vgic, vm->vcpu.cptr);
#ifdef CONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    vm->unlock();
#endif //CONCONFIG_LIB_SEL4_ARM_VMM_VCHAN_SUPPORT

    return 0;
}


static void vgic_dist_reset(struct device *d)
{
    struct gic_dist_map *gic_dist = vgic_priv_get_dist(d);
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

int vm_inject_IRQ(virq_handle_t virq)
{
    assert(virq);
    vm_t *vm = virq->vm;

    // vm->lock();

    DIRQ("VM received IRQ %d\n", virq->virq);

    /* Grab a handle to the VGIC */
    struct device *vgic_device = vm_find_device_by_id(vm, DEV_VGIC_DIST);
    if (vgic_device == NULL) {
        return -1;
    }

    vgic_t *vgic = vgic_device_get_vgic(vgic_device);
    vgic_dist_set_pending_irq(vgic, vm->vcpu.cptr, virq->virq);

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
    vgic_t *vgic = malloc(sizeof(*vgic));
    if (!vgic) {
        assert(!"Unable to malloc memory for VGIC");
        return -1;
    }
    int err = virq_init(vgic);
    if (err) {
        free(vgic);
        return -1;
    }

    /* Distributor */
    struct device dist = dev_vgic_dist;
    vgic->dist = map_emulated_device(vm, &dev_vgic_dist);
    assert(vgic->dist);
    if (vgic->dist == NULL) {
        return -1;
    }

    dist.priv = (void *)vgic;
    vgic_dist_reset(&dist);
    int err = vm_add_device(vm, &dist);
    if (err) {
        free(dist.priv);
        return -1;
    }

    /* Remap VCPU to CPU */
    struct device vcpu = dev_vgic_vcpu;
    void *addr = map_vm_device(vm, vcpu.pstart, dev_vgic_cpu.pstart, seL4_AllRights);
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
