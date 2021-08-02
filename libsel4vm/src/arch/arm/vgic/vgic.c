#include "vgic.h"
#include "fault.h"
#include "vm.h"
#include "virq.h"

int vm_install_vgic(vm_t *vm, struct vm_irq_controller_params *params)
{
    switch (params->version) {
        case VM_GIC_V2:
            printf("Install gic v2\n");
            return vm_install_vgic_v2(vm, params);
        case VM_GIC_V3:
            printf("Install gic v3\n");
            return vm_install_vgic_v3(vm, params);
        default:
            break;
    }
    return -1;
}

int vm_inject_irq(vm_vcpu_t *vcpu, int irq)
{
    struct vgic_device *vgic = vcpu->vm->arch.vgic;

    switch (vgic->version) {
        case VM_GIC_V2:
            return vm_v2_inject_irq(vcpu, irq);
        case VM_GIC_V3:
            return vm_v3_inject_irq(vcpu, irq);
        default:
            return -1;
    }
}

int vm_vgic_maintenance_handler(vm_vcpu_t *vcpu)
{
    struct vgic_device *vgic = vcpu->vm->arch.vgic;
    int idx;
    int err;

    idx = seL4_GetMR(seL4_UnknownSyscall_ARG0);
    /* Currently not handling spurious IRQs */
    assert(idx >= 0);

    switch (vgic->version) {
        case VM_GIC_V2:
            err = handle_vgic_v2_maintenance(vcpu, idx);
            break;
        case VM_GIC_V3:
            err = handle_vgic_v3_maintenance(vcpu, idx);
            break;
        default:
            break;
    }
    if (!err) {
        seL4_MessageInfo_t reply;
        reply = seL4_MessageInfo_new(0, 0, 0, 0);
        seL4_Reply(reply);
    }
    return VM_EXIT_HANDLED;
}

void vgic_dist_reset(struct vgic_device *vgic)
{
    switch (vgic->version) {
        case VM_GIC_V2:
            return vgic_dist_v2_reset(&vgic->distributor);
        case VM_GIC_V3:
            return vgic_dist_v3_reset(&vgic->distributor);
        default:
            break;
    }
}

int vm_register_irq(vm_vcpu_t *vcpu, int irq, irq_ack_fn_t ack_fn, void *cookie)
{
    struct virq_handle *virq_data;
    struct vgic_device *vgic;
    int err;

    vgic = vcpu->vm->arch.vgic;
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
