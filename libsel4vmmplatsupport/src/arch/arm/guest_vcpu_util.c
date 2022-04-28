/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vm/guest_vm.h>
#include <sel4vm/boot.h>
#include <sel4vm/guest_irq_controller.h>

#include <sel4vmmplatsupport/guest_vcpu_util.h>
#include <sel4vmmplatsupport/arch/guest_vcpu_fault.h>
#include <sel4vmmplatsupport/arch/irq_defs.h>

#include <libfdt.h>
#include <fdtgen.h>

#define MAX_CPU_NAME_LENGTH 8

#define FDT_OP(op)                                      \
    do {                                                \
        int err = (op);                                 \
        ZF_LOGF_IF(err < 0, "FDT operation failed");    \
    } while(0)                                          \

static void vppi_event_ack(vm_vcpu_t *vcpu, int irq, void *cookie)
{
    seL4_Error err = seL4_ARM_VCPU_AckVPPI(vcpu->vcpu.cptr, (seL4_Word)irq);
    if (err) {
        ZF_LOGE("Failed to ACK VPPI: VCPU VPPIAck invocation failed");
    }
}

static void sgi_ack(vm_vcpu_t *vcpu, int irq, void *cookie) {}

vm_vcpu_t *create_vmm_plat_vcpu(vm_t *vm, int priority)
{
    int err;
    vm_vcpu_t *vm_vcpu = vm_create_vcpu(vm, priority);
    if (vm_vcpu == NULL) {
        ZF_LOGE("Failed to create platform vcpu");
        return NULL;
    }
    err = vm_register_unhandled_vcpu_fault_callback(vm_vcpu, vmm_handle_arm_vcpu_exception, NULL);
    if (err) {
        ZF_LOGE("Failed to register vcpu platform fault callback handlers");
        return NULL;
    }
    err = vm_register_irq(vm_vcpu, PPI_VTIMER_IRQ, &vppi_event_ack, NULL);
    if (err == -1) {
        ZF_LOGE("Failed to register vcpu virtual time irq");
        return NULL;
    }

    err = vm_register_irq(vm_vcpu, SGI_RESCHEDULE_IRQ, &sgi_ack, NULL);
    if (err == -1) {
        ZF_LOGE("Failed to register vcpu sgi 0 irq");
        return NULL;
    }

    err = vm_register_irq(vm_vcpu, SGI_FUNC_CALL, &sgi_ack, NULL);
    if (err == -1) {
        ZF_LOGE("Failed to register vcpu sgi 1 irq");
        return NULL;
    }

    return vm_vcpu;
}

static int generate_psci_node(void *fdt, int root_offset)
{
    int psci_node = fdt_add_subnode(fdt, root_offset, "psci");
    if (psci_node < 0) {
        return psci_node;
    }
    FDT_OP(fdt_appendprop_u32(fdt, psci_node, "cpu_off", 0x84000002));
    FDT_OP(fdt_appendprop_u32(fdt, psci_node, "cpu_on", 0xc4000003));
    FDT_OP(fdt_appendprop_u32(fdt, psci_node, "cpu_suspend", 0xc4000001));
    FDT_OP(fdt_appendprop_string(fdt, psci_node, "method", "smc"));
    FDT_OP(fdt_appendprop_string(fdt, psci_node, "compatible", "arm,psci-1.0"));
    FDT_OP(fdt_appendprop_string(fdt, psci_node, "status", "okay"));
    return 0;
}

int fdt_generate_plat_vcpu_node(vm_t *vm, void *fdt)
{
    int root_offset = fdt_path_offset(fdt, "/");
    int cpu_node = fdt_add_subnode(fdt, root_offset, "cpus");
    if (cpu_node < 0) {
        return cpu_node;
    }
    FDT_OP(fdt_appendprop_u32(fdt, cpu_node, "#address-cells", 0x1));
    FDT_OP(fdt_appendprop_u32(fdt, cpu_node, "#size-cells", 0x0));
    for (int i = 0; i < vm->num_vcpus; i++) {
        vm_vcpu_t *vcpu = vm->vcpus[i];
        char cpu_name[MAX_CPU_NAME_LENGTH];
        snprintf(cpu_name, MAX_CPU_NAME_LENGTH, "cpu@%x", vcpu->vcpu_id);
        int sub_cpu_node = fdt_add_subnode(fdt, cpu_node, cpu_name);
        if (sub_cpu_node < 0) {
            return sub_cpu_node;
        }
        FDT_OP(fdt_appendprop_string(fdt, sub_cpu_node, "device_type", "cpu"));
        FDT_OP(fdt_appendprop_string(fdt, sub_cpu_node, "compatible", PLAT_CPU_COMPAT));
        FDT_OP(fdt_appendprop_u32(fdt, sub_cpu_node, "reg", vcpu->vcpu_id));
        if (vm->num_vcpus > 1) {
            FDT_OP(fdt_appendprop_string(fdt, sub_cpu_node, "enable-method", "psci"));
        }
    }
    int ret = 0;
    if (vm->num_vcpus > 1) {
        ret = generate_psci_node(fdt, root_offset);
    }
    return ret;
}
