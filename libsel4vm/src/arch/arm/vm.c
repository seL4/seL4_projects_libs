/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <autoconf.h>
#include <stdio.h>
#include <stdlib.h>

#include <sel4/sel4.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_vm_util.h>
#include <sel4vm/boot.h>
#include <sel4vm/guest_vm_exits.h>
#include <sel4vm/guest_irq_controller.h>
#include <sel4vm/sel4_arch/processor.h>
#include <sel4vm/arch/guest_arm_context.h>

#include "vm.h"
#include "arm_vm.h"
#include "arm_vm_exits.h"
#include "fault.h"

#include "vgic/vgic.h"
#include "syscalls.h"
#include "mem_abort.h"

static int vm_user_exception_handler(vm_vcpu_t *vcpu);
static int vm_vcpu_handler(vm_vcpu_t *vcpu);
static int vm_unknown_exit_handler(vm_vcpu_t *vcpu);
static int vm_vppi_event_handler(vm_vcpu_t *vcpu);

static vm_exit_handler_fn_t arm_exit_handlers[] = {
    [VM_GUEST_ABORT_EXIT] = vm_guest_mem_abort_handler,
    [VM_SYSCALL_EXIT] = vm_syscall_handler,
    [VM_USER_EXCEPTION_EXIT] = vm_user_exception_handler,
    [VM_VGIC_MAINTENANCE_EXIT] = vm_vgic_maintenance_handler,
    [VM_VCPU_EXIT] = vm_vcpu_handler,
    [VM_VPPI_EXIT] = vm_vppi_event_handler,
    [VM_UNKNOWN_EXIT] = vm_unknown_exit_handler
};

static int vm_decode_exit(seL4_Word label)
{
    int exit_reason = VM_UNKNOWN_EXIT;

    switch (label) {
    case seL4_Fault_VMFault:
        exit_reason = VM_GUEST_ABORT_EXIT;
        break;
    case seL4_Fault_UnknownSyscall:
        exit_reason = VM_SYSCALL_EXIT;
        break;
    case seL4_Fault_UserException:
        exit_reason = VM_USER_EXCEPTION_EXIT;
        break;
    case seL4_Fault_VGICMaintenance:
        exit_reason = VM_VGIC_MAINTENANCE_EXIT;
        break;
    case seL4_Fault_VCPUFault:
        exit_reason = VM_VCPU_EXIT;
        break;
    case seL4_Fault_VPPIEvent:
        exit_reason = VM_VPPI_EXIT;
        break;
    default:
        exit_reason = VM_UNKNOWN_EXIT;
    }
    return exit_reason;
}

static int vm_vppi_event_handler(vm_vcpu_t *vcpu)
{
    int err;
    seL4_Word ppi_irq;
    ppi_irq = seL4_GetMR(0);
    /* We directly inject the interrupt assuming it has been previously registered
     * If not the interrupt will dropped by the VM */
    err = vm_inject_irq(vcpu, ppi_irq);
    if (err) {
        ZF_LOGE("VPPI IRQ %"SEL4_PRId_word" dropped on vcpu %d", ppi_irq, vcpu->vcpu_id);
        /* Acknowledge to unmask it as our guest will not use the interrupt */
        seL4_Error ack_err = seL4_ARM_VCPU_AckVPPI(vcpu->vcpu.cptr, ppi_irq);
        if (ack_err) {
            ZF_LOGE("Failed to ACK VPPI: VPPI Ack invocation failed");
            return -1;
        }
    }
    seL4_MessageInfo_t reply;
    reply = seL4_MessageInfo_new(0, 0, 0, 0);
    seL4_Reply(reply);
    return 0;
}

static int vm_user_exception_handler(vm_vcpu_t *vcpu)
{
    seL4_Word ip = seL4_GetMR(0);
    ZF_LOGE("%sInvalid instruction fault in VM '%s' on vCPU %d at PC 0x"SEL4_PRIx_word"%s",
            ANSI_COLOR(RED, BOLD), vcpu->vm->vm_name, vcpu->vcpu_id, ip, ANSI_COLOR(RESET));

    /* Dump registers */
    seL4_UserContext regs;
    seL4_Error ret = seL4_TCB_ReadRegisters(vm_get_vcpu_tcb(vcpu), false, 0,
                                            sizeof(regs) / sizeof(regs.pc), &regs);
    if (ret != seL4_NoError) {
        ZF_LOGE("Failure reading regs, error %d", ret);
    } else {
        print_ctx_regs(&regs);
    }

    seL4_Reply(seL4_MessageInfo_new(0, 0, 0, 0));
    return VM_EXIT_HANDLED;
}

static void print_unhandled_vcpu_hsr(vm_vcpu_t *vcpu, uint32_t hsr)
{
    printf("======= Unhandled VCPU fault from [%s] =======\n", vcpu->vm->vm_name);
    printf("HSR Value: 0x%08x\n", hsr);
    printf("HSR Exception Class: %s [0x%x]\n", hsr_reasons[HSR_EXCEPTION_CLASS(hsr)], HSR_EXCEPTION_CLASS(hsr));
    printf("Instruction Length: %d\n", HSR_IL(hsr));
    printf("ISS Value: 0x%x\n", hsr & HSR_ISS_MASK);
    printf("==============================================\n");
}

static int vm_vcpu_handler(vm_vcpu_t *vcpu)
{
    uint32_t hsr;
    int err;
    fault_t *fault;
    fault = vcpu->vcpu_arch.fault;
    hsr = seL4_GetMR(seL4_UnknownSyscall_ARG0);
    if (vcpu->vcpu_arch.unhandled_vcpu_callback) {
        /* Pass the vcpu fault to library user in case they can handle it */
        err = new_vcpu_fault(fault, hsr);
        if (err) {
            ZF_LOGE("Failed to create new fault");
            return VM_EXIT_HANDLE_ERROR;
        }
        err = vcpu->vcpu_arch.unhandled_vcpu_callback(vcpu, hsr, vcpu->vcpu_arch.unhandled_vcpu_callback_cookie);
        if (!err) {
            return VM_EXIT_HANDLED;
        }
    }
    print_unhandled_vcpu_hsr(vcpu, hsr);
    return VM_EXIT_HANDLE_ERROR;
}

static int vm_unknown_exit_handler(vm_vcpu_t *vcpu)
{
    /* What? Why are we here? What just happened? */
    ZF_LOGE("Unknown fault from [%s]", vcpu->vm->vm_name);
    vcpu->vm->run.exit_reason = VM_GUEST_UNKNOWN_EXIT;
    return VM_EXIT_HANDLE_ERROR;
}

static int vcpu_stop(vm_vcpu_t *vcpu)
{
    vcpu->vcpu_online = false;
    return seL4_TCB_Suspend(vm_get_vcpu_tcb(vcpu));
}

int vcpu_start(vm_vcpu_t *vcpu)
{
    int err;
    vcpu->vcpu_online = true;
    seL4_Word vmpidr_val;
    seL4_Word vmpidr_reg;

#if CONFIG_MAX_NUM_NODES > 1
#ifdef CONFIG_ARCH_AARCH64
    vmpidr_reg = seL4_VCPUReg_VMPIDR_EL2;
#else
    vmpidr_reg = seL4_VCPUReg_VMPIDR;
#endif
    if (vcpu->vcpu_id == BOOT_VCPU) {
        /*  VMPIDR Bit Assignments [G8.2.167, Arm Architecture Reference Manual Armv8]
         * - BIT(24): Performance of PEs (processing element) at the lowest affinity level is very interdependent
         * - BIT(31): This implementation includes the ARMv7 Multiprocessing Extensions functionality
         */
        vmpidr_val = BIT(24) | BIT(31);
    } else {
        vmpidr_val = vcpu->target_cpu;
    }
    err = vm_set_arm_vcpu_reg(vcpu, vmpidr_reg, vmpidr_val);
    if (err) {
        ZF_LOGE("Failed to set VMPIDR register");
        return -1;
    }
#endif
    return seL4_TCB_Resume(vm_get_vcpu_tcb(vcpu));
}

int vm_register_unhandled_vcpu_fault_callback(vm_vcpu_t *vcpu, unhandled_vcpu_fault_callback_fn vcpu_fault_callback,
                                              void *cookie)
{
    if (!vcpu) {
        ZF_LOGE("Failed to register fault callback: Invalid VCPU handle");
        return -1;
    }

    if (!vcpu_fault_callback) {
        ZF_LOGE("Failed to register vcpu fault callback: Invalid callback");
        return -1;
    }
    vcpu->vcpu_arch.unhandled_vcpu_callback = vcpu_fault_callback;
    vcpu->vcpu_arch.unhandled_vcpu_callback_cookie = cookie;
    return 0;

}

int vm_run_arch(vm_t *vm)
{
    int err;
    int ret;

    ret = 1;
    /* Loop, handling events */
    while (ret > 0) {
        seL4_MessageInfo_t tag;
        seL4_Word sender_badge;
        seL4_Word label;
        int vm_exit_reason;

        tag = seL4_Recv(vm->host_endpoint, &sender_badge);
        label = seL4_MessageInfo_get_label(tag);
        if (sender_badge >= MIN_VCPU_BADGE && sender_badge <= MAX_VCPU_BADGE) {
            seL4_Word vcpu_idx = VCPU_BADGE_IDX(sender_badge);
            if (vcpu_idx >= vm->num_vcpus) {
                ZF_LOGE("Invalid VCPU index. Exiting");
                ret = -1;
            } else {
                vm_exit_reason = vm_decode_exit(label);
                ret = arm_exit_handlers[vm_exit_reason](vm->vcpus[vcpu_idx]);
                if (ret == VM_EXIT_HANDLE_ERROR) {
                    vm->run.exit_reason = VM_GUEST_ERROR_EXIT;
                }
            }
        } else {
            if (vm->run.notification_callback) {
                err = vm->run.notification_callback(vm, sender_badge, tag,
                                                    vm->run.notification_callback_cookie);
            } else {
                ZF_LOGE("Unable to handle VM notification. Exiting");
                err = -1;
            }
            if (err) {
                ret = -1;
                vm->run.exit_reason = VM_GUEST_ERROR_EXIT;
            }
        }
    }

    return ret;
}
