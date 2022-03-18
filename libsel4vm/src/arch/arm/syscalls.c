/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <autoconf.h>
#include <stdio.h>
#include <stdlib.h>

#include <sel4/sel4.h>
#include <utils/util.h>

#include <sel4vm/guest_vm_util.h>
#include <sel4vm/guest_memory.h>

#include "vm.h"
#include "syscalls.h"

static void sys_pa_to_ipa(vm_t *vm, seL4_UserContext *regs)
{
    uint32_t pa;
#ifdef CONFIG_ARCH_AARCH64
#else
    pa = regs->r0;
#endif

    ZF_LOGD("PA translation syscall from [%s]: 0x%08x->?\n", vm->vm_name, pa);
#ifdef CONFIG_ARCH_AARCH64
#else
    regs->r0 = pa;
#endif
}

#if 0
/* sys_ipa_to_pa currently not supported
 * TODO: Re-enable or re-evaluate support for syscall
 */
static void sys_ipa_to_pa(vm_t *vm, seL4_UserContext *regs)
{
    seL4_ARM_Page_GetAddress_t ret;
    long ipa;
    int err;
    seL4_CPtr cap;
#ifdef CONFIG_ARCH_AARCH64
#else
    ipa = regs->r0;
#endif
    cap = vspace_get_cap(vm_get_vspace(vm), (void *)ipa);
    if (cap == seL4_CapNull) {
        err = vm_alloc_guest_ram_at(vm, ipa, 0x1000);
        if (err) {
            printf("Could not map address for IPA translation\n");
            return;
        }
        cap = vspace_get_cap(vm_get_vspace(vm), (void *)ipa);
        assert(cap != seL4_CapNull);
    }

    ret = seL4_ARM_Page_GetAddress(cap);
    assert(!ret.error);
    ZF_LOGD("IPA translation syscall from [%s]: 0x%08x->0x%08x\n",
            vm->vm_name, ipa, ret.paddr);
#ifdef CONFIG_ARCH_AARCH64
#else
    regs->r0 = ret.paddr;
#endif
}
#endif

static void sys_nop(vm_t *vm, seL4_UserContext *regs)
{
    ZF_LOGD("NOP syscall from [%s]\n", vm->vm_name);
}

static int handle_syscall(vm_vcpu_t *vcpu)
{
    seL4_Word syscall, ip;
    seL4_UserContext regs;
    seL4_CPtr tcb;
    vm_t *vm;
    int err;

    vm = vcpu->vm;
    syscall = seL4_GetMR(seL4_UnknownSyscall_Syscall);
    ip = seL4_GetMR(seL4_UnknownSyscall_FaultIP);

    tcb = vm_get_vcpu_tcb(vcpu);
    err = seL4_TCB_ReadRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    regs.pc += 4;

    ZF_LOGI("Syscall %d from [%s]\n", syscall, vm->vm_name);
    switch (syscall) {
    case SYS_PA_TO_IPA:
        sys_pa_to_ipa(vm, &regs);
        break;
#if 0
    case SYS_IPA_TO_PA:
        /* sys_ipa_to_pa currently not supported
         * TODO: Re-enable or re-evaluate support for syscall
         */
        sys_ipa_to_pa(vcpu->vm, &regs);
        break;
#endif
    case SYS_NOP:
        sys_nop(vm, &regs);
        break;
    default:
        ZF_LOGE("%sBad syscall from [%s]: scno %zd at PC: %p%s\n",
                ANSI_COLOR(RED, BOLD), vm->vm_name, syscall, (void *) ip, ANSI_COLOR(RESET));
        return -1;
    }
    err = seL4_TCB_WriteRegisters(tcb, false, 0, sizeof(regs) / sizeof(regs.pc), &regs);
    assert(!err);
    return VM_EXIT_HANDLED;
}

int vm_syscall_handler(vm_vcpu_t *vcpu)
{
    int err;
    err = handle_syscall(vcpu);
    if (!err) {
        seL4_MessageInfo_t reply;
        reply = seL4_MessageInfo_new(0, 0, 0, 0);
        seL4_Reply(reply);
    }
    return err;
}
