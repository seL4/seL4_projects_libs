/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

/* vm exits and general handling of interrupt injection */

#include <stdio.h>
#include <stdlib.h>

#include <sel4/sel4.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/boot.h>

#include "vm.h"
#include "i8259/i8259.h"
#include "guest_state.h"
#include "processor/decode.h"
#include "processor/lapic.h"
#include "interrupt.h"

#define TRAMPOLINE_LENGTH (100)

static void resume_guest(vm_vcpu_t *vcpu)
{
    /* Disable exit-for-interrupt in guest state to allow the guest to resume. */
    uint32_t state = vm_guest_state_get_control_ppc(vcpu->vcpu_arch.guest_state);
    state &= ~BIT(2); /* clear the exit for interrupt flag */
    vm_guest_state_set_control_ppc(vcpu->vcpu_arch.guest_state, state);
}

static void inject_irq(vm_vcpu_t *vcpu, int irq)
{
    /* Inject a vectored exception into the guest */
    assert(irq >= 16);
    vm_guest_state_set_control_entry(vcpu->vcpu_arch.guest_state, BIT(31) | irq);
}

void vm_inject_exception(vm_vcpu_t *vcpu, int exception, int has_error, uint32_t error_code)
{
    assert(exception < 16);
    // ensure we are not already injecting an interrupt or exception
    uint32_t int_control = vm_guest_state_get_control_entry(vcpu->vcpu_arch.guest_state);
    if ((int_control & BIT(31)) != 0) {
        ZF_LOGF("Cannot inject exception");
    }
    if (has_error) {
        seL4_Word code_to_set = error_code;
        vm_guest_state_set_entry_exception_error_code(vcpu->vcpu_arch.guest_state, code_to_set);
    }
    vm_guest_state_set_control_entry(vcpu->vcpu_arch.guest_state, BIT(31) | exception | 3 << 8 | (has_error ? BIT(11) : 0));
}

void wait_for_guest_ready(vm_vcpu_t *vcpu)
{
    /* Request that the guest exit at the earliest point that we can inject an interrupt. */
    uint32_t state = vm_guest_state_get_control_ppc(vcpu->vcpu_arch.guest_state);
    state |= BIT(2); /* set the exit for interrupt flag */
    vm_guest_state_set_control_ppc(vcpu->vcpu_arch.guest_state, state);
}

int can_inject(vm_vcpu_t *vcpu)
{
    uint32_t rflags = vm_guest_state_get_rflags(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
    uint32_t guest_int = vm_guest_state_get_interruptibility(vcpu->vcpu_arch.guest_state, vcpu->vcpu.cptr);
    uint32_t int_control = vm_guest_state_get_control_entry(vcpu->vcpu_arch.guest_state);

    /* we can only inject if the interrupt mask flag is not set in flags,
       guest is not in an uninterruptable state and we are not already trying to
       inject an interrupt */

    if ((rflags & BIT(9)) && (guest_int & 0xF) == 0 && (int_control & BIT(31)) == 0) {
        return 1;
    }
    return 0;
}

/* This function is called by the local apic when a new interrupt has occured. */
void vm_have_pending_interrupt(vm_vcpu_t *vcpu)
{
    if (vm_apic_has_interrupt(vcpu) >= 0) {
        /* there is actually an interrupt to inject */
        if (can_inject(vcpu)) {
            if (vcpu->vcpu_arch.guest_state->virt.interrupt_halt) {
                /* currently halted. need to put the guest
                 * in a state where it can inject again */
                wait_for_guest_ready(vcpu);
                vcpu->vcpu_arch.guest_state->virt.interrupt_halt = 0;
            } else {
                int irq = vm_apic_get_interrupt(vcpu);
                inject_irq(vcpu, irq);
                /* see if there are more */
                if (vm_apic_has_interrupt(vcpu) >= 0) {
                    wait_for_guest_ready(vcpu);
                }
            }
        } else {
            wait_for_guest_ready(vcpu);
            if (vcpu->vcpu_arch.guest_state->virt.interrupt_halt) {
                vcpu->vcpu_arch.guest_state->virt.interrupt_halt = 0;
            }
        }
    }
}

int vm_pending_interrupt_handler(vm_vcpu_t *vcpu)
{
    /* see if there is actually a pending interrupt */
    assert(can_inject(vcpu));
    int irq = vm_apic_get_interrupt(vcpu);
    if (irq == -1) {
        resume_guest(vcpu);
    } else {
        /* inject the interrupt */
        inject_irq(vcpu, irq);
        if (!(vm_apic_has_interrupt(vcpu) >= 0)) {
            resume_guest(vcpu);
        }
        vcpu->vcpu_arch.guest_state->virt.interrupt_halt = 0;
    }
    return VM_EXIT_HANDLED;
}

/* Start an AP vcpu after a sipi with the requested vector */
void vm_start_ap_vcpu(vm_vcpu_t *vcpu, unsigned int sipi_vector)
{
    ZF_LOGD("trying to start vcpu %d\n", vcpu->vcpu_id);

    uint16_t segment = sipi_vector * 0x100;
    uintptr_t eip = sipi_vector * 0x1000;
    guest_state_t *gs = vcpu->vcpu_arch.guest_state;

    /* Emulate up to 100 bytes of trampoline code */
    uint8_t instr[TRAMPOLINE_LENGTH];
    vm_fetch_instruction(vcpu, eip, vm_guest_state_get_cr3(gs, vcpu->vcpu.cptr),
                         TRAMPOLINE_LENGTH, instr);

    eip = vm_emulate_realmode(vcpu, instr, &segment, eip,
                              TRAMPOLINE_LENGTH, gs, 0);

#ifdef CONFIG_ARCH_X86_64
    /* 64-bit guests go from realmode to 32-bit emulation mode to longmode */
    memset(instr, 0, TRAMPOLINE_LENGTH);

    vm_fetch_instruction(vcpu, eip, vm_guest_state_get_cr3(gs, vcpu->vcpu.cptr),
                         TRAMPOLINE_LENGTH, instr);

    eip = vm_emulate_realmode(vcpu, instr, &segment, eip,
                              TRAMPOLINE_LENGTH, gs, 1);
#endif

    vm_guest_state_set_eip(vcpu->vcpu_arch.guest_state, eip);

    vm_sync_guest_context(vcpu);
    vm_sync_guest_vmcs_state(vcpu);

    assert(!"no tcb");
//    seL4_TCB_Resume(vcpu->guest_tcb);
}

/* Got interrupt(s) from PIC, propagate to relevant vcpu lapic */
void vm_check_external_interrupt(vm_t *vm)
{
    /* TODO if all lapics are enabled, store which lapic
       (only one allowed) receives extints, and short circuit this */
    if (i8259_has_interrupt(vm) != -1) {
        vm_vcpu_t *vcpu = vm->vcpus[BOOT_VCPU];
        if (vm_apic_accept_pic_intr(vcpu)) {
            vm_vcpu_accept_interrupt(vcpu);
        }
    }
}

void vm_vcpu_accept_interrupt(vm_vcpu_t *vcpu)
{
    if (vm_apic_has_interrupt(vcpu) == -1) {
        return;
    }

    /* in an exit, can call the regular injection method */
    vm_have_pending_interrupt(vcpu);
}
