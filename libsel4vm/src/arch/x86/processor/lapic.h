/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <sel4vm/guest_memory.h>

enum vm_lapic_state {
    LAPIC_STATE_NEW,
    LAPIC_STATE_WAITSIPI,
    LAPIC_STATE_RUN
};

#if 0
struct vm_timer {
    struct hrtimer timer;
    int64_t period;                 /* unit: ns */
    uint32_t timer_mode_mask;
    uint64_t tscdeadline;
    atomic_t pending;           /* accumulated triggered timers */
};
#endif

typedef struct vm_lapic {
    uint32_t apic_base; // BSP flag is ignored in this

    //struct vm_timer lapic_timer;
    uint32_t divide_count;

    bool irr_pending;
    /* Number of bits set in ISR. */
    int16_t isr_count;
    /* The highest vector set in ISR; if -1 - invalid, must scan ISR. */
    int highest_isr_cache;
    /**
     * APIC register page.  The layout matches the register layout seen by
     * the guest 1:1, because it is accessed by the vmx microcode. XXX ???
     * Note: Only one register, the TPR, is used by the microcode.
     */
    void *regs;
    unsigned int sipi_vector;

    enum vm_lapic_state state;
    int arb_prio;

    /* back pointer */
    vm_vcpu_t *vcpu;
} vm_lapic_t;

int vm_apic_enabled(vm_lapic_t *apic);

int vm_create_lapic(vm_vcpu_t *vcpu, int enabled);
void vm_free_lapic(vm_vcpu_t *vcpu);

int vm_apic_has_interrupt(vm_vcpu_t *vcpu);
int vm_apic_get_interrupt(vm_vcpu_t *vcpu);

void vm_apic_consume_extints(vm_vcpu_t *vcpu, int (*get)(void));

/* MSR functions */
void vm_lapic_set_base_msr(vm_vcpu_t *vcpu, uint32_t value);
uint32_t vm_lapic_get_base_msr(vm_vcpu_t *vcpu);

int vm_apic_local_deliver(vm_vcpu_t *vcpu, int lvt_type);
int vm_apic_accept_pic_intr(vm_vcpu_t *vcpu);

memory_fault_result_t apic_fault_callback(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                          size_t fault_length, void *cookie);

uint64_t vm_get_lapic_tscdeadline_msr(vm_vcpu_t *vcpu);
void vm_set_lapic_tscdeadline_msr(vm_vcpu_t *vcpu, uint64_t data);

