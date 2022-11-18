/*
 * Copyright 2022, UNSW (ABN 57 195 873 179)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <sel4/sel4.h>
#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_irq_controller.h>
#include <sel4vmmplatsupport/drivers/pci_msi.h>

#define I8259_NR_IRQS   16
#define LAPIC_NR_IRQS   14
#define NR_IRQS         (I8259_NR_IRQS + LAPIC_NR_IRQS)

/**
 * seL4 vectors
 * 
 * For x86 there are 256 idt entries. Vectors are used to index
 * into the idt and invoke interrupt handlers. This is how vectors
 * are reserved in seL4:
 * 
 * 0 - 31       : system traps and exceptions (hardcoded)
 * 32 - ...     : start of seL4 vectors
 *  32 - 47     : PIC interrupts/kernel only
 *  48          : start of user interrupts, when an irq is passed into an seL4
 *                system call, the _vector_ it is assigned to is (irq + 48).
 */
#define USER_IRQ_TO_CPU_VECTOR(x) ((x) + 48)


typedef struct x86_irq_msi_cookie {
    /* The original MSI data the guest programmed. We copy
     * this back on every irq injection. */
    pci_msi_data_t data;
} x86_irq_msi_cookie_t;

typedef struct x86_irq_cookie {
    bool is_msi;
    x86_irq_msi_cookie_t msi_cookie;    /* Cookie for MSI interrupts */
    seL4_CPtr irq_cap;                  /* Cap to ack on after EOI signal from guest */
} x86_irq_cookie_t;

/* Struct to store information needed when injecting or ack'ing interrupts */
typedef struct irq_info {
    irq_ack_fn_t callback;
    x86_irq_cookie_t *cookie;
} irq_info_t;

/***
 * @function vm_timer_inject_irq(vcpu)
 * Inject an a timer IRQ. This is for when the IRQ controller handles the nitty-gritty
 * IRQ assignments, and we have no way of telling where the timer IRQ is supposed to go.
 * @param {vm_vcpu_t *} vcpu    Handle to the VCPU
 * @return                      0 on success, otherwise -1 for error
 */
int vm_inject_timer_irq(vm_vcpu_t *vcpu);

/***
 * @function vm_irq_set_msi_data(irq, msi_data)
 * Save the msi data for an irq so we can patch values in later when the
 * device invokes an msi.
 * 
 * @param {int} irq                     the msi's irq
 * @param {pci_msi_data_t *} msi_data   the msi data we want to patch in later
 */
void vm_irq_set_msi_data(int irq, pci_msi_data_t *msi_data);
