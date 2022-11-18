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

typedef struct x86_irq_cookie {
    seL4_CPtr irq_cap;                  /* Cap to ack on after EOI signal from guest */
} x86_irq_cookie_t;

/* Struct to store information needed when injecting or ack'ing interrupts */
typedef struct irq_info {
    irq_ack_fn_t callback;
    x86_irq_cookie_t *cookie;
} irq_info_t;

