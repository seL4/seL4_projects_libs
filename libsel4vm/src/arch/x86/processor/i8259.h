/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_irq_controller.h>
#include <sel4vm/arch/guest_x86_irq_controller.h>

extern irq_info_t irq_info[NR_IRQS];

/* Init function */
int i8259_pre_init(vm_t *vm);

/* Functions to retrieve interrupt state */
int i8259_get_interrupt(vm_t *vm);
int i8259_has_interrupt(vm_t *vm);

/* Inject IRQ into guest PIC */
int i8259_inject_irq(vm_vcpu_t *vcpu, int irq);

/* Register IRQ with an ack function for EOIs */
int i8259_register_irq(vm_vcpu_t *vcpu, int irq, irq_ack_fn_t fn, void *cookie);
