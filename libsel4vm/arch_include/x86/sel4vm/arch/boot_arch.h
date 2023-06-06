/*
 * Copyright 2022, UNSW (ABN 57 195 873 179)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <sel4vmmplatsupport/arch/drivers/timer_emul.h>

/***
 * @function vm_assign_vcpu_timer(vcpu, timer_functions)
 * Assign a vcpu with timer functions to emulate a timer.
 * @param {vm_vcpu_t *} vcpu    A handle to the VCPU
 * @param {int} target          Logical target CPU ID
 */
void vm_assign_vcpu_timer(vm_vcpu_t *vcpu, struct timer_functions *timer_emul);