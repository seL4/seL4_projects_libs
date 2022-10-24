/*
 * Copyright 2019, DornerWorks
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4vm/guest_vm.h>
#include <sel4vmmplatsupport/device.h>
#include <sel4vmmplatsupport/plat/device_map.h>

#if defined(CONFIG_PLAT_ZYNQMP_ULTRA96) || defined(CONFIG_PLAT_ZYNQMP_ULTRA96V2)
#define dev_vconsole  dev_uart1
#define VCONSOLE_IRQ  UART1_IRQ
#else
#define dev_vconsole  dev_uart0
#define VCONSOLE_IRQ  UART0_IRQ
#endif

extern const struct device dev_uart0;
extern const struct device dev_uart1;

typedef void (*print_func_t)(int);

int vm_install_vconsole(vm_t *vm, print_func_t func);
void vuart_handle_irq(int c);
