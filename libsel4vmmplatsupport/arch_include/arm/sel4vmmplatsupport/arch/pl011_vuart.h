/*
 * Copyright 2024, DornerWorks
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once
typedef void (*print_func_t)(int);
int fdt_generate_vuart_node(vm_t* vm, void* fdt, int gic_phandle);
int vm_install_pl011_vconsole(vm_t *vm, print_func_t func);
void pl011_vuart_handle_irq(int c);