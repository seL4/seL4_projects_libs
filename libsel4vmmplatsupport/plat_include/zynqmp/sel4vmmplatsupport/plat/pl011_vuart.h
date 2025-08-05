/*
 * Copyright 2024, DornerWorks
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once
#include <sel4vmmplatsupport/arch/pl011_vuart.h>

#define PL011_VUART_PLAT_SUPPORT
#define PL011_VUART_ADDR       0xA000A000
#define PL011_VUART_SIZE       0x1000
#define PL011_VUART_SPEED      115200
#define PL011_VUART_PORT       0x0
#define PL011_VUART_IRQ_TYPE   0x0
#define PL011_VUART_IRQ_NUM    0xA1
#define PL011_VUART_IRQ_FLAGS  0x4
