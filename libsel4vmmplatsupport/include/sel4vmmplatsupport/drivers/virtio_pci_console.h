/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

typedef void (*console_handle_irq_fn_t)(void *cookie);
typedef void (*console_putchar_fn_t)(int port, char c);

struct console_passthrough {
    console_handle_irq_fn_t handleIRQ;
    console_putchar_fn_t    putchar;
    void                   *console_data;
};

typedef int (*console_driver_init)(struct console_passthrough *driver, ps_io_ops_t io_ops, void *config);
