/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

typedef struct virtio_emul virtio_emul_t;

typedef void (*console_putchar_fn_t)(int port, char c);

/***
 * @struct virtio_console_callbacks
 * Callback functions provided by the emul layer of virtio console
 *
 * @param backend_putchar putchar
 */
typedef struct virtio_console_callbacks {
    void (*emul_putchar)(int port, virtio_emul_t *con, char *buffer, uint32_t head, uint32_t tail);
} virtio_console_callbacks_t;

/***
 * @struct virtio_console_passthrough
 * Virtion console backend layer interface
 *
 * @param handleIRQ handle IRQ
 * @param backend_putchar putchar
 * @param console_data data specified by the backend
 */
typedef struct virtio_console_passthrough {
    void (*handleIRQ)(void *cookie);
    console_putchar_fn_t backend_putchar;
    void *console_data;
} virtio_console_passthrough_t;

/***
 * @struct virtio_console_driver
 * Structure to hold the interface for a virtio console driver
 *
 * @param backend_fn backend layer interface
 * @param emul_cb emul layer interface
 */
struct virtio_console_driver {
    virtio_console_passthrough_t backend_fn;
    virtio_console_callbacks_t emul_cb;
};

typedef int (*console_driver_init)(struct virtio_console_driver *driver, ps_io_ops_t io_ops, void *config);
