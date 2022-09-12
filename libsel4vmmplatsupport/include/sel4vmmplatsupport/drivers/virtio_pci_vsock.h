/*
 * Copyright 2022, UNSW (ABN 57 195 873 179)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

typedef void (*vsock_inject_irq_fn_t)(void *cookie);
typedef void (*vsock_tx_forward_fn_t)(int cid, void *buffer, unsigned int len);

struct vsock_passthrough {
    int guest_cid;
    vsock_inject_irq_fn_t injectIRQ;
    vsock_tx_forward_fn_t forward;
    void *vsock_data;
};

typedef int (*vsock_driver_init)(struct vsock_passthrough *driver, ps_io_ops_t io_ops, void *config);
