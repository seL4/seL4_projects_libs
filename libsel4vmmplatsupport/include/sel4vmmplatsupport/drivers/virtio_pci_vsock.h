/*
 * Copyright 2022, UNSW (ABN 57 195 873 179)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

typedef struct virtio_emul virtio_emul_t;

typedef void (*vsock_tx_forward_fn_t)(int cid, void *buffer, unsigned int len);

typedef struct virtio_vsock_callbacks {
    void (*rx_complete)(virtio_emul_t *emul, char *buf, unsigned int len);
} virtio_vsock_callbacks_t;

typedef struct virtio_vsock_passthrough {
    int guest_cid;
    void (*injectIRQ)(void *cookie);
    vsock_tx_forward_fn_t forward;
    void *vsock_data;
} virtio_vsock_passthrough_t;

struct virtio_vsock_driver {
    virtio_vsock_passthrough_t backend_fn;
    virtio_vsock_callbacks_t emul_cb;
};

typedef int (*vsock_driver_init)(struct virtio_vsock_driver *driver, ps_io_ops_t io_ops, void *config);
