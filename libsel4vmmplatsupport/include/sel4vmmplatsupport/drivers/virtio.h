/*
 * Copyright 2022, UNSW (ABN 57 195 873 179)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/* The following constants are found in the virtio spec: http://docs.oasis-open.org/virtio/virtio/v1.0/cs04/virtio-v1.0-cs04.html#x1-650001
 * Necessary for identifying a virtio pci device */

/* Virtio PCI Vendor ID. A PCI device with the following vendor ID is a
 * virtio device */
#define VIRTIO_PCI_VENDOR_ID            0x1af4
#define VIRTIO_PCI_SUBSYSTEM_VENDOR_ID  0x1af4

/* Virtio device IDs  */
#define VIRTIO_NET_PCI_DEVICE_ID        0x1000
#define VIRTIO_BLOCK_PCI_DEVICE_ID      0x1001
#define VIRTIO_CONSOLE_PCI_DEVICE_ID    0x1003
#define VIRTIO_VSOCK_PCI_DEVICE_ID      0x1012

/* Virtio subsystem device ids */
#define VIRTIO_ID_NET                   1
#define VIRTIO_ID_BLOCK                 2
#define VIRTIO_ID_CONSOLE               3
#define VIRTIO_ID_VSOCK                 19

/* Virtio PCI device classes, source: https://pci-ids.ucw.cz/read/PD/ */
/* Device class 02 (network controller), subclass 00 (ethernet controller) */
#define VIRTIO_PCI_CLASS_NET            0x020000
/* Device class 01 (mass storage controller), subclass 00 (SCSI storage controller) */
#define VIRTIO_PCI_CLASS_BLOCK          0x010000
/* Device class 07 (communication controller), subclass 80 (communication controller) */
#define VIRTIO_PCI_CLASS_CONSOLE        0x078000
/* Device class 07 (communication controller), subclass 80 (communication controller) */
#define VIRTIO_PCI_CLASS_VSOCK          0x078000
