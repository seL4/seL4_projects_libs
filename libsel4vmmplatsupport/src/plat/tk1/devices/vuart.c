/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <stdlib.h>
#include <string.h>

#include <sel4vmmplatsupport/plat/devices.h>
#include <platsupport/plat/serial.h>

const struct device dev_uartd = {
    .name = "uartd",
    .pstart = UARTD_PADDR,
    .size = PAGE_SIZE_4K,
    .handle_device_fault = NULL,
    .priv = NULL
};
