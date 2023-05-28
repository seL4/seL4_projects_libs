/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 * Copyright 2023, Technology Innovation Institute
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vmmplatsupport/fdt_utils.h>

#include <libfdt.h>
#include <utils/util.h>

int fdt_appendprop_uint(void *fdt, int offset, const char *name, uint64_t val,
                        int num_cells)
{
    int err;
    if (num_cells == 2) {
        err = fdt_appendprop_u64(fdt, offset, name, val);
    } else if (num_cells == 1) {
        err = fdt_appendprop_u32(fdt, offset, name, val);
    } else {
        ZF_LOGE("invalid number of cells (%d)", num_cells);
        err = -FDT_ERR_BADNCELLS;
    }

    return err;
}
