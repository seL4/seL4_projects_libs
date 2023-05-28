/*
 * Copyright 2023, Technology Innovation Institute
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <libfdt.h>

/**
 * fdt_appendprop_uint - append a 32-bit or a 64-bit integer value to a property
 * @fdt: pointer to the device tree blob
 * @offset: offset of the node whose property to change
 * @name: name of the property to change
 * @val: integer value to append to the property (native endian)
 * @num_cells: typically fdt_address_cells() or fdt_size_cells() of parent node
 * @return -1 on error, 0 otherwise
 *
 * fdt_appendprop_uint() is a wrapper calling either fdt_appendprop_u32() or
 * fdt_appendprop_u64(), whether num_cells is 1 or 2, respectively. Hence
 * consult the documentation of those functions for return values. In case
 * num_cells is invalid, -FDT_ERR_BADNCELLS is returned.
 */
int fdt_appendprop_uint(void *fdt, int offset, const char *name, uint64_t val,
                        int num_cells);
