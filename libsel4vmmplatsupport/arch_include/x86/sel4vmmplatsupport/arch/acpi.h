/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

/***
 * @module acpi.h
 * The ACPI module provides support for generating ACPI table in a guest x86 VM. This being particularly leveraged
 * for booting a guest Linux VM.
 */

#define LOWER_BIOS_START (0xE0000)
#define LOWER_BIOS_SIZE (0xFFFF)

#define ACPI_START (LOWER_BIOS_START) // Start of ACPI tables; RSD PTR is right here
#define XSDT_START (ACPI_START + 0x1000)

#define MAX_ACPI_TABLES (16)

#include <sel4vm/guest_vm.h>

/***
 * @function make_guest_acpi_tables(vm)
 * Creates ACPI table for the guest VM
 * @param {vm_t *} vm       A handle to the guest VM instance
 * @return                  0 for success, -1 for error
 */
int make_guest_acpi_tables(vm_t *vm);
