<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `acpi.h`

The ACPI module provides support for generating ACPI table in a guest x86 VM. This being particularly leveraged
for booting a guest Linux VM.

### Brief content:

**Functions**:

> [`make_guest_acpi_tables(vm)`](#function-make_guest_acpi_tablesvm)


## Functions

The interface `acpi.h` defines the following functions.

### Function `make_guest_acpi_tables(vm)`

Creates ACPI table for the guest VM

**Parameters:**

- `vm {vm_t *}`: A handle to the guest VM instance

**Returns:**

- 0 for success, -1 for error

Back to [interface description](#interface-acpih).


Back to [top](#).

