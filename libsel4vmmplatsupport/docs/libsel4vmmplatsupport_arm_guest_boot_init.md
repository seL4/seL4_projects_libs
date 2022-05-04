<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_boot_init.h`

The libsel4vmmplatsupport arm guest boot init interface provides helpers to initialise the booting state of
a VM instance. This currently only targets booting a Linux guest OS.

### Brief content:

**Functions**:

> [`vcpu_set_bootargs(vcpu, pc, mach_type, atags)`](#function-vcpu_set_bootargsvcpu-pc-mach_type-atags)


## Functions

The interface `guest_boot_init.h` defines the following functions.

### Function `vcpu_set_bootargs(vcpu, pc, mach_type, atags)`

Set the boot args and pc for the VM. The Linux kernel documentation contains a detailed description about
the boot ABI for [AARCH32](https://www.kernel.org/doc/Documentation/arm/Booting) and
[AARCH64](https://www.kernel.org/doc/Documentation/arm64/booting.txt)

The register setup for the booting core on AARCH32 is

- `r0` = `0`
- `r1` = `MACH_TYPE`
- `r2` = ATAGS or FTD/DTB address

For AARCH64 it is:

- `x0` = FTD/DTB address
- `x1` = `0`
- `x2` = `0`
- `x3` = `0`

Passing a pointer to ATAGs is deprecated, modern kernels use a device tree that is passed as Flattened Device
Tree (FDT), sometimes also called Device Tree Blob (DTB). The parameter `MACH_TYPE` contains a machine
specific ID, various `MACH_TYPE_xxx` constants can be found in the
[Linux Source code at `arch/arm/tools/mach-types`](https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/tree/arch/arm/tools/mach-types)
FTDs usually contain all necessary information about the machine, so`-1` (`0xfff...fff`) can be passed.

**Parameters:**

- `vcpu {vm_vcpu_t *}`: A handle to the boot VCPU
- `pc {seL4_Word}`: The initial PC for the VM
- `mach_type {seL4_Word}`: Linux specific machine ID
- `atags {seL4_Word}`: Linux specific IPA of ATAGS or FTD/DTB address

**Returns:**

- 0 on success, otherwise -1 for failure

Back to [interface description](#module-guest_boot_inith).


Back to [top](#).

