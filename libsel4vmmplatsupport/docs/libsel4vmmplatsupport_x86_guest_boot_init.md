<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_boot_init.h`

The libsel4vmmplatsupport x86 guest boot init interface provides helpers to initialise the booting state of
a VM instance. This currently only targets booting a Linux guest OS.

### Brief content:

**Functions**:

> [`vmm_plat_init_guest_boot_structure(vm, cmdline, guest_kernel_image, guest_ramdisk_image, guest_boot_info_addr)`](#function-vmm_plat_init_guest_boot_structurevm-cmdline-guest_kernel_image-guest_ramdisk_image-guest_boot_info_addr)

> [`vmm_plat_init_guest_thread_state(vcpu, guest_entry_addr, guest_boot_info_addr)`](#function-vmm_plat_init_guest_thread_statevcpu-guest_entry_addr-guest_boot_info_addr)


## Functions

The interface `guest_boot_init.h` defines the following functions.

### Function `vmm_plat_init_guest_boot_structure(vm, cmdline, guest_kernel_image, guest_ramdisk_image, guest_boot_info_addr)`

Establish the necessary BIOS boot structures to initialise and boot a guest Linux OS. This includes the creation of a BIOS
boot info structure, an e820 map and ACPI tables.
(in guest physical address space)

**Parameters:**

- `vm {vm_t *}`: A handle to the guest VM
- `cmdline {const char *}`: The guest Linux boot commandline
- `guest_kernel_image {guest_kernel_image_t}`: Guest kernel image (preloaded into the VM's memory)
- `guest_ramdisk_image {guest_image_t}`: Guest ramdisk image (preloaded into the VM's memory)
- `guest_boot_info_addr {uintptr_t *}`: Resulting address of loaded generated guest boot info structure

**Returns:**

- 0 for success, -1 for error

Back to [interface description](#interface-guest_boot_inith).

### Function `vmm_plat_init_guest_thread_state(vcpu, guest_entry_addr, guest_boot_info_addr)`

Initialise the booting state of a guest VM, establishing the necessary thread state to launch a guest Linux OS

**Parameters:**

- `vcpu {vm_vcpu_t *}`: A handle to the boot vcpu
- `guest_entry_addr {uintptr_t}`: Address of VM entry point (often entry point defined in kernel elf image)
- `guest_boot_info_addr {uintptr_t}`: Address of loaded guest boot info structure

**Returns:**

- 0 for success, -1 for error

Back to [interface description](#interface-guest_boot_inith).


Back to [top](#).

