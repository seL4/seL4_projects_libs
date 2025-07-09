<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_image.h`

The guest image interface provides general utilites to load guest vm images (e.g. kernel, initrd, modules).
This interface probably stops being relevant/useful after we start running a VM instance.

### Brief content:

**Functions**:

> [`vm_load_guest_kernel(vm, kernel_name, load_address, alignment, guest_kernel_image)`](#function-vm_load_guest_kernelvm-kernel_name-load_address-alignment-guest_kernel_image)

> [`vm_load_guest_module(vm, module_name, load_address, alignment, guest_image)`](#function-vm_load_guest_modulevm-module_name-load_address-alignment-guest_image)



**Structs**:

> [`guest_image`](#struct-guest_image)

> [`guest_kernel_image`](#struct-guest_kernel_image)


## Functions

The interface `guest_image.h` defines the following functions.

### Function `vm_load_guest_kernel(vm, kernel_name, load_address, alignment, guest_kernel_image)`

Load guest kernel image

**Parameters:**

- `vm {vm_t *}`: Handle to the VM
- `kernel_name {const char *}`: Name of the kernel image
- `load_address {uintptr_t}`: Address to load guest kernel image at
- `alignment {size_t}`: Alignment for loading kernel image
- `guest_kernel_image {guest_kernel_image_t *}`: Handle to information regarding the resulted loading of the guest kernel image

**Returns:**

- 0 on success, otherwise -1 on error

Back to [interface description](#interface-guest_imageh).

### Function `vm_load_guest_module(vm, module_name, load_address, alignment, guest_image)`

Load guest kernel module e.g. initrd

**Parameters:**

- `vm {vm_t *}`: Handle to the VM
- `module_name {const char *}`: Name of the module image
- `load_address {uintptr_t}`: Address to load guest kernel image at
- `alignment {size_t}`: Alignment for loading module image
- `guest_image {guest_image_t *}`: Handle to information regarding the resulted loading of the guest module image

**Returns:**

- 0 on success, otherwise -1 on error

Back to [interface description](#interface-guest_imageh).


## Structs

The interface `guest_image.h` defines the following structs.

### Struct `guest_image`

General datastructure for managing a guest image

**Elements:**

- `load_paddr {uintptr_t}`: Base address (in guest physical) where the image was loaded
- `alignment {size_t}`: Alignment we used when loading the image
- `size {size_t}`: Size of guest image

Back to [interface description](#interface-guest_imageh).

### Struct `guest_kernel_image`

Stores information about the guest kernel image we are loading.

**Elements:**

- `kernel_image {guest_image_t}`: Datastructure referring to guest kernel image
- `kernel_image_arch {guest_kernel_image_arch_t}`: Architecture specific information for loaded guest image

Back to [interface description](#interface-guest_imageh).


Back to [top](#).

