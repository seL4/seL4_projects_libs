<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `device_utils.h`

The device utils interface provides various helpers to establish different types devices for a given VM
instance.

### Brief content:

**Functions**:

> [`vm_install_passthrough_device(vm, device)`](#function-vm_install_passthrough_devicevm-device)

> [`vm_install_ram_only_device(vm, device)`](#function-vm_install_ram_only_devicevm-device)

> [`vm_install_listening_device(vm, device)`](#function-vm_install_listening_devicevm-device)


## Functions

The interface `device_utils.h` defines the following functions.

### Function `vm_install_passthrough_device(vm, device)`

Install a passthrough device into a VM

**Parameters:**

- `vm {vm_t *}`: A handle to the VM that the device should be install to
- `device {const struct device *}`: A description of the device

**Returns:**

- 0 on success, -1 for error

Back to [interface description](#interface-device_utilsh).

### Function `vm_install_ram_only_device(vm, device)`

Install a device backed by ram into a VM

**Parameters:**

- `vm {vm_t *}`: A handle to the VM that the device should be install to
- `device {const struct device *}`: A description of the device

**Returns:**

- 0 on success, -1 for error

Back to [interface description](#interface-device_utilsh).

### Function `vm_install_listening_device(vm, device)`

Install a passthrough device into a VM, but trap and print all access

**Parameters:**

- `vm {vm_t *}`: A handle to the VM that the device should be install to
- `device {const struct device *}`: A description of the device

**Returns:**

- 0 on success, -1 for error

Back to [interface description](#interface-device_utilsh).


Back to [top](#).

