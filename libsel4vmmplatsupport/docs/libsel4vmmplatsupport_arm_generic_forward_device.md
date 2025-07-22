<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `generic_forward_device.h`

This interface facilitates the creation of a virtual device used for
dispatching faults to external handlers. For example when using CAmkES,
a device frame for a real device can be given to a different CAmkES component
and this virtual device will forward read and write faults over a CAmkES
interface so the component can perform or emulate the actions.

### Brief content:

**Functions**:

> [`vm_install_generic_forward_device(vm, d, cfg)`](#function-vm_install_generic_forward_devicevm-d-cfg)



**Structs**:

> [`generic_forward_cfg`](#struct-generic_forward_cfg)


## Functions

The interface `generic_forward_device.h` defines the following functions.

### Function `vm_install_generic_forward_device(vm, d, cfg)`

Install the virtual forwarding device into a VM instance

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `Virtual {const struct device *}`: device being forwarded
- `Interface {struct generic_forward_cfg}`: for forwarding the devices read and write faults

**Returns:**

- -1 for error, otherwise 0 for success

Back to [interface description](#interface-generic_forward_deviceh).


## Structs

The interface `generic_forward_device.h` defines the following structs.

### Struct `generic_forward_cfg`

Interface for forwarding read and write faults

**Elements:**

- `write_fn {forward_write_fn}`: A callback for forwarding write faults
- `read_fn {forward_read_fn}`: A callback for forwarding read faults

Back to [interface description](#interface-generic_forward_deviceh).


Back to [top](#).

