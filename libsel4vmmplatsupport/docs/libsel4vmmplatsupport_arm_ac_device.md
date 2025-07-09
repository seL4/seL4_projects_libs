<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `ac_device.h`

The ARM access control device interface facilitates the creation of generic virtual devices in a VM instance with
access control permissions over the devices addressable memory. An access controlled device is often used to abstract
the memory of a specific platform hardware device e.g a clock device. This allows the user to present hardware devices
to a VM instance but limit their permissions with regards to modifying its register state.

### Brief content:

**Functions**:

> [`vm_install_generic_ac_device(vm, d, mask, size, action)`](#function-vm_install_generic_ac_devicevm-d-mask-size-action)


## Functions

The interface `ac_device.h` defines the following functions.

### Function `vm_install_generic_ac_device(vm, d, mask, size, action)`

Installs a generic access controlled device
are modifiable by the guest.
'1' represents bits that the guest can read and write
                                            '0' represents bits that can only be read by the guest
Underlying memory for the mask should remain accessible for
the life of this device. The mask may be updated at run time
on demand.
cases where the underlying device does not occupy a full
page. If an access lies outside of the range of the mask,
guest access.

**Parameters:**

- `vm {vm_t *}`: The VM to install the device into
- `d {const struct device *}`: A description of the device to install
- `mask {void *}`: An access mask. The mask provides a map of device bits that
- `size {size_t}`: The size of the mask. This is useful for conserving memory in
- `action {enum vacdev_action}`: Action to take when access is violated.

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-ac_deviceh).


Back to [top](#).

