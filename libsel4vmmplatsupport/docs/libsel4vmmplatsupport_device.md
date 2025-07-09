<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `device.h`

The device.h interface provides a series of datastructures and helpers to manage VMM devices.

### Brief content:

**Functions**:

> [`device_list_init(list)`](#function-device_list_initlist)

> [`add_device(dev_list, d)`](#function-add_devicedev_list-d)

> [`find_device_by_pa(dev_list, addr)`](#function-find_device_by_padev_list-addr)



**Structs**:

> [`device`](#struct-device)

> [`device_list`](#struct-device_list)


## Functions

The interface `device.h` defines the following functions.

### Function `device_list_init(list)`

Initialise an empty device list

**Parameters:**

- `list {device_list_t *}`: device list to initialise

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-deviceh).

### Function `add_device(dev_list, d)`

Add a generic device to a given device list without performing any initialisation of the device

**Parameters:**

- `dev_list {device_list_t *}`: A handle to the device list that the device should be installed into
- `device {const struct device *}`: A description of the device

**Returns:**

- 0 on success, otherwise -1 for error

Back to [interface description](#interface-deviceh).

### Function `find_device_by_pa(dev_list, addr)`

Find a device by a given addr within a device list

**Parameters:**

- `dev_list {device_list_t *}`: Device list to search within
- `addr {uintptr_t}`: Add to search with

**Returns:**

- Pointer to device if found, otherwise NULL if not found

Back to [interface description](#interface-deviceh).


## Structs

The interface `device.h` defines the following structs.

### Struct `device`

Device Management Object


**Elements:**

- `name {const char *}`: A string representation of the device. Useful for debugging
- `pstart {seL4_Word}`: The physical address of the device
- `size {seL4_Word}`: Device mapping size
- `handle_device_fault {int *(vm_t, vm_vcpu_t, dev, addr, len)}`: Fault handler
- `priv {void *}`: Device emulation private data

Back to [interface description](#interface-deviceh).

### Struct `device_list`

Management for a list of devices

**Elements:**

- `devices {struct device *}`: List of registered devices
- `num_devices {int}`: Total number of registered devices

Back to [interface description](#interface-deviceh).


Back to [top](#).

