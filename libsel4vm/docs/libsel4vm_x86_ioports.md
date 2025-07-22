<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `ioports.h`

The x86 ioports interface provides a useful abstraction for initialising, registering and handling ioport events
for a guest VM instance. IOPort faults are directed through this interface.

### Brief content:

**Functions**:

> [`vm_io_port_add_handler(vm, ioport_range, ioport_interface)`](#function-vm_io_port_add_handlervm-ioport_range-ioport_interface)

> [`vm_register_unhandled_ioport_callback(vm, ioport_callback, cookie)`](#function-vm_register_unhandled_ioport_callbackvm-ioport_callback-cookie)

> [`vm_enable_passthrough_ioport(vcpu, port_start, port_end)`](#function-vm_enable_passthrough_ioportvcpu-port_start-port_end)


## Functions

The interface `ioports.h` defines the following functions.

### Function `vm_io_port_add_handler(vm, ioport_range, ioport_interface)`

Add an io port range for emulation

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `ioport_range {vm_ioport_range_t}`: Range of ioport being emulated with the given handler
- `ioport_interface {vm_ioport_interface_t}`: Interface for ioport range, containing io_in and io_out handler functions

**Returns:**

- 0 for success, -1 for error

Back to [interface description](#interface-ioportsh).

### Function `vm_register_unhandled_ioport_callback(vm, ioport_callback, cookie)`

Register a callback for processing unhandled ioport faults (faults unknown to libsel4vm)

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `ioport_callback {unhandled_ioport_callback_fn}`: A user supplied callback to process unhandled ioport faults
- `cookie {void *}`: A cookie to supply to the callback

**Returns:**

- 0 for success, -1 for error

Back to [interface description](#interface-ioportsh).

### Function `vm_enable_passthrough_ioport(vcpu, port_start, port_end)`

Enable the passing-through of specific ioport ranges to the VM

**Parameters:**

- `vcpu {vm_vcpu_t *}`: A handle to the VCPU being given ioport passthrough access
- `port_start {uint16_t}`: Base address of ioport
- `port_end {uint16_t}`: End address of ioport

**Returns:**

- 0 for success, -1 for error

Back to [interface description](#interface-ioportsh).


Back to [top](#).

