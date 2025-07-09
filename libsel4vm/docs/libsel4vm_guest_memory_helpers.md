<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_memory_helpers.h`

The libsel4vm guest memory helpers interface provides simple utilities for using the guest memory interface.

### Brief content:

**Functions**:

> [`default_error_fault_callback(vm, vcpu, fault_addr, fault_length, cookie)`](#function-default_error_fault_callbackvm-vcpu-fault_addr-fault_length-cookie)


## Functions

The interface `guest_memory_helpers.h` defines the following functions.

### Function `default_error_fault_callback(vm, vcpu, fault_addr, fault_length, cookie)`

Default fault callback that throws a fault error.
Useful to avoid having to re-define a fault callback on regions that should be mapped with all rights.

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `vcpu {vm_vcpu_t *}`: A handle to the fault vcpu
- `fault {uintptr_t}`: addr    Faulting address
- `fault_length {size_t}`: Length of faulted access
- `cookie {void *}`: User cookie to pass onto callback

**Returns:**

- Always returns FAULT_ERROR

Back to [interface description](#interface-guest_memory_helpersh).


Back to [top](#).

