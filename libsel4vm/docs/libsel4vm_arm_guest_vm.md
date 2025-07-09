<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_vm_arch.h`

The guest arm vm interface is central to using libsel4vm on an ARM platform, providing definitions of the arm guest vm
datastructures and primitives to configure the VM instance.

### Brief content:

**Functions**:

> [`vm_register_unhandled_vcpu_fault_callback(vcpu, vcpu_fault_callback, cookie)`](#function-vm_register_unhandled_vcpu_fault_callbackvcpu-vcpu_fault_callback-cookie)



**Structs**:

> [`vm_vcpu_arch`](#struct-vm_vcpu_arch)


## Functions

The interface `guest_vm_arch.h` defines the following functions.

### Function `vm_register_unhandled_vcpu_fault_callback(vcpu, vcpu_fault_callback, cookie)`

Register a callback for processing unhandled vcpu faults

**Parameters:**

- `vcpu {vm_vcpu_t *}`: A handle to the VCPU
- `A {unhandled_vcpu_fault_callback_fn}`: user supplied callback to process unhandled vcpu faults
- `A {void *}`: cookie to supply to the vcpu fault handler

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-guest_vm_archh).


## Structs

The interface `guest_vm_arch.h` defines the following structs.

### Struct `vm_vcpu_arch`

Structure representing ARM specific vcpu properties

**Elements:**

- `fault {fault_t *}`: Current VCPU fault
- `unhandled_vcpu_callback {unhandled_vcpu_fault_callback_fn}`: A callback for processing unhandled vcpu faults
- `unhandled_vcpu_callback_cookie {void *}`: A cookie to supply to the vcpu fault handler

Back to [interface description](#interface-guest_vm_archh).


Back to [top](#).

