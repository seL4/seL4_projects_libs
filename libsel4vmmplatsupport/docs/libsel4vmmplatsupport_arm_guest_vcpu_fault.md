<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_vcpu_fault.h`

The arm guest vcpu fault interface provides a module for registering
and processing vcpu faults. The module can be used such that it directly hooks
into an ARM VMMs unhandled vcpu callback interface or wrapped over by another driver. This
module particularly provides useful handlers to vcpu faults relating to SMC and MSR/MRS instructions.

### Brief content:

**Functions**:

> [`vmm_handle_arm_vcpu_exception(vcpu, hsr, cookie)`](#function-vmm_handle_arm_vcpu_exceptionvcpu-hsr-cookie)

> [`register_arm_vcpu_exception_handler(ec_class, exception_handler)`](#function-register_arm_vcpu_exception_handlerec_class-exception_handler)


## Functions

The interface `guest_vcpu_fault.h` defines the following functions.

### Function `vmm_handle_arm_vcpu_exception(vcpu, hsr, cookie)`

Handle a vcpu exception given the HSR value - Syndrome information

**Parameters:**

- `vcpu {vm_vcpu_t *}`: A handle to the faulting VCPU
- `hsr {uint32_t }`: Syndrome information value describing the exception/fault
- `cookie {void *}`: User supplied cookie to pass onto exception

**Returns:**

- -1 on error, otherwise 0 for success

Back to [interface description](#interface-guest_vcpu_faulth).

### Function `register_arm_vcpu_exception_handler(ec_class, exception_handler)`

Register a handler to a vcpu exception class

**Parameters:**

- `ec_class {uint32_t}`: The exception class the handler will be called on
- `exception_handler {vcpu_exception_handler_fn}`: Function pointer to the exception handler

**Returns:**

No return

Back to [interface description](#interface-guest_vcpu_faulth).


Back to [top](#).

