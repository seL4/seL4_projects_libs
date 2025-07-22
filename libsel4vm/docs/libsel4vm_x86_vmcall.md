<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `vmcall.h`

The x86 vmcall interface provides methods for registering and managing vmcall handlers. These being used
to process x86 guest hypercalls though the vmcall instruction.

### Brief content:

**Functions**:

> [`vm_reg_new_vmcall_handler(vm, func, token)`](#function-vm_reg_new_vmcall_handlervm-func-token)


## Functions

The interface `vmcall.h` defines the following functions.

### Function `vm_reg_new_vmcall_handler(vm, func, token)`

Register a new vmcall handler. The being hypercalls invoked by the
guest through the vmcall instruction.
This being matched with the value found in the vcpu EAX register on a vmcall exception

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `func {vmcall_handler}`: A handler function for the given vmcall being registered
- `token {int}`: A token to associate with a vmcall handler.

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-vmcallh).


Back to [top](#).

