<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_reboot.h`

The guest reboot interface provides a series of helpers for registering callbacks when rebooting the VMM.
This interface giving various VMM components and drivers the ability to reset necessary state on a reboot.

### Brief content:

**Functions**:

> [`vmm_init_reboot_hooks_list(rb_hooks_list)`](#function-vmm_init_reboot_hooks_listrb_hooks_list)

> [`vmm_register_reboot_callback(rb_hooks_list, hook, token)`](#function-vmm_register_reboot_callbackrb_hooks_list-hook-token)

> [`vmm_process_reboot_callbacks(vm, rb_hooks_list)`](#function-vmm_process_reboot_callbacksvm-rb_hooks_list)



**Structs**:

> [`reboot_hook`](#struct-reboot_hook)

> [`reboot_hooks_list`](#struct-reboot_hooks_list)


## Functions

The interface `guest_reboot.h` defines the following functions.

### Function `vmm_init_reboot_hooks_list(rb_hooks_list)`

Initialise state of a given reboot hooks list

**Parameters:**

- `rb_hooks_list {reboot_hooks_list_t *}`: Handle to reboot hooks list

**Returns:**

- 0 for success, otherwise -1 for error

Back to [interface description](#interface-guest_rebooth).

### Function `vmm_register_reboot_callback(rb_hooks_list, hook, token)`

Register a reboot callback within a given reboot hooks list

**Parameters:**

- `rb_hooks_list {reboot_hooks_list_t *}`: Handle to reboot hooks list
- `hook {rb_hook_fn}`: Reboot callback to be invoked when list is processed
- `token {void *}`: Cookie passed to reboot callback when invoked

**Returns:**

- 0 for success, otherwise -1 for error

Back to [interface description](#interface-guest_rebooth).

### Function `vmm_process_reboot_callbacks(vm, rb_hooks_list)`

Process the reboot hooks registered in a reboot hooks list

**Parameters:**

- `vm {vm_t *}`: Handle to vm - passed onto reboot callback
- `rb_hooks_list {reboot_hooks_list_t *}`: Handle to reboot hooks list

**Returns:**

- 0 for success, otherwise -1 for error

Back to [interface description](#interface-guest_rebooth).


## Structs

The interface `guest_reboot.h` defines the following structs.

### Struct `reboot_hook`

Datastructure representing a reboot hook, containing a callback function to invoke when processing the
hook

**Elements:**

- `fn {reboot_hook_fn}`: Function pointer to reboot callback
- `token {void *}`: Cookie passed to reboot callback when invoked

Back to [interface description](#interface-guest_rebooth).

### Struct `reboot_hooks_list`

Reboot hooks management datastructure. Contains a list of reboot hooks that a VMM registers

**Elements:**

- `rb_hooks {reboot_hook_t *}`: List of reboot hooks
- `nhooks {size_t}`: Number of reboot hooks in `rb_hooks` member

Back to [interface description](#interface-guest_rebooth).


Back to [top](#).

