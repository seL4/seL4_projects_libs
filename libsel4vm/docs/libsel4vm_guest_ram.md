<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_ram.h`

The libsel4vm RAM interface provides us with a set of methods to manage a guest VM's RAM. This involves functions
to register, allocate and copy to and from RAM regions.

### Brief content:

**Functions**:

> [`vm_guest_ram_read_callback(vm, guest_addr, vaddr, size, offset, buf)`](#function-vm_guest_ram_read_callbackvm-guest_addr-vaddr-size-offset-buf)

> [`vm_guest_ram_write_callback(vm, guest_addr, vaddr, size, offset, buf)`](#function-vm_guest_ram_write_callbackvm-guest_addr-vaddr-size-offset-buf)

> [`vm_ram_touch(vm, addr, size, touch_callback, cookie)`](#function-vm_ram_touchvm-addr-size-touch_callback-cookie)

> [`vm_ram_find_largest_free_region(vm, addr, size)`](#function-vm_ram_find_largest_free_regionvm-addr-size)

> [`vm_ram_register(vm, bytes)`](#function-vm_ram_registervm-bytes)

> [`vm_ram_register_at(vm, start, bytes, untyped)`](#function-vm_ram_register_atvm-start-bytes-untyped)

> [`vm_ram_mark_allocated(vm, start, bytes)`](#function-vm_ram_mark_allocatedvm-start-bytes)

> [`vm_ram_allocate(vm, bytes)`](#function-vm_ram_allocatevm-bytes)

> [`vm_ram_free(vm, start, bytes)`](#function-vm_ram_freevm-start-bytes)


## Functions

The interface `guest_ram.h` defines the following functions.

### Function `vm_guest_ram_read_callback(vm, guest_addr, vaddr, size, offset, buf)`

Common guest ram touch callback for reading from a guest address into a user supplied buffer

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `guest_addr {uintptr_t}`: Guest physical address to read from
- `vmm_vaddr {void *}`: Virtual address in hosts (vmm) vspace corresponding with the 'guest_addr'
- `size {size_t}`: Size of region being currently accessed
- `offset {size_t}`: Current offset from the base guest physical address supplied to 'vm_ram_touch'
- `cookie {void *}`: User supplied buffer to store read data into

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-guest_ramh).

### Function `vm_guest_ram_write_callback(vm, guest_addr, vaddr, size, offset, buf)`

Common guest ram touch callback for writing a user supplied buffer into a guest address

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `guest_addr {uintptr_t}`: Guest physical address to write to
- `vmm_vaddr {void *}`: Virtual address in hosts (vmm) vspace corresponding with the 'guest_addr'
- `size {size_t}`: Size of region being currently accessed
- `offset {size_t}`: Current offset from the base guest physical address supplied to 'vm_ram_touch'
- `cookie {void *}`: User supplied buffer to write data from

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-guest_ramh).

### Function `vm_ram_touch(vm, addr, size, touch_callback, cookie)`

Touch a series of pages in the guest vm and invoke a callback for each page accessed

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `addr {uintptr_t}`: Address to access in the guest vm
- `size {size_t}`: Size of memory region to access
- `callback {ram_touch_callback_fn}`: Callback to invoke on each page access
- `cookie {void *}`: User data to pass onto callback

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-guest_ramh).

### Function `vm_ram_find_largest_free_region(vm, addr, size)`

Find the largest free ram region

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `addr {uintptr_t *}`: Pointer to be set with largest region address
- `size {size_t *}`: Pointer to be set with largest region size

**Returns:**

- -1 on failure, otherwise 0 for success

Back to [interface description](#interface-guest_ramh).

### Function `vm_ram_register(vm, bytes)`

Reserve a region of memory for RAM in the guest VM

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `bytes {size_t}`: Size of RAM region to allocate

**Returns:**

- Starting address of registered ram region

Back to [interface description](#interface-guest_ramh).

### Function `vm_ram_register_at(vm, start, bytes, untyped)`

Reserve a region of memory for RAM in the guest VM at a starting guest physical address

**Parameters:**

- `vm {vm_t *}`: A handle to the VM that ram needs to be allocated for
- `start {uintptr_t}`: Starting guest physical address of the ram region being allocated
- `size {size_t}`: The size of the RAM region to be allocated
- `untyped {bool}`: Allocate RAM frames such that it uses untyped memory

**Returns:**

- 0 on success, -1 on error

Back to [interface description](#interface-guest_ramh).

### Function `vm_ram_mark_allocated(vm, start, bytes)`

Mark a registered region of RAM as allocated

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `start {uintptr_t}`: Starting address of guest ram region
- `bytes {size_t}`: Size of RAM region

**Returns:**

No return

Back to [interface description](#interface-guest_ramh).

### Function `vm_ram_allocate(vm, bytes)`

Allocate a region of registered ram

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `bytes {size_t}`: Size of allocation

**Returns:**

- Starting address of allocated ram region

Back to [interface description](#interface-guest_ramh).

### Function `vm_ram_free(vm, start, bytes)`

Free a RAM a previously allocated RAM region

**Parameters:**

- `vm {vm_t *}`: A handle to the VM that ram needs to be free'd for
- `start {uintptr_t}`: Starting guest physical address of the ram region being free'd
- `size {size_t}`: The size of the RAM region to be free'd

**Returns:**

No return

Back to [interface description](#interface-guest_ramh).


Back to [top](#).

