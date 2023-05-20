<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_memory.h`

The libsel4vm memory interface provides useful abstractions to manage your guest VM's address space.
This interface can be leveraged for uses such as mapping device memory into your VM instance
or for creating emulated device regions binded with custom handlers/callbacks.
The main mechanisms this interface provides involves reserving memory regions and using those reservations
to either map memory into your guest VM's address space or emulate subsequent accesses. Reservations
are created through either using `vm_reserve_memory_at` or `vm_reserve_anon_memory`.
The user can then further back the reservation with seL4 frames by performing `vm_map_reservation`.

### Brief content:

**Functions**:

> [`vm_reserve_memory_at(vm, addr, size, fault_callback, cookie)`](#function-vm_reserve_memory_atvm-addr-size-fault_callback-cookie)

> [`vm_reserve_anon_memory(vm, size, fault_callback, cookie, addr)`](#function-vm_reserve_anon_memoryvm-size-fault_callback-cookie-addr)

> [`vm_free_reserved_memory(vm, reservation)`](#function-vm_free_reserved_memoryvm-reservation)

> [`vm_map_reservation(vm, reservation, map_iterator, cookie)`](#function-vm_map_reservationvm-reservation-map_iterator-cookie)

> [`vm_map_reservation_frames(vm, reservation, frames, num_frames, frame_size_bits)`](#function-vm_map_reservationvm-reservation-frames-num_frames-frame_size_bits)

> [`vm_get_reservation_memory_region(reservation, addr, size)`](#function-vm_get_reservation_memory_regionreservation-addr-size)

> [`vm_memory_init(vm)`](#function-vm_memory_initvm)



**Structs**:

> [`vm_frame_t`](#struct-vm_frame_t)


## Functions

The interface `guest_memory.h` defines the following functions.

### Function `vm_reserve_memory_at(vm, addr, size, fault_callback, cookie)`

Reserve a region of the VM's memory at a given base address

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `addr {uintptr}`: Base address of the memory region being reserved
- `size {size_t}`: Size of the memory region being reserved
- `fault_callback {memory_fault_callback_fn}`: Callback function that will be invoked if memory region is faulted on
- `cookie {void *}`: User cookie to pass onto to callback

**Returns:**

- NULL on failure otherwise a pointer to a reservation object representing the reserved region

Back to [interface description](#module-guest_memoryh).

### Function `vm_reserve_anon_memory(vm, size, align, fault_callback, cookie, addr)`

Reserve an anonymous region of the VM's memory. This uses memory previously made anonymous
through the `vm_memory_make_anon` function.

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `size {size_t}`: Size of the anoymous emory region being reserved
- `align {size_t}`: Requested alignment of the memory region
- `fault_callback {memory_fault_callback_fn}`: Callback function that will be invoked if memory region is faulted on
- `cookie {void *}`: User cookie to pass onto to callback
- `addr {uintptr_t *}`: Pointer that will be set with the base address of the reserved anonymous region

**Returns:**

- NULL on failure otherwise a pointer to a reservation object representing the reserved region

Back to [interface description](#module-guest_memoryh).

### Function `vm_free_reserved_memory(vm, reservation)`

Free memory reservation from the VM

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `reservation {vm_memory_reservation_t *}`: Pointer to the reservation being free'd

**Returns:**

- -1 on failure otherwise 0 for success

Back to [interface description](#module-guest_memoryh).

### Function `vm_map_reservation(vm, reservation, map_iterator, cookie)`

Map a reservation into the VM's virtual address space

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `reservation {vm_memory_reservation_t *}`: Pointer to reservation object being mapped
- `map_iterator {memory_map_iterator_fn}`: Iterator function that returns a cap to the memory region being mapped
- `cookie {void *}`: Cookie to pass onto map_iterator function

**Returns:**

- -1 on failure otherwise 0 for success

Back to [interface description](#module-guest_memoryh).

### Function `vm_map_reservation_frames(vm, reservation, frames, num_frames, frame_size_bits)`

Map a reservation into the VM's virtual address space from given frames

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `reservation {vm_memory_reservation_t *}`: Pointer to reservation object being mapped
- `frames {seL4_CPtr *}`: Array of frame caps
- `num_frames {size_t}`: Number of frame caps
- `frame_size_bits {size_t}`: Bit size of a single frame

**Returns:**

- -1 on failure otherwise 0 for success

Back to [interface description](#module-guest_memoryh).

### Function `vm_get_reservation_memory_region(reservation, addr, size)`

Get the memory region information (address & size) from a given reservation

**Parameters:**

- `reservation {vm_memory_reservation_t *}`: Pointer to reservation object
- `addr {uintptr_t *}`: Pointer that will be set with the address of reservation
- `size {size_t *}`: Pointer that will be set with the size of reservation

**Returns:**

No return

Back to [interface description](#module-guest_memoryh).

### Function `vm_memory_init(vm)`

Initialise a VM's memory management interface

**Parameters:**

- `vm {vm_t *}`: A handle to the VM

**Returns:**

- -1 on failure otherwise 0 for success

Back to [interface description](#module-guest_memoryh).


## Structs

The interface `guest_memory.h` defines the following structs.

### Struct `vm_frame_t`

Structure representing a mappable memory frame

**Elements:**

- `cptr {seL4_CPtr}`: Capability to frame
- `rights {seL4_CapRights_t}`: Mapping rights of frame
- `vaddr {uintptr_t}`: Virtual address of which to map the frame into
- `size_bits {size_t}`: Size of frame in bits

Back to [interface description](#module-guest_memoryh).


Back to [top](#).

