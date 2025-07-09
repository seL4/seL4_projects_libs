<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `guest_memory_util.h`

The guest memory util interface provides various utilities and helpers for using the libsel4vm guest memory
interface. The methods in the interface cover common usage patterns when managing memory for a VM instance.

### Brief content:

**Functions**:

> [`create_allocated_reservation_frame(vm, addr, rights, alloc_fault_callback, alloc_fault_cookie)`](#function-create_allocated_reservation_framevm-addr-rights-alloc_fault_callback-alloc_fault_cookie)

> [`create_device_reservation_frame(vm, addr, rights, fault_callback, fault_cookie)`](#function-create_device_reservation_framevm-addr-rights-fault_callback-fault_cookie)

> [`map_ut_alloc_reservation_with_base_paddr(vm, paddr, reservation)`](#function-map_ut_alloc_reservation_with_base_paddrvm-paddr-reservation)

> [`map_ut_alloc_reservation(vm, reservation)`](#function-map_ut_alloc_reservationvm-reservation)

> [`map_frame_alloc_reservation(vm, reservation)`](#function-map_frame_alloc_reservationvm-reservation)

> [`map_maybe_device_reservation(vm, reservation)`](#function-map_maybe_device_reservationvm-reservation)


## Functions

The interface `guest_memory_util.h` defines the following functions.

### Function `create_allocated_reservation_frame(vm, addr, rights, alloc_fault_callback, alloc_fault_cookie)`

Create and map a reservation for a vka allocated frame. The allocated frame is mapped in both the vm and vmm vspace

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `addr {uintptr_t}`: Address of emulated frame
- `rights {seL4_CapRights_t}`: Rights for mapping the allocated frame into the vm's vspace
- `alloc_fault_callback {memory_fault_callback_fn}`: Fault callback for allocated frame
- `alloc_fault_cookie {void *}`: Cookie for fault callback

**Returns:**

- Address of allocated frame in vmm vspace

Back to [interface description](#interface-guest_memory_utilh).

### Function `create_device_reservation_frame(vm, addr, rights, fault_callback, fault_cookie)`

Create and map a reservation for a device frame. The device frame is mapped in both the vm and vmm vspace

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `addr {uintptr_t}`: Address of emulated frame
- `rights {seL4_CapRights_t}`: Rights for mapping the device frame into the vm's vspace
- `fault_callback {memory_fault_callback_fn}`: Fault callback for the frame
- `fault_cookie {void *}`: Cookie for fault callback

**Returns:**

- Address of device frame in vmm vspace

Back to [interface description](#interface-guest_memory_utilh).

### Function `map_ut_alloc_reservation_with_base_paddr(vm, paddr, reservation)`

Map a guest reservation backed with untyped frames allocated from a base paddr

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `paddr {uintptr_t}`: Base paddr to allocate from
- `reservation {vm_memory_reservation_t *}`: Pointer to reservation object being mapped

**Returns:**

- -1 on failure otherwise 0 for success

Back to [interface description](#interface-guest_memory_utilh).

### Function `map_ut_alloc_reservation(vm, reservation)`

Map a guest reservation backed with untyped frames

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `reservation {vm_memory_reservation_t *}`: Pointer to reservation object being mapped

**Returns:**

- -1 on failure otherwise 0 for success

Back to [interface description](#interface-guest_memory_utilh).

### Function `map_frame_alloc_reservation(vm, reservation)`

Map a guest reservation backed with free vka frames

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `reservation {vm_memory_reservation_t *}`: Pointer to reservation object being mapped

**Returns:**

- -1 on failure otherwise 0 for success

Back to [interface description](#interface-guest_memory_utilh).

### Function `map_maybe_device_reservation(vm, reservation)`

Map a guest reservation backed with device frames

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `reservation {vm_memory_reservation_t *}`: Pointer to reservation object being mapped

**Returns:**

- -1 on failure otherwise 0 for success

Back to [interface description](#interface-guest_memory_utilh).


Back to [top](#).

