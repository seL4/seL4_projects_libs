<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `cross_vm_connection.h`

The crossvm connection module facilitates the creation of communication channels between VM's and other
components on a seL4-based system. The module exports registered cross vm connections to a Linux VM such that
processes can access them from userlevel. This being facilitated over a virtual PCI device.

### Brief content:

**Functions**:

> [`cross_vm_connections_init_common(vm, connection_base_addr, connections, num_connections, pci, alloc_irq)`](#function-cross_vm_connections_init_commonvm-connection_base_addr-connections-num_connections-pci-alloc_irq)

> [`consume_connection_event(vm, event_id, inject_irq)`](#function-consume_connection_eventvm-event_id-inject_irq)



**Structs**:

> [`crossvm_dataport_handle`](#struct-crossvm_dataport_handle)

> [`crossvm_handle`](#struct-crossvm_handle)


## Functions

The interface `cross_vm_connection.h` defines the following functions.

### Function `cross_vm_connections_init_common(vm, connection_base_addr, connections, num_connections, pci, alloc_irq)`

Install a set of cross vm connections into a guest VM (for either x86 or ARM VM platforms)
for the crossvm connectors
PCI device

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `connection_base_addr {uintptr_t}`: The base guest physical address that can be used to reserve memory
- `connections {crossvm_handle_t *}`: The set of crossvm connections to be initialised and installed in the guest
- `num_connection {int}`: The number of connections passed in through the 'connections' parameter
- `pci {vmm_pci_space_t *}`: A handle to the VM's host PCI device. The connections are advertised through the
- `alloc_irq {alloc_free_interrupt_fn}`: A function that is used to allocated an irq number for the crossvm connections

**Returns:**

- -1 on failure otherwise 0 for success

Back to [interface description](#module-cross_vm_connectionh).

### Function `consume_connection_event(vm, event_id, inject_irq)`

Handler to consume a cross vm connection event. This being called by the VMM when it recieves a notification from an
external process. The event is then relayed onto the VM.

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `event_id {seL4_Word}`: The id that corresponds to the occuring event
- `inject_irq {bool}`: Whether to inject an interrupt into the VM

**Returns:**

No return

Back to [interface description](#module-cross_vm_connectionh).


## Structs

The interface `cross_vm_connection.h` defines the following structs.

### Struct `crossvm_dataport_handle`

Datastructure representing a dataport of a crossvm connection

**Elements:**

- `frame_size_bits {size_t}`: Bit size of a single frame in the `frames` member
- `num_frames {int}`: Total number of frames in the `frames` member
- `frames {seL4_CPtr *}`: The set of frames backing the dataport

Back to [interface description](#module-cross_vm_connectionh).

### Struct `crossvm_handle`

Datastructure representing a single crossvm connection
This is matched on when invoking `consume_connection_event`

**Elements:**

- `dataport {crossvm_dataport_handle_t *}`: The dataport associated with the crossvm connection
- `emit_fn {emit_fn}`: The function pointer to the crossvm emit method
- `consume_id {seL4_Word}`: The identifier used for the crossvm connection when receiving incoming notifications

Back to [interface description](#module-cross_vm_connectionh).


Back to [top](#).

