<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `ioports.h`

The ioports interface provides a useful abstraction for initialising, registering and handling ioport events
for a guest VM instance. This is independent from the x86 libsel4vm ioports interface. This interface is intended
to be more generic and architecture independent, useful for usecases that require ioports without architecture
support (virtio PCI).

### Brief content:

**Functions**:

> [`vmm_io_port_init(io_list, ioport_alloc_addr)`](#function-vmm_io_port_initio_list-ioport_alloc_addr)

> [`vmm_io_port_add_handler(io_list, ioport_range, ioport_interface, port_type)`](#function-vmm_io_port_add_handlerio_list-ioport_range-ioport_interface-port_type)

> [`emulate_io_handler(io_port, port_no, is_in, size, data)`](#function-emulate_io_handlerio_port-port_no-is_in-size-data)



**Structs**:

> [`ioport_range`](#struct-ioport_range)

> [`ioport_interface`](#struct-ioport_interface)

> [`ioport_entry`](#struct-ioport_entry)

> [`vmm_io_list`](#struct-vmm_io_list)


## Functions

The interface `ioports.h` defines the following functions.

### Function `vmm_io_port_init(io_list, ioport_alloc_addr)`

Initialize the io port list manager.
This is used when registering ioport handlers of type 'IOPORT_FREE'

**Parameters:**

- `io_list {vmm_io_port_list_t **}`: Pointer to io_port list handle. This will be allocated and initialised
- `ioport_alloc_addr {uint16_t}`: Base ioport address we can safely bump allocate from (doesn't conflict with other ioports).

**Returns:**

- 0 for success, otherwise -1 for error

Back to [interface description](#interface-ioportsh).

### Function `vmm_io_port_add_handler(io_list, ioport_range, ioport_interface, port_type)`

Add an io port range for emulation

**Parameters:**

- `io_list {vmm_io_port_list_t *}`: Handle to ioport list. This is where the new ioport handler will be appended to
- `ioport_range {ioport_range_t}`: Range the ioport handler will emulate
- `ioport_interface {ioport_interface_t}`: IOPort emulation interface
- `port_type {ioport_type_t}`: The type of ioport being registered - IOPORT_FREE, IOPORT_ADDR

**Returns:**

- NULL for error, otherwise pointer to newly created ioport entry

Back to [interface description](#interface-ioportsh).

### Function `emulate_io_handler(io_port, port_no, is_in, size, data)`

From a set of registered ioports, emulate an io instruction given a current ioport access.

**Parameters:**

- `io_port {vmm_io_port_list_t *}`: List of registered ioports with in/out handlers
- `port_no {unsigned int}`: IOPort address being accessed
- `is_in {bool}`: True if we are performing an io in operation, otherwise False
- `size {size_t}`: Size of io access
- `data {unsigned int *}`: Pointer with the data being written if io-out op, otherwise will be populated with data from an io-in op

**Returns:**

- 0 if handled, 1 if unhandled, otherwise -1 for error

Back to [interface description](#interface-ioportsh).


## Structs

The interface `ioports.h` defines the following structs.

### Struct `ioport_range`

Range of ioport handler

**Elements:**

- `start {uint16_t}`: Start address of ioport range
- `end {uint16_t}`: End address of ioport range
- `size {uint16_t}`: Size of ioport range

Back to [interface description](#interface-ioportsh).

### Struct `ioport_interface`

Datastructure used for ioport emulation, containing handlers for the ioport range

**Elements:**

- `cookie {void *}`: User supplied cookie to pass onto handlers
- `port_in {ioport_in_fn}`: IO in operation handler
- `port_out {ioport_out_fn}`: IO out operation handler
- `desc {const char *}`: IOPort description, useful for debugging

Back to [interface description](#interface-ioportsh).

### Struct `ioport_entry`

Datastructure used to present a registered ioport range

**Elements:**

- `range {ioport_range_t}`: IO address range of ioport entry
- `interface {ioport_interface_t}`: Emulation interface for ioport range

Back to [interface description](#interface-ioportsh).

### Struct `vmm_io_list`

Parent datastructure used to maintain a list of registered ioports

**Elements:**

- `num_ioports {int}`: Total number of registered ioports
- `List {ioport_entry_t **}`: of registered ioport objects
- `alloc_addr {uint16_t}`: Base ioport address we can safely bump allocate from, used when registering ioport handlers of type 'IOPORT_FREE'

Back to [interface description](#interface-ioportsh).


Back to [top](#).

