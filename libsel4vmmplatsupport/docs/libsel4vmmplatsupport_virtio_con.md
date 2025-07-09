<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `virtio_con.h`

This interface provides the ability to initalise a VMM virtio console driver. This creating a virtio
PCI device in the VM's virtual pci. This can subsequently be accessed through '/dev/hvc0' in the guest.

### Brief content:

**Functions**:

> [`common_make_virtio_con(vm, pci, ioport, ioport_range, port_type, interrupt_pin, interrupt_lin, backend)`](#function-common_make_virtio_convm-pci-ioport-ioport_range-port_type-interrupt_pin-interrupt_lin-backend)



**Structs**:

> [`virtio_con`](#struct-virtio_con)


## Functions

The interface `virtio_con.h` defines the following functions.

### Function `common_make_virtio_con(vm, pci, ioport, ioport_range, port_type, interrupt_pin, interrupt_lin, backend)`

Initialise a new virtio_con device with Base Address Registers (BARs) starting at iobase and backend functions
specified by the console_passthrough struct.
virtio_con_default_backend for default methods.

**Parameters:**

- `vm {vm_t *}`: Handle to the VM
- `pci {vmm_pci_space_t *}`: PCI library instance to register virtio con device
- `ioport {vmm_io_port_list_t *}`: IOPort library instance to register virtio con ioport
- `ioport_range {ioport_range_t}`: BAR port for front end emulation
- `iotype {ioport_type_t}`: Type of ioport i.e. whether to alloc or use given range
- `interrupt_pin {unsigned int}`: PCI interrupt pin e.g. INTA = 1, INTB = 2 ,...
- `interrupt_line {unsigned int}`: PCI interrupt line for virtio con IRQS
- `backend {struct console_passthrough}`: Function pointers to backend implementation. Can be initialised by

**Returns:**

- Pointer to an initialised virtio_con_t, NULL if error.

Back to [interface description](#interface-virtio_conh).


## Structs

The interface `virtio_con.h` defines the following structs.

### Struct `virtio_con`

Virtio Console Driver Interface

**Elements:**

- `iobase {unsigned int}`: IO Port base for virtio con device
- `emul {virtio_emul_t *}`: Virtio console emulation interface: VMM <-> Guest
- `emul_driver_funcs {struct console_passthrough}`: Virtio console emulation functions: VMM <-> Guest
- `ioops {ps_io_ops_t}`: Platform support io ops datastructure

Back to [interface description](#interface-virtio_conh).


Back to [top](#).

