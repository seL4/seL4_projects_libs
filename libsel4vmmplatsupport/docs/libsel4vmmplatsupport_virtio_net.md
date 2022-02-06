<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `virtio_net.h`

This interface provides the ability to initalise a VMM virtio net driver. This creating a virtio
PCI device in the VM's virtual pci. This can subsequently be accessed as an ethernet interface in the
guest.

### Brief content:

**Functions**:

> [`common_make_virtio_net(vm, pci, ioport, ioport_range, port_type, interrupt_pin, interrupt_line, backend)`](#function-common_make_virtio_netvm-pci-ioport-ioport_range-port_type-interrupt_pin-interrupt_line-backend)

> [`virtio_net_default_backend()`](#function-virtio_net_default_backend)



**Structs**:

> [`virtio_net`](#struct-virtio_net)


## Functions

The interface `virtio_net.h` defines the following functions.

### Function `common_make_virtio_net(vm, pci, ioport, ioport_range, port_type, interrupt_pin, interrupt_line, backend)`

Initialise a new virtio_net device with Base Address Registers (BARs) starting at iobase and backend functions
specified by the raw_iface_funcs struct.
virtio_net_default_backend for default methods.

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `pci {vmm_pci_space_t *}`: PCI library instance to register virtio net device
- `ioport {vmm_io_port_list_t *}`: IOPort library instance to register virtio net ioport
- `ioport_range {ioport_range_t}`: BAR port for front end emulation
- `port_type {ioport_type_t}`: Type of ioport i.e. whether to alloc or use given range
- `interrupt_pin {unsigned int}`: PCI interrupt pin e.g. INTA = 1, INTB = 2 ,...
- `interrupt_line {unsigned int}`: PCI interrupt line for virtio net IRQS
- `backend {struct raw_iface_funcs}`: Function pointers to backend implementation. Can be initialised by

**Returns:**

- Pointer to an initialised virtio_net_t, NULL if error.

Back to [interface description](#module-virtio_neth).

### Function `virtio_net_default_backend()`

update these function pointers with its own custom backend.

**Parameters:**

No parameters

**Returns:**

- A struct with a default virtio_net backend. It is the responsibility of the caller to

Back to [interface description](#module-virtio_neth).


## Structs

The interface `virtio_net.h` defines the following structs.

### Struct `virtio_net`

Virtio Net Driver Interface

**Elements:**

- `iobase {unsigned int}`: IO Port base for Virtio Net device
- `emul {virtio_emul_t *}`: Virtio Ethernet emulation interface: VMM <-> Guest
- `emul_driver {struct eth_driver *}`: Backend Ethernet driver interface: VMM <-> Ethernet driver
- `emul_driver_funcs {struct raw_iface_funcs}`: Virtio Ethernet emulation functions: VMM <-> Guest
- `ioops {ps_io_ops_t}`: Platform support ioops for dma management

Back to [interface description](#module-virtio_neth).


Back to [top](#).

