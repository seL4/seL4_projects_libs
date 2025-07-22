<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `vpci.h`

The libsel4vmmplatsupport vpci interface presents a Virtual PCI driver for ARM-based VM's.
Using the `vmm_pci_space_t` management interface, the vpci driver establishes the configuration
space in the guest VM. The driver also handles and processes all subsequent memory and ioport accesses to the
virtual pci device.

### Brief content:

**Functions**:

> [`vm_install_vpci(vm, io_port, pci)`](#function-vm_install_vpcivm-io_port-pci)

> [`fdt_generate_vpci_node(vm, pci, fdt, gic_phandle)`](#function-fdt_generate_vpci_nodevm-pci-fdt-gic_phandle)


## Functions

The interface `vpci.h` defines the following functions.

### Function `vm_install_vpci(vm, io_port, pci)`



**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `io_port {vmm_io_port_list_t *}`: IOPort library instance to emulate io accesses with
- `pci {vmm_pci_space_t }`: PCI library instance to emulate PCI device accesses with

**Returns:**

- 0 for success, -1 for error

Back to [interface description](#interface-vpcih).

### Function `fdt_generate_vpci_node(vm, pci, fdt, gic_phandle)`

Generate a PCI device node for a given fdt. This taking into account
the virtual PCI device configuration space.

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `PCI {vmm_pci_space_t *}`: library instance to generate fdt node
- `fdt {void *}`: FDT blob to append generated device node
- `gic_phandle {int}`: Phandle of IRQ controller to generate a correct interrupt map property

**Returns:**

- 0 for success, -1 for error

Back to [interface description](#interface-vpcih).


Back to [top](#).

