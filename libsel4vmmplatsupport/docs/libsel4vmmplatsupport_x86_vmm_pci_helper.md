<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `vmm_pci_helper.h`

The interface presents a series of helpers for establishing VMM PCI support on x86 platforms.

### Brief content:

**Functions**:

> [`vmm_pci_helper_map_bars(vm, cfg, bars)`](#function-vmm_pci_helper_map_barsvm-cfg-bars)

> [`vmm_pci_io_port_in(vcpu, cookie, port_no, size, result)`](#function-vmm_pci_io_port_invcpu-cookie-port_no-size-result)

> [`vmm_pci_io_port_out(vcpu, cookie, port_no, size, value)`](#function-vmm_pci_io_port_outvcpu-cookie-port_no-size-value)


## Functions

The interface `vmm_pci_helper.h` defines the following functions.

### Function `vmm_pci_helper_map_bars(vm, cfg, bars)`

Given a PCI device config, map the PCI device bars into the VM, effectively passing-through the
PCI device. This will map MMIO and IO-based bars.

**Parameters:**

- `vm {vm_t *}`: A handle to the VM
- `cfg {libpci_device_iocfg_t *}`: PCI device config
- `bars {vmm_pci_bar_t *}`: Resulting PCI bars mapped into the VM

**Returns:**

- -1 for error, otherwise the number of bars mapped into the VM (>=0)

Back to [interface description](#interface-vmm_pci_helperh).

### Function `vmm_pci_io_port_in(vcpu, cookie, port_no, size, result)`

Emulates IOPort in access on the VMM Virtual PCI device

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Faulting vcpu performing ioport access
- `port_no {unsigned int}`: Port address being accessed
- `size {unsigned int}`: Size of ioport access
- `result {unsigned int *}`: Pointer that will be populated with resulting data of io-in op

**Returns:**

No return

Back to [interface description](#interface-vmm_pci_helperh).

### Function `vmm_pci_io_port_out(vcpu, cookie, port_no, size, value)`

Emulates IOPort out access on the VMM Virtual PCI device

**Parameters:**

- `vcpu {vm_vcpu_t *}`: Faulting vcpu performing ioport access
- `port_no {unsigned int}`: Port address being accessed
- `size {unsigned int}`: Size of ioport access
- `value {unsigned int}`: Value being written in io-out op

**Returns:**

No return

Back to [interface description](#interface-vmm_pci_helperh).


Back to [top](#).

