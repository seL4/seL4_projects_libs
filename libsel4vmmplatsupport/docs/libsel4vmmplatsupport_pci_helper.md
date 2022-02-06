<!--
     Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

## Interface `pci.h`

This interface presents a series of helpers when using the VMM PCI Driver. These helpers assisting with
the creation of pci device entries and accessors the their configuration spaces.

### Brief content:

**Functions**:

> [`vmm_pci_entry_ignore_write(cookie, offset, size, value)`](#function-vmm_pci_entry_ignore_writecookie-offset-size-value)

> [`vmm_pci_mem_device_read(cookie, offset, size, result)`](#function-vmm_pci_mem_device_readcookie-offset-size-result)

> [`vmm_pci_mem_device_write(cookie, offset, size, value)`](#function-vmm_pci_mem_device_writecookie-offset-size-value)

> [`define_pci_host_bridge(bridge)`](#function-define_pci_host_bridgebridge)

> [`vmm_pci_create_passthrough(addr, config)`](#function-vmm_pci_create_passthroughaddr-config)

> [`vmm_pci_create_bar_emulation(existing, num_bars, bars)`](#function-vmm_pci_create_bar_emulationexisting-num_bars-bars)

> [`vmm_pci_create_irq_emulation(existing, irq)`](#function-vmm_pci_create_irq_emulationexisting-irq)

> [`vmm_pci_create_cap_emulation(existing, num_caps, cap, num_ranges, range_starts, range_ends)`](#function-vmm_pci_create_cap_emulationexisting-num_caps-cap-num_ranges-range_starts-range_ends)



**Structs**:

> [`vmm_pci_device_def`](#struct-vmm_pci_device_def)

> [`vmm_pci_bar`](#struct-vmm_pci_bar)

> [`pci_bar_emulation`](#struct-pci_bar_emulation)

> [`pci_irq_emulation`](#struct-pci_irq_emulation)

> [`pci_passthrough_device`](#struct-pci_passthrough_device)

> [`pci_cap_emulation`](#struct-pci_cap_emulation)


## Functions

The interface `pci.h` defines the following functions.

### Function `vmm_pci_entry_ignore_write(cookie, offset, size, value)`

Helper write function that just ignores any writes

**Parameters:**

- `cookie {void *}`: User supplied PCI entry cookie
- `offset {int}`: Offset into PCI device header
- `size {int}`: Size of data to be written
- `value {uint32_t}`: value to write to PCI device header offset

**Returns:**

- Returns 0

Back to [interface description](#module-pcih).

### Function `vmm_pci_mem_device_read(cookie, offset, size, result)`

Read method for a PCI devices memory
@result {uint32_t *} result  Resulting value read back from PCI device header

**Parameters:**

- `cookie {void *}`: PCI device header
- `offset {int}`: Offset into PCI device header
- `size {int}`: Size of data to be read

**Returns:**

- 0 if success, -1 if error

Back to [interface description](#module-pcih).

### Function `vmm_pci_mem_device_write(cookie, offset, size, value)`

Write method for a PCI devices memory
@value {uint32_t} value      Value to write to PCI device header offset

**Parameters:**

- `cookie {void *}`: PCI device header
- `offset {int}`: Offset into PCI device header
- `size {int}`: Size of data to be read

**Returns:**

- 0 if success, -1 if error

Back to [interface description](#module-pcih).

### Function `define_pci_host_bridge(bridge)`

Defines the configuration space values of the PCI host bridge

**Parameters:**

- `bridge {vmm_pci_device_def_t *}`: PCI bridge device definition

**Returns:**

No return

Back to [interface description](#module-pcih).

### Function `vmm_pci_create_passthrough(addr, config)`

Construct a pure passthrough device based on the real PCI. This is almost always useless as
you will almost certainly want to rebase io memory

**Parameters:**

- `addr {vmm_pci_address_t}`: Address of passthrough PCI device
- `config {vmm_pci_config_t}`: Ops for accessing the passthrough config space

**Returns:**

- `vmm_pci_entry_t` for passthrough device

Back to [interface description](#module-pcih).

### Function `vmm_pci_create_bar_emulation(existing, num_bars, bars)`

Construct a pci entry that emulates configuration space bar read/write's. The rest of the configuration space is passed on

**Parameters:**

- `existing {vmm_pci_entry_t}`: Existing PCI entry to wrap over and emulate its bar accesses
- `num_bars {int}`: Number of emulated bars in PCI entry
- `bars {vmm_pci_bar_t *}`: Set of bars to emulate access to

**Returns:**

- `vmm_pci_entry_t` for emulated bar device

Back to [interface description](#module-pcih).

### Function `vmm_pci_create_irq_emulation(existing, irq)`

Construct a pci entry the emulates configuration space interrupt read/write's. The rest of the configuration space is passed on

**Parameters:**

- `existing {vmm_pci_entry_t}`: Existing PCI entry to wrap over and emulate its IRQ accesses
- `irq {int}`: IRQ line value in PCI entry

**Returns:**

- `vmm_pci_entry_t` for emulated irq device

Back to [interface description](#module-pcih).

### Function `vmm_pci_create_cap_emulation(existing, num_caps, cap, num_ranges, range_starts, range_ends)`

Capability space emulation. Takes list of addresses to use to form a capability linked list, as well as a
ranges of the capability space that should be directly disallowed. Assumes a type 0 device.

**Parameters:**

- `existing {vmm_pci_entry_t}`: Existing PCI entry to wrap over and emulated it capability space accesses
- `num_caps {int}`: Number of caps in capability space
- `caps {uint8_t *}`: Capability list
- `num_ranges {int}`: Number of disallowed/ignored capability ranges e.g. MSI capabilities
- `range_starts {uint8_t *}`: Array of starting indexes of ignored capability ranges
- `range_end {uint8_t *}`: Array of ending idexes of ignored capabilities range

**Returns:**

- `vmm_pci_entry_t` with an emulated capability space

Back to [interface description](#module-pcih).


## Structs

The interface `pci.h` defines the following structs.

### Struct `vmm_pci_device_def`

Struct definition of a PCI device. This is used for emulating a device from
purely memory reads. This is not generally useful on its own, but provides
a nice skeleton

**Elements:**

- `vendor_id {uint16_t}`
- `device_id {uint16_t}`
- `command {uint16_t}`
- `status {uint16_t}`
- `revision_id {uint8_t}`
- `prog_if {uint8_t}`
- `subclass {uint8_t}`
- `class_code {uint8_t}`
- `cache_line_size {uint8_t}`
- `latency_timer {uint8_t}`
- `header_type {uint8_t}`
- `bist {uint8_t}`
- `bar0 {uint32_t}`
- `bar1 {uint32_t}`
- `bar2 {uint32_t}`
- `bar3 {uint32_t}`
- `bar4 {uint32_t}`
- `bar5 {uint32_t}`
- `cardbus {uint32_t}`
- `subsystem_vendor_id {uint16_t}`
- `subsystem_id {uint16_t}`
- `expansion_rom {uint32_t}`
- `caps_pointer {uint8_t}`
- `reserved1 {uint8_t}`
- `reserved2 {uint16_t}`
- `reserved3 {uint32_t}`
- `interrupt_line {uint8_t}`
- `interrupt_pin {uint8_t}`
- `min_grant {uint8_t}`
- `max_latency {uint8_t}`
- `caps_len {int}`
- `caps {void *}`

Back to [interface description](#module-pcih).

### Struct `vmm_pci_bar`

Represents a PCI bar within a device

**Elements:**

- `mem_type {pci_mem_type_t}`: Type of memory supporting PCI bar
- `address {uintptr_t}`: Address of PCI bar
- `size_bits {size_t}`: Size of PCI bar in bits

Back to [interface description](#module-pcih).

### Struct `pci_bar_emulation`

Wrapper datastructure over a pci entry and its configuration space. This is leveraged to emulate
BAR accesses in an entries configuration space
the guest OS over-writing elements in the configuration space

**Elements:**

- `passthrough {vmm_pci_entry_t}`: PCI entry being emulated
- `num_bars {int}`: Number of PCI bars
- `bars {vmm_pci_bar_t}`: Set of PCI bars being emulated in the PCI entry
- `bar_writes {uint32_t}`: Most recent write to each PCI bar in the configuration space. This avoids

Back to [interface description](#module-pcih).

### Struct `pci_irq_emulation`

Wrapper datastructure over a pci entry and its configuration space. This is leveraged to emulate
IRQ line accesses in an entries configuration space

**Elements:**

- `passthrough {vmm_pci_entry_t}`: PCI entry being emulated
- `irq {int}`: IRQ line value in PCI entry

Back to [interface description](#module-pcih).

### Struct `pci_passthrough_device`

Datastructure providing direct passthrough access to a pci entry configuration space

**Elements:**

- `addr {vmm_pci_address_t}`: Address of PCI device
- `config {vmm_pci_config_t}`: Ops for accessing config space

Back to [interface description](#module-pcih).

### Struct `pci_cap_emulation`

Wrappper datastructure over a pci entry and its configuration space. This is leveraged to emulate
capabilities in an entries configuration space.

**Elements:**

- `passthrough {vmm_pci_entry_t}`: PCI entry being emulation
- `num_caps {int}`: Number of caps in capability space
- `caps {uint8_t *}`: Capability list
- `num_ignore {int}`: Number of disallowed/ignored capability ranges e.g. MSI capabilities
- `ignore_start {uint8_t *}`: Array of starting indexes of ignored capability ranges
- `ignore_end {uint8_t *}`: Array of ending idexes of ignored capabilities range

Back to [interface description](#module-pcih).


Back to [top](#).

