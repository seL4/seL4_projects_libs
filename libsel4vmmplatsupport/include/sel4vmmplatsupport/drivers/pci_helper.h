/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/***
 * @module pci.h
 * This interface presents a series of helpers when using the VMM PCI Driver. These helpers assisting with
 * the creation of pci device entries and accessors the their configuration spaces.
 */

#include <stdint.h>
#include <pci/pci.h>

#include <sel4vmmplatsupport/drivers/pci.h>

#define PCI_BAR_OFFSET(b)   (offsetof(vmm_pci_device_def_t, bar##b))

/***
 * @struct vmm_pci_device_def
 * Struct definition of a PCI device. This is used for emulating a device from
 * purely memory reads. This is not generally useful on its own, but provides
 * a nice skeleton
 * @param {uint16_t} vendor_id
 * @param {uint16_t} device_id
 * @param {uint16_t} command
 * @param {uint16_t} status
 * @param {uint8_t} revision_id
 * @param {uint8_t} prog_if
 * @param {uint8_t} subclass
 * @param {uint8_t} class_code
 * @param {uint8_t} cache_line_size
 * @param {uint8_t} latency_timer
 * @param {uint8_t} header_type
 * @param {uint8_t} bist
 * @param {uint32_t} bar0
 * @param {uint32_t} bar1
 * @param {uint32_t} bar2
 * @param {uint32_t} bar3
 * @param {uint32_t} bar4
 * @param {uint32_t} bar5
 * @param {uint32_t} cardbus
 * @param {uint16_t} subsystem_vendor_id
 * @param {uint16_t} subsystem_id
 * @param {uint32_t} expansion_rom
 * @param {uint8_t} caps_pointer
 * @param {uint8_t} reserved1
 * @param {uint16_t} reserved2
 * @param {uint32_t} reserved3
 * @param {uint8_t} interrupt_line
 * @param {uint8_t} interrupt_pin
 * @param {uint8_t} min_grant
 * @param {uint8_t} max_latency
 * @param {int} caps_len
 * @param {void *} caps
 */
typedef struct vmm_pci_device_def {
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t command;
    uint16_t status;
    uint8_t revision_id;
    uint8_t prog_if;
    uint8_t subclass;
    uint8_t class_code;
    uint8_t cache_line_size;
    uint8_t latency_timer;
    uint8_t header_type;
    uint8_t bist;
    uint32_t bar0;
    uint32_t bar1;
    uint32_t bar2;
    uint32_t bar3;
    uint32_t bar4;
    uint32_t bar5;
    uint32_t cardbus;
    uint16_t subsystem_vendor_id;
    uint16_t subsystem_id;
    uint32_t expansion_rom;
    uint8_t caps_pointer;
    uint8_t reserved1;
    uint16_t reserved2;
    uint32_t reserved3;
    uint8_t interrupt_line;
    uint8_t interrupt_pin;
    uint8_t min_grant;
    uint8_t max_latency;
    /* Now additional pointer to arbitrary capabilities */
    int caps_len;
    void *caps;
} PACKED vmm_pci_device_def_t;

typedef enum pci_mem_type {
    NON_MEM = 0,
    NON_PREFETCH_MEM,
    PREFETCH_MEM
} pci_mem_type_t;

/***
 * @struct vmm_pci_bar
 * Represents a PCI bar within a device
 * @param {pci_mem_type_t} mem_type     Type of memory supporting PCI bar
 * @param {uintptr_t} address           Address of PCI bar
 * @param {size_t} size_bits            Size of PCI bar in bits
 */
typedef struct vmm_pci_bar {
    pci_mem_type_t mem_type;
    /* Address must be size aligned */
    uintptr_t address;
    size_t size_bits;
} vmm_pci_bar_t;

/***
 * @struct pci_bar_emulation
 * Wrapper datastructure over a pci entry and its configuration space. This is leveraged to emulate
 * BAR accesses in an entries configuration space
 * @param {vmm_pci_entry_t} passthrough     PCI entry being emulated
 * @param {int} num_bars                    Number of PCI bars
 * @param {vmm_pci_bar_t} bars              Set of PCI bars being emulated in the PCI entry
 * @param {uint32_t} bar_writes             Most recent write to each PCI bar in the configuration space. This avoids
 *                                          the guest OS over-writing elements in the configuration space
 */
typedef struct pci_bar_emulation {
    vmm_pci_entry_t passthrough;
    int num_bars;
    vmm_pci_bar_t bars[6];
    uint32_t bar_writes[6];
} pci_bar_emulation_t;

/***
 * @struct pci_irq_emulation
 * Wrapper datastructure over a pci entry and its configuration space. This is leveraged to emulate
 * IRQ line accesses in an entries configuration space
 * @param {vmm_pci_entry_t} passthrough     PCI entry being emulated
 * @param {int} irq                         IRQ line value in PCI entry
 */
typedef struct pci_irq_emulation {
    vmm_pci_entry_t passthrough;
    int irq;
} pci_irq_emulation_t;

/***
 * @struct pci_passthrough_device
 * Datastructure providing direct passthrough access to a pci entry configuration space
 * @param {vmm_pci_address_t} addr          Address of PCI device
 * @param {vmm_pci_config_t} config         Ops for accessing config space
 */
typedef struct pci_passthrough_device {
    /* The address on the host system of this device */
    vmm_pci_address_t addr;
    vmm_pci_config_t config;
} pci_passthrough_device_t;

/***
 * @struct pci_cap_emulation
 * Wrappper datastructure over a pci entry and its configuration space. This is leveraged to emulate
 * capabilities in an entries configuration space.
 * @param {vmm_pci_entry_t} passthrough     PCI entry being emulation
 * @param {int} num_caps                    Number of caps in capability space
 * @param {uint8_t *} caps                  Capability list
 * @param {int} num_ignore                  Number of disallowed/ignored capability ranges e.g. MSI capabilities
 * @param {uint8_t *} ignore_start          Array of starting indexes of ignored capability ranges
 * @param {uint8_t *} ignore_end            Array of ending idexes of ignored capabilities range
 */
typedef struct pci_cap_emulation {
    vmm_pci_entry_t passthrough;
    int num_caps;
    uint8_t *caps;
    int num_ignore;
    uint8_t *ignore_start;
    uint8_t *ignore_end;
} pci_cap_emulation_t;

/***
 * @function vmm_pci_entry_ignore_write(cookie, offset, size, value)
 * Helper write function that just ignores any writes
 * @param {void *} cookie           User supplied PCI entry cookie
 * @param {int} offset              Offset into PCI device header
 * @param {int} size                Size of data to be written
 * @param {uint32_t} value          value to write to PCI device header offset
 * @return                          Returns 0
 */
int vmm_pci_entry_ignore_write(void *cookie, int offset, int size, uint32_t value);

/***
 * @function vmm_pci_mem_device_read(cookie, offset, size, result)
 * Read method for a PCI devices memory
 * @param {void *} cookie       PCI device header
 * @param {int} offset          Offset into PCI device header
 * @param {int} size            Size of data to be read
 * @result {uint32_t *} result  Resulting value read back from PCI device header
 * @return                      0 if success, -1 if error
 */
int vmm_pci_mem_device_read(void *cookie, int offset, int size, uint32_t *result);

/***
 * @function vmm_pci_mem_device_write(cookie, offset, size, value)
 * Write method for a PCI devices memory
 * @param {void *} cookie       PCI device header
 * @param {int} offset          Offset into PCI device header
 * @param {int} size            Size of data to be read
 * @value {uint32_t} value      Value to write to PCI device header offset
 * @return                      0 if success, -1 if error
 */
int vmm_pci_mem_device_write(void *cookie, int offset, int size, uint32_t value);

/***
 * @function define_pci_host_bridge(bridge)
 * Defines the configuration space values of the PCI host bridge
 * @param {vmm_pci_device_def_t *} bridge       PCI bridge device definition
 */
void define_pci_host_bridge(vmm_pci_device_def_t *bridge);

/***
 * @function vmm_pci_create_passthrough(addr, config)
 * Construct a pure passthrough device based on the real PCI. This is almost always useless as
 * you will almost certainly want to rebase io memory
 * @param {vmm_pci_address_t} addr      Address of passthrough PCI device
 * @param {vmm_pci_config_t} config     Ops for accessing the passthrough config space
 * @return                              `vmm_pci_entry_t` for passthrough device
 */
vmm_pci_entry_t vmm_pci_create_passthrough(vmm_pci_address_t addr, vmm_pci_config_t config);

/***
 * @function vmm_pci_create_bar_emulation(existing, num_bars, bars)
 * Construct a pci entry that emulates configuration space bar read/write's. The rest of the configuration space is passed on
 * @param {vmm_pci_entry_t} existing    Existing PCI entry to wrap over and emulate its bar accesses
 * @param {int} num_bars                Number of emulated bars in PCI entry
 * @param {vmm_pci_bar_t *} bars        Set of bars to emulate access to
 * @return                              `vmm_pci_entry_t` for emulated bar device
 */
vmm_pci_entry_t vmm_pci_create_bar_emulation(vmm_pci_entry_t existing, int num_bars, vmm_pci_bar_t *bars);

/***
 * @function vmm_pci_create_irq_emulation(existing, irq)
 * Construct a pci entry the emulates configuration space interrupt read/write's. The rest of the configuration space is passed on
 * @param {vmm_pci_entry_t} existing    Existing PCI entry to wrap over and emulate its IRQ accesses
 * @param {int} irq                     IRQ line value in PCI entry
 * @return                              `vmm_pci_entry_t` for emulated irq device
 */
vmm_pci_entry_t vmm_pci_create_irq_emulation(vmm_pci_entry_t existing, int irq);

/***
 * @function vmm_pci_create_cap_emulation(existing, num_caps, cap, num_ranges, range_starts, range_ends)
 * Capability space emulation. Takes list of addresses to use to form a capability linked list, as well as a
 * ranges of the capability space that should be directly disallowed. Assumes a type 0 device.
 * @param {vmm_pci_entry_t} existing    Existing PCI entry to wrap over and emulated it capability space accesses
 * @param {int} num_caps                Number of caps in capability space
 * @param {uint8_t *} caps              Capability list
 * @param {int} num_ranges              Number of disallowed/ignored capability ranges e.g. MSI capabilities
 * @param {uint8_t *} range_starts      Array of starting indexes of ignored capability ranges
 * @param {uint8_t *} range_end         Array of ending idexes of ignored capabilities range
 * @return                              `vmm_pci_entry_t` with an emulated capability space
 */
vmm_pci_entry_t vmm_pci_create_cap_emulation(vmm_pci_entry_t existing, int num_caps, uint8_t *caps, int num_ranges,
                                             uint8_t *range_starts, uint8_t *range_ends);

/***
 * Finds the MSI capabilities and uses vmm_pci_create_cap_emulation to register them as ignored
 * @param {vmm_pci_entry_t} existing    Existing PCI entry to wrap over with ignored MSI capabilities
 * @return                              `vmm_pci_entry_t` with an emulated capability space (ignoring MSI capabilties)
 */
vmm_pci_entry_t vmm_pci_no_msi_cap_emulation(vmm_pci_entry_t existing);
