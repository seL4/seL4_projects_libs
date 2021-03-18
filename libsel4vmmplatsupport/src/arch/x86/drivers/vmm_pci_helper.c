/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_memory_helpers.h>
#include <sel4vm/arch/ioports.h>
#include <sel4vm/boot.h>

#include <sel4vmmplatsupport/guest_memory_util.h>
#include <sel4vmmplatsupport/arch/drivers/vmm_pci_helper.h>
#include <sel4vmmplatsupport/drivers/pci.h>
#include <sel4vmmplatsupport/drivers/pci_helper.h>

int vmm_pci_helper_map_bars(vm_t *vm, libpci_device_iocfg_t *cfg, vmm_pci_bar_t *bars)
{
    int i;
    int bar = 0;
    for (i = 0; i < 6; i++) {
        if (cfg->base_addr[i] == 0) {
            continue;
        }
        size_t size = cfg->base_addr_size[i];
        assert(size != 0);
        int size_bits = BYTES_TO_SIZE_BITS(size);
        if (BIT(size_bits) != size) {
            ZF_LOGE("PCI bar is not power of 2 size (%zu)", size);
            return -1;
        }
        bars[bar].size_bits = size_bits;
        if (cfg->base_addr_space[i] == PCI_BASE_ADDRESS_SPACE_MEMORY) {
            /* Need to map into the VMM. Make sure it is aligned */
            uintptr_t addr;
            vm_memory_reservation_t *reservation = vm_reserve_anon_memory(vm, size, size,
                                                                          default_error_fault_callback, NULL,
                                                                          &addr);
            if (!reservation) {
                ZF_LOGE("Failed to reserve PCI bar %p size %zu", (void *)(uintptr_t)cfg->base_addr[i], size);
                return -1;
            }
            /* Make sure that the address is naturally aligned to its size */
            if (addr % size) {
                ZF_LOGE("Guest PCI bar address 0x%lx is not aligned to size %zu", addr, size);
                return -1;
            }
            int err = map_ut_alloc_reservation_with_base_paddr(vm, (uintptr_t)cfg->base_addr[i], reservation);
            if (err) {
                ZF_LOGE("Failed to map PCI bar %p size %zu", (void *)(uintptr_t)cfg->base_addr[i], size);
                return -1;
            }
            bars[bar].address = addr;
            if (cfg->base_addr_prefetchable[i]) {
                bars[bar].mem_type = PREFETCH_MEM;
            } else {
                bars[bar].mem_type = NON_PREFETCH_MEM;
            }
        } else {
            /* Need to add the IO port range */
            int error = vm_enable_passthrough_ioport(vm->vcpus[BOOT_VCPU], cfg->base_addr[i], cfg->base_addr[i] + size - 1);
            if (error) {
                return error;
            }
            bars[bar].mem_type = NON_MEM;
            bars[bar].address = cfg->base_addr[i];
        }
        bar++;
    }
    return bar;
}

ioport_fault_result_t vmm_pci_io_port_in(vm_vcpu_t *vcpu, void *cookie, unsigned int port_no, unsigned int size,
                                         unsigned int *result)
{
    vmm_pci_space_t *self = (vmm_pci_space_t *)cookie;
    uint8_t offset;

    if (port_no >= PCI_CONF_PORT_ADDR && port_no < PCI_CONF_PORT_ADDR_END) {
        offset = port_no - PCI_CONF_PORT_ADDR;
        assert(port_no + size <= PCI_CONF_PORT_ADDR_END);
        /* Emulate read addr. */
        *result = 0;
        memcpy(result, ((char *)&self->conf_port_addr) + offset, size);
        return IO_FAULT_HANDLED;
    }
    assert(port_no >= PCI_CONF_PORT_DATA && port_no + size <= PCI_CONF_PORT_DATA_END);
    offset = port_no - PCI_CONF_PORT_DATA;

    /* construct a pci address from the current value in the config port */
    vmm_pci_address_t addr;
    uint8_t reg;
    make_addr_reg_from_config(self->conf_port_addr, &addr, &reg);
    reg += offset;

    /* Check if this device exists */
    vmm_pci_entry_t *dev = find_device(self, addr);
    if (!dev) {
        /* if the guest strayed from bus 0 then somethign went wrong. otherwise random reads
         * could just be it probing for devices */
        if (addr.bus != 0) {
            ZF_LOGI("Guest attempted access to non existent device %02x:%02x.%d register 0x%x", addr.bus, addr.dev, addr.fun, reg);
        } else {
            ZF_LOGI("Ignoring guest probe for device %02x:%02x.%d register 0x%x", addr.bus, addr.dev, addr.fun, reg);
        }
        *result = -1;
        return IO_FAULT_HANDLED;
    }
    int error = dev->ioread(dev->cookie, reg, size, result);
    if (error) {
        return IO_FAULT_ERROR;
    }
    /* Strip out any multi function reporting */
    if (reg + size > PCI_HEADER_TYPE && reg <= PCI_HEADER_TYPE) {
        /* This read overlapped with the header type, work out where it is and mask
         * the MF bit out */
        int header_offset = PCI_HEADER_TYPE - reg;
        unsigned int mf_mask = ~(BIT(7) << (header_offset * 8));
        (*result) &= mf_mask;
    }
    return IO_FAULT_HANDLED;
}

ioport_fault_result_t vmm_pci_io_port_out(vm_vcpu_t *vcpu, void *cookie, unsigned int port_no, unsigned int size,
                                          unsigned int value)
{
    vmm_pci_space_t *self = (vmm_pci_space_t *)cookie;
    uint8_t offset;

    if (port_no >= PCI_CONF_PORT_ADDR && port_no < PCI_CONF_PORT_ADDR_END) {
        offset = port_no - PCI_CONF_PORT_ADDR;
        assert(port_no + size <= PCI_CONF_PORT_ADDR_END);
        /* Emulated set addr. First mask out the bottom two bits of the address that
         * should never be used*/
        value &= ~MASK(2);
        memcpy(((char *)&self->conf_port_addr) + offset, &value, size);
        return IO_FAULT_HANDLED;
    }
    assert(port_no >= PCI_CONF_PORT_DATA && port_no + size <= PCI_CONF_PORT_DATA_END);
    offset = port_no - PCI_CONF_PORT_DATA;

    /* construct a pci address from the current value in the config port */
    vmm_pci_address_t addr;
    uint8_t reg;
    make_addr_reg_from_config(self->conf_port_addr, &addr, &reg);
    reg += offset;

    /* Check if this device exists */
    vmm_pci_entry_t *dev = find_device(self, addr);
    if (!dev) {
        ZF_LOGI("Guest attempted access to non existent device %02x:%02x.%d register 0x%x", addr.bus, addr.dev, addr.fun, reg);
        return IO_FAULT_HANDLED;
    }
    int err = dev->iowrite(dev->cookie, reg + offset, size, value);
    if (err) {
        return IO_FAULT_ERROR;
    }
    return IO_FAULT_HANDLED;
}
