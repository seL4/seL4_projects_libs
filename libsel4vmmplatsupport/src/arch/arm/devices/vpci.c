/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sel4/sel4.h>

#include <pci/virtual_pci.h>
#include <pci/helper.h>
#include <pci/pci.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_vcpu_fault.h>

#include <sel4vmmplatsupport/drivers/pci_helper.h>
#include <sel4vmmplatsupport/drivers/pci.h>
#include <sel4vmmplatsupport/ioports.h>

#include <sel4vmmplatsupport/device.h>
#include <sel4vmmplatsupport/arch/vpci.h>

#include <libfdt.h>
#include <fdtgen.h>

#define FDT_OP(op)                                      \
    do {                                                \
        int err = (op);                                 \
        ZF_LOGF_IF(err < 0, "FDT operation failed");    \
    } while(0)                                          \

#define PCI_RANGE_IO 1
#define PCI_RANGE_MEM32 2

#define PCI_ADDR_FUNC_SHIFT 8
#define PCI_ADDR_DEV_SHIFT 11
#define PCI_ADDR_BUS_SHIFT 16

struct pci_fdt_address {
    uint32_t hi;
    uint32_t mid;
    uint32_t low;
} PACKED;

struct pci_interrupt_map_mask {
    struct pci_fdt_address pci_addr;
    uint32_t irq_pin;
} PACKED;

struct pci_interrupt_map {
    struct pci_interrupt_map_mask pci_mask;
    uint32_t gic_phandle;
#if GIC_ADDRESS_CELLS == 0x1
    /* FIXME Ideally we want to extract the address cells value out of the irq controller fdt node */
    uint32_t irq_type;
    uint32_t irq_num;
#elif GIC_ADDRESS_CELLS == 0x2
    uint64_t irq_type;
    uint64_t irq_num;
#else
#error "Undefined GIC Address Cells"
#endif
    uint32_t irq_flags;
} PACKED;

struct pci_cfg_data {
    vmm_io_port_list_t *io_port;
    vmm_pci_space_t *pci;
};

static void pci_cfg_read_fault(vm_vcpu_t *vcpu, uint8_t offset, size_t len, vmm_pci_entry_t *dev)
{
    uint32_t data = 0;
    int err = 0;

    err = dev->ioread((void *)dev->cookie, offset, len, &data);
    if (err) {
        ZF_LOGE("Failure performing read from PCI CFG device");
    }

    seL4_Word s = (get_vcpu_fault_address(vcpu) & 0x3) * 8;
    set_vcpu_fault_data(vcpu, data << s);
}

static void pci_cfg_write_fault(vm_vcpu_t *vcpu, uint8_t offset, size_t len, vmm_pci_entry_t *dev)
{
    uint32_t mask;
    uint32_t value;
    int err;

    mask = get_vcpu_fault_data_mask(vcpu);
    value = get_vcpu_fault_data(vcpu) & mask;

    err = dev->iowrite((void *)dev->cookie, offset, len, value);
    if (err) {
        ZF_LOGE("Failure writing to PCI CFG device");
    }
}

static memory_fault_result_t pci_cfg_fault_handler(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr, size_t fault_length,
                                                   void *cookie)
{
    uint8_t offset;
    vmm_pci_address_t pci_addr;
    struct device *dev = (struct device *)cookie;
    struct pci_cfg_data *cfg_data = (struct pci_cfg_data *)dev->priv;
    vmm_pci_space_t *pci = cfg_data->pci;

    fault_addr -= PCI_CFG_REGION_ADDR;

    make_addr_reg_from_config(fault_addr, &pci_addr, &offset);
    pci_addr.fun = 0;

    vmm_pci_entry_t *pci_dev = find_device(pci, pci_addr);
    if (!pci_dev) {
        ZF_LOGW("Failed to find pci device B:%d D:%d F:%d", pci_addr.bus, pci_addr.dev, pci_addr.fun);
        /* No device found */
        advance_vcpu_fault(vcpu);
        return FAULT_HANDLED;
    }

    if (is_vcpu_read_fault(vcpu)) {
        pci_cfg_read_fault(vcpu, offset, fault_length, pci_dev);
    } else {
        pci_cfg_write_fault(vcpu, offset, fault_length, pci_dev);
    }

    advance_vcpu_fault(vcpu);
    return FAULT_HANDLED;
}

static memory_fault_result_t pci_cfg_io_fault_handler(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                      size_t fault_length, void *cookie)
{
    struct device *dev = (struct device *)cookie;
    /* Get CFG Port address */
    uint16_t cfg_port = (fault_addr - dev->pstart) & USHRT_MAX;
    unsigned int value = 0;
    seL4_Word fault_data = 0;
    struct pci_cfg_data *cfg_data = (struct pci_cfg_data *)dev->priv;
    vmm_io_port_list_t *io_port = cfg_data->io_port;

    /* Determine io direction */
    bool is_in = false;
    if (is_vcpu_read_fault(vcpu)) {
        is_in = true;
    } else {
        value = get_vcpu_fault_data(vcpu);
    }
    /* Emulate IO */
    emulate_io_handler(io_port, cfg_port, is_in, fault_length, (void *)&value);

    if (is_in) {
        memcpy(&fault_data, (void *)&value, fault_length);
        seL4_Word s = (fault_addr & 0x3) * 8;
        set_vcpu_fault_data(vcpu, fault_data << s);
    }

    advance_vcpu_fault(vcpu);
    return FAULT_HANDLED;
}

struct device dev_vpci_cfg = {
    .name = "vpci.cfg",
    .pstart = PCI_CFG_REGION_ADDR,
    .size = PCI_CFG_REGION_SIZE,
    .priv = NULL,
};

struct device dev_vpci_cfg_io = {
    .name = "vpci.cfg_io",
    .pstart = PCI_IO_REGION_ADDR,
    .size = PCI_IO_REGION_SIZE,
    .priv = NULL,
};

int vm_install_vpci(vm_t *vm, vmm_io_port_list_t *io_port, vmm_pci_space_t *pci)
{

    ps_io_ops_t *ops = vm->io_ops;
    struct pci_cfg_data *cfg_data;
    int err = ps_calloc(&ops->malloc_ops, 1, sizeof(struct pci_cfg_data), (void **)&cfg_data);
    if (err) {
        ZF_LOGE("Failed to install VPCI: Failed allocate pci cfg io data");
        return -1;
    }
    cfg_data->io_port = io_port;
    cfg_data->pci = pci;

    /* Install base VPCI CFG region */
    dev_vpci_cfg.priv = (void *)cfg_data;
    vm_memory_reservation_t *cfg_reservation = vm_reserve_memory_at(vm, dev_vpci_cfg.pstart, dev_vpci_cfg.size,
                                                                    pci_cfg_fault_handler, (void *)&dev_vpci_cfg);
    if (!cfg_reservation) {
        return -1;
    }

    /* Install base VPCI CFG IOPort region */
    dev_vpci_cfg_io.priv = (void *)cfg_data;
    vm_memory_reservation_t *cfg_io_reservation = vm_reserve_memory_at(vm, dev_vpci_cfg_io.pstart, dev_vpci_cfg_io.size,
                                                                       pci_cfg_io_fault_handler, (void *)&dev_vpci_cfg_io);
    if (!cfg_io_reservation) {
        return -1;
    }
    return 0;
}

static int append_prop_with_cells(void *fdt, int offset,  uint64_t val, int num_cells, const char *name)
{
    int err;
    if (num_cells == 2) {
        err = fdt_appendprop_u64(fdt, offset, name, val);
    } else if (num_cells == 1) {
        err = fdt_appendprop_u32(fdt, offset, name, val);
    } else {
        ZF_LOGE("non-supported arch");
        err = -1;
    }

    return err;
}

int fdt_generate_vpci_node(vm_t *vm, vmm_pci_space_t *pci, void *fdt, int gic_phandle)
{
    int err;
    int root_offset = fdt_path_offset(fdt, "/");
    int address_cells = fdt_address_cells(fdt, root_offset);
    int size_cells = fdt_size_cells(fdt, root_offset);

    int pci_node = fdt_add_subnode(fdt, root_offset, "pci");
    if (pci_node < 0) {
        return pci_node;
    }

    /* Basic PCI Properties*/
    FDT_OP(append_prop_with_cells(fdt, pci_node, 0x3, 1, "#address-cells"));
    FDT_OP(append_prop_with_cells(fdt, pci_node, 0x2, 1, "#size-cells"));
    FDT_OP(append_prop_with_cells(fdt, pci_node, 0x1, 1, "#interrupt-cells"));
    FDT_OP(fdt_appendprop_string(fdt, pci_node, "compatible", "pci-host-cam-generic"));
    FDT_OP(fdt_appendprop_string(fdt, pci_node, "device_type", "pci"));
    FDT_OP(fdt_appendprop(fdt, pci_node, "dma-coherent", NULL, 0));
    FDT_OP(append_prop_with_cells(fdt, pci_node, 0x0, 1, "bus-range"));
    FDT_OP(append_prop_with_cells(fdt, pci_node, 0x1, 1, "bus-range"));

    /* PCI Host CFG Region */
    FDT_OP(append_prop_with_cells(fdt, pci_node, PCI_CFG_REGION_ADDR, address_cells, "reg"));
    FDT_OP(append_prop_with_cells(fdt, pci_node, PCI_CFG_REGION_SIZE, size_cells, "reg"));

    /* PCI IO Region Range */
    struct pci_fdt_address pci_io_range_addr;
    pci_io_range_addr.hi = cpu_to_fdt32(PCI_RANGE_IO << 24);
    pci_io_range_addr.mid = 0;
    pci_io_range_addr.low = 0;
    FDT_OP(fdt_appendprop(fdt, pci_node, "ranges", &pci_io_range_addr, sizeof(struct pci_fdt_address)));
    FDT_OP(append_prop_with_cells(fdt, pci_node, PCI_IO_REGION_ADDR, address_cells, "ranges"));
    FDT_OP(fdt_appendprop_u64(fdt, pci_node, "ranges", PCI_IO_REGION_SIZE));

    /* PCI Mem Region Range */
    struct pci_fdt_address pci_mem_range_addr;
    pci_mem_range_addr.hi = cpu_to_fdt32(PCI_RANGE_MEM32 << 24);
    pci_mem_range_addr.mid = cpu_to_fdt32(PCI_MEM_REGION_ADDR >> 32);
    pci_mem_range_addr.low = cpu_to_fdt32((uint32_t)PCI_MEM_REGION_ADDR);
    FDT_OP(fdt_appendprop(fdt, pci_node, "ranges", &pci_mem_range_addr, sizeof(pci_mem_range_addr)));
    FDT_OP(append_prop_with_cells(fdt, pci_node, PCI_MEM_REGION_ADDR, address_cells, "ranges"));
    FDT_OP(fdt_appendprop_u64(fdt, pci_node, "ranges", PCI_MEM_REGION_SIZE));

    /* PCI IRQ map */
    bool is_irq_map = false;
    /* The first device is always the bridge (which doesn't need to be recorded in the ranges) */
    for (int i = 1; i < 32; i++) {
        if (pci->bus0[i][0]) {
            pci_bar_emulation_t *bar_emul = (pci_bar_emulation_t *)(pci->bus0[i][0])->cookie;
            vmm_pci_entry_t entry = bar_emul->passthrough;
            vmm_pci_device_def_t *pci_config = (vmm_pci_device_def_t *)entry.cookie;
            struct pci_interrupt_map irq_map;
            irq_map.pci_mask.pci_addr.hi  = cpu_to_fdt32(i << PCI_ADDR_DEV_SHIFT);
            irq_map.pci_mask.pci_addr.mid  = 0;
            irq_map.pci_mask.pci_addr.low  = 0;
            irq_map.pci_mask.irq_pin = cpu_to_fdt32(pci_config->interrupt_pin);
            irq_map.gic_phandle = cpu_to_fdt32(gic_phandle);
            irq_map.irq_type = 0;
#if GIC_ADDRESS_CELLS == 0x1
            irq_map.irq_num = cpu_to_fdt32(pci_config->interrupt_line - 32);
#else
            irq_map.irq_num = cpu_to_fdt64(pci_config->interrupt_line - 32);
#endif
            irq_map.irq_flags = cpu_to_fdt32(0x4);
            FDT_OP(fdt_appendprop(fdt, pci_node, "interrupt-map", &irq_map, sizeof(irq_map)));
            is_irq_map = true;
        } else {
            /* We assume no empty gaps */
            break;
        }
    }
    if (is_irq_map) {
        struct pci_interrupt_map_mask irq_mask;
        irq_mask.pci_addr.hi = cpu_to_fdt32(0xf800);
        irq_mask.pci_addr.mid = 0;
        irq_mask.pci_addr.low = 0;
        irq_mask.irq_pin = cpu_to_fdt32(0x7);
        FDT_OP(fdt_appendprop(fdt, pci_node, "interrupt-map-mask", &irq_mask, sizeof(irq_mask)));
    }

    return 0;
}
