/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdlib.h>
#include <sel4/sel4.h>
#include <vka/capops.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_memory.h>
#include <sel4vm/guest_memory_helpers.h>
#include <sel4vm/guest_irq_controller.h>
#include <sel4vm/guest_vcpu_fault.h>
#include <sel4vm/boot.h>

#include <sel4vmmplatsupport/device.h>
#include <sel4vmmplatsupport/guest_memory_util.h>
#include <sel4vmmplatsupport/drivers/virtio.h>
#include <sel4vmmplatsupport/drivers/pci_helper.h>
#include <sel4vmmplatsupport/drivers/cross_vm_connection.h>

#include <pci/helper.h>

#define MAX_NUM_CONNECTIONS 32

#define EVENT_BAR_EMIT_REGISTER 0x0
#define EVENT_BAR_EMIT_REGISTER_INDEX 0
#define EVENT_BAR_CONSUME_EVENT_REGISTER 0x4
#define EVENT_BAR_CONSUME_EVENT_REGISTER_INDEX 1
#define EVENT_BAR_DEVICE_NAME_REGISTER 0x8
#define EVENT_BAR_DEVICE_NAME_MAX_LEN 50

struct connection_info {
    uintptr_t event_address;
    void *event_registers;
    uintptr_t dataport_address;
    size_t dataport_size_bits;
    crossvm_handle_t connection;
    int connection_irq;
};

struct dataport_iterator_cookie {
    seL4_CPtr *dataport_frames;
    uintptr_t dataport_start;
    size_t dataport_size;
    vm_t *vm;
};

struct connection_info info[MAX_NUM_CONNECTIONS];
int total_connections;

static int construct_connection_bar(vm_t *vm, struct connection_info *info, int num_connections, vmm_pci_space_t *pci)
{
    for (int conn_idx = 0; conn_idx < num_connections; conn_idx++) {
        vmm_pci_device_def_t *pci_config;
        pci_config = calloc(1, sizeof(*pci_config));
        ZF_LOGF_IF(pci_config == NULL, "Failed to allocate pci config");
        *pci_config = (vmm_pci_device_def_t) {
            .vendor_id = 0x1af4,
            .device_id = 0xa111,
            .revision_id = 0x1,
            .subsystem_vendor_id = 0x0,
            .subsystem_id = 0x0,
            .interrupt_pin = 1,
            .interrupt_line = info[conn_idx].connection_irq,
            .bar0 = info[conn_idx].event_address | PCI_BASE_ADDRESS_SPACE_MEMORY,
            .bar1 = info[conn_idx].dataport_address | PCI_BASE_ADDRESS_SPACE_MEMORY,
            .command = PCI_COMMAND_MEMORY,
            .header_type = PCI_HEADER_TYPE_NORMAL,
            .subclass = (PCI_CLASS_MEMORY_RAM >> 8) & 0xff,
            .class_code = (PCI_CLASS_MEMORY_RAM >> 16) & 0xff,
        };

        vmm_pci_entry_t entry = (vmm_pci_entry_t) {
            .cookie = pci_config,
            .ioread = vmm_pci_mem_device_read,
            .iowrite = vmm_pci_mem_device_write
        };

        vmm_pci_bar_t bars[2] = {
            {
                .mem_type = PREFETCH_MEM,
                .address = info[conn_idx].event_address,
                /* if size_bits isn't the same for all resources then Linux tries
                   to remap the resources which the vmm driver doesn't support.
                 */
                .size_bits = info[conn_idx].dataport_size_bits
            },
            {
                .mem_type = PREFETCH_MEM,
                .address = info[conn_idx].dataport_address,
                .size_bits = info[conn_idx].dataport_size_bits
            }
        };
        vmm_pci_entry_t connection_pci_bar = vmm_pci_create_bar_emulation(entry, 2, bars);
        vmm_pci_add_entry(pci, connection_pci_bar, NULL);
    }
    return 0;
}

static memory_fault_result_t handle_event_bar_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                    size_t fault_length, void *cookie)
{
    struct device *d = (struct device *)cookie;
    struct connection_info *info = (struct connection_info *)d->priv;
    uint8_t reg = (fault_addr - d->pstart) & 0xff;

    if (is_vcpu_read_fault(vcpu)) {
        /* This shouldn't happen - the guest should have read writes to the event bar */
        ZF_LOGE("Error: Event Bar Memory is misconfigured");
    } else if (reg == EVENT_BAR_EMIT_REGISTER) {
        /* Emit operation: Write to register 0 */
        if (!info->connection.emit_fn) {
            ZF_LOGE("Connection is not configured with an emit function\n");
        } else {
            info->connection.emit_fn();
        }
    } else if (reg == EVENT_BAR_CONSUME_EVENT_REGISTER) {
        uint32_t mask = get_vcpu_fault_data_mask(vcpu);
        uint32_t value = get_vcpu_fault_data(vcpu) & mask;
        uint32_t *event_registers = (uint32_t *)info->event_registers;
        event_registers[EVENT_BAR_CONSUME_EVENT_REGISTER_INDEX] = value;
    } else {
        ZF_LOGE("Event Bar register unsupported");
    }
    advance_vcpu_fault(vcpu);
    return FAULT_HANDLED;
}

static int reserve_event_bar(vm_t *vm, uintptr_t event_bar_address, struct connection_info *info)
{
    struct device *event_bar = calloc(1, sizeof(struct device));
    if (!event_bar) {
        ZF_LOGE("Failed to create event bar device");
        return -1;
    }
    event_bar->pstart = event_bar_address;
    event_bar->priv = (void *)info;

    info->event_registers = create_allocated_reservation_frame(vm, event_bar_address, seL4_CanRead,
                                                               handle_event_bar_fault, event_bar);
    if (info->event_registers == NULL) {
        ZF_LOGE("Failed to map emulated event bar space");
        return -1;
    }
    /* Zero out memory */
    memset(info->event_registers, 0, PAGE_SIZE);
    info->event_address = event_bar_address;
    return 0;
}

static vm_frame_t dataport_memory_iterator(uintptr_t addr, void *cookie)
{
    int error;
    cspacepath_t return_frame;
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };
    struct dataport_iterator_cookie *dataport_cookie = (struct dataport_iterator_cookie *)cookie;
    seL4_CPtr *dataport_frames = dataport_cookie->dataport_frames;
    vm_t *vm = dataport_cookie->vm;
    uintptr_t dataport_start = dataport_cookie->dataport_start;
    size_t dataport_size = dataport_cookie->dataport_size;
    int page_size = seL4_LargePageBits;

    uintptr_t frame_start = ROUND_DOWN(addr, BIT(page_size));
    if (frame_start <  dataport_start ||
        frame_start > dataport_start + dataport_size) {
        ZF_LOGE("Error: Not Dataport region");
        return frame_result;
    }
    int page_idx = (frame_start - dataport_start) / BIT(page_size);
    frame_result.cptr = dataport_frames[page_idx];
    frame_result.rights = seL4_AllRights;
    frame_result.vaddr = frame_start;
    frame_result.size_bits = page_size;
    return frame_result;
}

static int reserve_dataport_memory(vm_t *vm, crossvm_dataport_handle_t *dataport, uintptr_t dataport_address,
                                   struct connection_info *info)
{
    int err;
    size_t size = dataport->size;
    unsigned int num_frames = dataport->num_frames;
    seL4_CPtr *frames = dataport->frames;

    vm_memory_reservation_t *dataport_reservation = vm_reserve_memory_at(vm, dataport_address, size,
                                                                         default_error_fault_callback,
                                                                         NULL);
    struct dataport_iterator_cookie *dataport_cookie = malloc(sizeof(struct dataport_iterator_cookie));
    if (!dataport_cookie) {
        ZF_LOGE("Failed to allocate dataport iterator cookie");
        return -1;
    }
    dataport_cookie->vm = vm;
    dataport_cookie->dataport_frames = frames;
    dataport_cookie->dataport_start = dataport_address;
    dataport_cookie->dataport_size = size;
    err = vm_map_reservation(vm, dataport_reservation, dataport_memory_iterator, (void *)dataport_cookie);
    if (err) {
        ZF_LOGE("Failed to map dataport memory");
        return -1;
    }
    info->dataport_address = dataport_address;
    info->dataport_size_bits = BYTES_TO_SIZE_BITS(size);
    return 0;
}

static void connection_consume_ack(vm_vcpu_t *vcpu, int irq, void *cookie) {}

void consume_connection_event(vm_t *vm, seL4_Word event_id, bool inject_irq)
{
    struct connection_info *conn_info = NULL;
    /* Search for a connection with a matching badge */
    for (int i = 0; i < total_connections; i++) {
        if (info[i].connection.consume_id == event_id) {
            conn_info = &info[i];
            break;
        }
    }
    if (conn_info == NULL) {
        /* No match */
        return;
    }
    /* We have an event - update the value in the event status register */
    uint32_t *event_registers = (uint32_t *)conn_info->event_registers;
    /* Increment the register to indicate a signal event occured -
     * we assume our kernel module will clear it as it consumes interrupt */
    event_registers[EVENT_BAR_CONSUME_EVENT_REGISTER_INDEX]++;
    if (inject_irq) {
        /* Inject our event interrupt */
        int err = vm_inject_irq(vm->vcpus[BOOT_VCPU], conn_info->connection_irq);
        if (err) {
            ZF_LOGE("Failed to inject connection irq");
        }
    }
    return;
}

static int register_consume_event(vm_t *vm, crossvm_handle_t *connection, struct connection_info *conn_info)
{
    if (connection->consume_id != -1 && conn_info->connection_irq > 0) {
        /* Register an irq for the crossvm connection */
        int err = vm_register_irq(vm->vcpus[BOOT_VCPU], conn_info->connection_irq, connection_consume_ack, NULL);
        if (err) {
            ZF_LOGE("Failed to register IRQ for event consume");
            return -1;
        }
    }
    return 0;
}

static int initialise_connections(vm_t *vm, uintptr_t connection_base_addr, crossvm_handle_t *connections,
                                  int num_connections, struct connection_info *info, int connection_irq)
{
    int err;
    uintptr_t connection_curr_addr = connection_base_addr;
    for (int i = 0; i < num_connections; i++) {
        /* We need to round everything up to the largest sized resource to prevent
         * Linux from remapping the devices, which the vpci device can't emulate.
         */
        crossvm_dataport_handle_t *dataport = connections[i].dataport;
        uintptr_t dataport_size = dataport->size;
        err = reserve_event_bar(vm, connection_curr_addr, &info[i]);
        if (err) {
            ZF_LOGE("Failed to create event bar (id:%d)", i);
            return -1;
        }
        connection_curr_addr += dataport_size;
        err = reserve_dataport_memory(vm, dataport, connection_curr_addr, &info[i]);
        if (err) {
            ZF_LOGE("Failed to create dataport bar (id %d)", i);
            return -1;
        }
        /* Register a callback event consuming (if we have one) */
        info[i].connection_irq = connection_irq;
        err = register_consume_event(vm, &connections[i], &info[i]);
        if (err) {
            ZF_LOGE("Failed to register connections consume event");
            return -1;
        }
        info[i].connection = connections[i];
        if (connections[i].connection_name == NULL) {
            connections[i].connection_name = "connector";
        }
        strncpy(info[i].event_registers + EVENT_BAR_DEVICE_NAME_REGISTER, connections[i].connection_name,
                EVENT_BAR_DEVICE_NAME_MAX_LEN);

        connection_curr_addr += dataport_size;
    }
    return 0;
}

int cross_vm_connections_init_common(vm_t *vm, uintptr_t connection_base_addr, crossvm_handle_t *connections,
                                     int num_connections, vmm_pci_space_t *pci, alloc_free_interrupt_fn alloc_irq)
{
    uintptr_t guest_paddr = 0;
    size_t guest_size = 0;
    if (num_connections > MAX_NUM_CONNECTIONS) {
        ZF_LOGE("Unable to register more than %d dataports", MAX_NUM_CONNECTIONS);
        return -1;
    }
    int connection_irq = alloc_irq();
    int err = initialise_connections(vm, connection_base_addr, connections, num_connections, info, connection_irq);
    if (err) {
        ZF_LOGE("Failed to reserve memory for dataports");
        return -1;
    }
    err = construct_connection_bar(vm, info, num_connections, pci);
    if (err) {
        ZF_LOGE("Failed to construct pci device for dataports");
        return -1;
    }
    total_connections = num_connections;
    return 0;
}
