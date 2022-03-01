/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_vcpu_fault.h>
#include <sel4vm/guest_memory_helpers.h>

#include <sel4vmmplatsupport/guest_memory_util.h>
#include <sel4vmmplatsupport/device.h>
#include <sel4vmmplatsupport/device_utils.h>

int vm_install_ram_only_device(vm_t *vm, const struct device *device)
{
    struct device d;
    uintptr_t paddr;
    int err;
    d = *device;

    vm_memory_reservation_t *reservation = vm_reserve_memory_at(vm, d.pstart, d.size,
                                                                default_error_fault_callback, NULL);
    if (!reservation) {
        return -1;
    }

    err = map_frame_alloc_reservation(vm, reservation);
    assert(!err);
    return err;
}

static memory_fault_result_t passthrough_device_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                      size_t fault_length, void *cookie)
{
    ZF_LOGE("Fault occured on passthrough device");
    return FAULT_ERROR;
}

int vm_install_passthrough_device(vm_t *vm, const struct device *device)
{
    struct device d;
    uintptr_t paddr;
    int err;
    d = *device;
    for (paddr = d.pstart; paddr - d.pstart < d.size; paddr += 0x1000) {
        void *addr;
        vm_memory_reservation_t *reservation;
        reservation = vm_reserve_memory_at(vm, paddr, 0x1000, passthrough_device_fault, NULL);
        if (!reservation) {
            return -1;
        }
        err = map_ut_alloc_reservation(vm, reservation);
#ifdef PLAT_EXYNOS5
        if (err && paddr == MCT_ADDR) {
            printf("*****************************************\n");
            printf("*** Linux will try to use the MCT but ***\n");
            printf("*** the kernel is not exporting it!   ***\n");
            printf("*****************************************\n");
            /* VMCT is not fully functional yet */
//            err = vm_install_vmct(vm);
            return -1;
        }
#endif
        if (err) {
            return -1;
        }
    }
    return err;
}

static memory_fault_result_t handle_listening_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                    size_t fault_length, void *cookie)
{
    volatile uint32_t *reg;
    int offset;
    void **map;
    struct device *d = (struct device *)cookie;

    assert(d->priv);
    map = (void **)d->priv;
    offset = fault_addr - d->pstart;

    reg = (volatile uint32_t *)(map[offset >> 12] + (offset & MASK(12)));

    printf("[Listener/%s] ", d->name);
    if (is_vcpu_read_fault(vcpu)) {
        printf("read ");
        set_vcpu_fault_data(vcpu, *reg);
    } else {
        printf("write");
        *reg = emulate_vcpu_fault(vcpu, *reg);
    }
    printf(" ");
    seL4_Word data = get_vcpu_fault_data(vcpu);
    seL4_Word data_mask = get_vcpu_fault_data_mask(vcpu);
    printf("0x%"SEL4_PRIx_word, data & data_mask);
    printf(" address %p @ pc %p\n", (void *) fault_addr,
           (void *) get_vcpu_fault_ip(vcpu));
    advance_vcpu_fault(vcpu);
    return FAULT_HANDLED;
}


int vm_install_listening_device(vm_t *vm, const struct device *dev_listening)
{
    struct device *d;
    int pages;
    int i;
    void **map;
    int err;
    pages = dev_listening->size >> 12;
    d = (struct device *)calloc(1, sizeof(struct device));
    if (!d) {
        return -1;
    }
    memcpy(d, dev_listening, sizeof(struct device));
    /* Build device memory map */
    map = (void **)calloc(1, sizeof(void *) * pages);
    if (map == NULL) {
        return -1;
    }
    d->priv = map;
    for (i = 0; i < pages; i++) {
        map[i] = ps_io_map(&vm->io_ops->io_mapper, d->pstart + (i << 12), PAGE_SIZE_4K, 0, PS_MEM_NORMAL);
    }
    vm_memory_reservation_t *reservation = vm_reserve_memory_at(vm, d->pstart, d->size,
                                                                handle_listening_fault, (void *)d);
    if (!reservation) {
        free(d);
        free(map);
        return -1;
    }
    return 0;
}


static memory_fault_result_t handle_listening_ram_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                        size_t fault_length, void *cookie)
{
    volatile uint32_t *reg;
    int offset;

    struct device *d = (struct device *)cookie;
    assert(d->priv);
    offset = fault_addr - d->pstart;

    reg = (volatile uint32_t *)(d->priv + offset);

    if (is_vcpu_read_fault(vcpu)) {
        set_vcpu_fault_data(vcpu, *reg);
    } else {
        *reg = emulate_vcpu_fault(vcpu, *reg);
    }
    printf("Listener pc%p| %s%p:%p\n", (void *) get_vcpu_fault_ip(vcpu),
           is_vcpu_read_fault(vcpu) ? "r" : "w",
           (void *) fault_addr,
           (void *) get_vcpu_fault_data(vcpu));
    advance_vcpu_fault(vcpu);
    return FAULT_HANDLED;
}


const struct device dev_listening_ram = {
    .name = "<listing_ram>",
    .pstart = 0x0,
    .size = 0x1000,
    .priv = NULL
};


int vm_install_listening_ram(vm_t *vm, uintptr_t addr, size_t size)
{
    struct device *d;
    int err;

    d = (struct device *)calloc(1, sizeof(struct device));
    if (!d) {
        return -1;
    }
    memcpy(d, &dev_listening_ram, sizeof(struct device));

    d->pstart = addr;
    d->size = size;
    d->priv = calloc(1, 0x1000);
    assert(d->priv);
    if (!d->priv) {
        ZF_LOGE("calloc failed\n");
        return -1;
    }

    vm_memory_reservation_t *reservation = vm_reserve_memory_at(vm, d->pstart, d->size,
                                                                handle_listening_ram_fault, (void *)d);
    if (!reservation) {
        return -1;
    }
    return 0;
}
