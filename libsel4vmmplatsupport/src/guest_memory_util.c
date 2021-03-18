/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sel4/sel4.h>
#include <vka/vka.h>
#include <vka/capops.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_memory.h>
#include <sel4vm/guest_memory_helpers.h>

#include <sel4vmmplatsupport/guest_memory_util.h>

struct device_frame_cookie {
    vka_object_t frame;
    cspacepath_t mapped_frame;
    vm_t *vm;
    uintptr_t addr;
    vm_memory_reservation_t *reservation;
    seL4_CapRights_t rights;
};

struct ut_alloc_iterator_cookie {
    vm_t *vm;
    vm_memory_reservation_t *reservation;
    uintptr_t paddr;
    bool with_paddr;
};

static vm_frame_t device_frame_iterator(uintptr_t addr, void *cookie)
{
    cspacepath_t return_frame;
    vm_t *vm;
    int page_size;
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };

    struct device_frame_cookie *device_cookie = (struct device_frame_cookie *)cookie;
    if (!device_cookie) {
        return frame_result;
    }
    vm = device_cookie->vm;
    page_size = seL4_PageBits;

    int ret = vka_cspace_alloc_path(vm->vka, &return_frame);
    if (ret) {
        ZF_LOGE("Failed to allocate cspace path from device frame");
        return frame_result;
    }
    ret = vka_cnode_copy(&return_frame, &device_cookie->mapped_frame, seL4_AllRights);
    if (ret) {
        ZF_LOGE("Failed to cnode_copy for device frame");
        vka_cspace_free_path(vm->vka, return_frame);
        return frame_result;
    }
    frame_result.cptr = return_frame.capPtr;
    frame_result.rights = device_cookie->rights;
    frame_result.vaddr = device_cookie->addr;
    frame_result.size_bits = page_size;

    return frame_result;
}

static vm_frame_t ut_alloc_iterator(uintptr_t addr, void *cookie)
{
    int ret;
    int error;
    vka_object_t object;
    uintptr_t alloc_addr;
    cspacepath_t path;
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };
    struct ut_alloc_iterator_cookie *alloc_cookie = (struct ut_alloc_iterator_cookie *)cookie;
    vm_t *vm = alloc_cookie->vm;
    int page_size = seL4_PageBits;
    uintptr_t frame_start = ROUND_DOWN(addr, BIT(page_size));

    if (alloc_cookie->with_paddr) {
        uintptr_t base_vaddr;
        size_t size;
        vm_get_reservation_memory_region(alloc_cookie->reservation, &base_vaddr, &size);
        alloc_addr = ROUND_DOWN(alloc_cookie->paddr + (addr - base_vaddr),  BIT(page_size));
    } else {
        alloc_addr = frame_start;
    }
    error = vka_cspace_alloc_path(vm->vka, &path);
    if (error) {
        ZF_LOGE("Failed to allocate path");
        return frame_result;
    }
    error = simple_get_frame_cap(vm->simple, (void *)alloc_addr, page_size, &path);
    if (error) {
        /* attempt to allocate */
        uintptr_t vka_cookie;
        error = vka_utspace_alloc_at(vm->vka, &path, kobject_get_type(KOBJECT_FRAME, page_size), page_size, alloc_addr,
                                     &vka_cookie);
    }
    if (error) {
        ZF_LOGE("Failed to allocate page");
        vka_cspace_free_path(vm->vka, path);
        return frame_result;
    }

    frame_result.cptr = path.capPtr;
    frame_result.rights = seL4_AllRights;
    frame_result.vaddr = frame_start;
    frame_result.size_bits = page_size;
    return frame_result;
}

static vm_frame_t maybe_device_alloc_iterator(uintptr_t addr, void *cookie)
{
    int ret;
    vka_object_t object;
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };
    vm_t *vm = (vm_t *)cookie;
    if (!vm) {
        return frame_result;
    }
    int page_size = seL4_PageBits;
    uintptr_t frame_start = ROUND_DOWN(addr, BIT(page_size));
    ret = vka_alloc_frame_maybe_device(vm->vka, page_size, true, &object);
    if (ret) {
        ZF_LOGE("Failed to allocate frame for address 0x%lx", addr);
        return frame_result;
    }
    frame_result.cptr = object.cptr;
    frame_result.rights = seL4_AllRights;
    frame_result.vaddr = frame_start;
    frame_result.size_bits = page_size;
    return frame_result;
}

static vm_frame_t frame_alloc_iterator(uintptr_t addr, void *cookie)
{
    int ret;
    vka_object_t object;
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };
    vm_t *vm = (vm_t *)cookie;
    if (!vm) {
        return frame_result;
    }
    int page_size = seL4_PageBits;
    uintptr_t frame_start = ROUND_DOWN(addr, BIT(page_size));
    ret = vka_alloc_frame(vm->vka, page_size, &object);
    if (ret) {
        ZF_LOGE("Failed to allocate frame for address 0x%lx", addr);
        return frame_result;
    }
    frame_result.cptr = object.cptr;
    frame_result.rights = seL4_AllRights;
    frame_result.vaddr = frame_start;
    frame_result.size_bits = page_size;
    return frame_result;
}

void *create_allocated_reservation_frame(vm_t *vm, uintptr_t addr, seL4_CapRights_t rights,
                                         memory_fault_callback_fn alloc_fault_callback, void *alloc_fault_cookie)
{
    int err;
    struct device_frame_cookie *cookie;
    int page_size = seL4_PageBits;
    void *alloc_addr;
    vspace_t *vmm_vspace = &vm->mem.vmm_vspace;
    ps_io_ops_t *ops = vm->io_ops;

    err = ps_calloc(&ops->malloc_ops, 1, sizeof(struct device_frame_cookie), (void **)&cookie);
    if (err) {
        ZF_LOGE("Failed to create allocated vm frame: Unable to allocate cookie");
        return NULL;
    }

    /* Reserve emulated vm frame */
    cookie->reservation = vm_reserve_memory_at(vm, addr, PAGE_SIZE_4K,
                                               alloc_fault_callback, (void *)alloc_fault_cookie);
    if (!cookie->reservation) {
        ZF_LOGE("Failed to create allocated vm frame: Unable to reservate emulated frame");
        ps_free(&ops->malloc_ops, sizeof(struct device_frame_cookie), (void **)&cookie);
        return NULL;
    }

    err = vka_alloc_frame(vm->vka, page_size, &cookie->frame);
    if (err) {
        ZF_LOGE("Failed vka_alloc_frame for allocated device frame");
        vm_free_reserved_memory(vm, cookie->reservation);
        ps_free(&ops->malloc_ops, sizeof(struct device_frame_cookie), (void **)&cookie);
        return NULL;
    }
    vka_cspace_make_path(vm->vka, cookie->frame.cptr, &cookie->mapped_frame);
    alloc_addr = vspace_map_pages(vmm_vspace, &cookie->mapped_frame.capPtr,
                                  NULL, seL4_AllRights, 1, page_size, 0);
    if (!alloc_addr) {
        ZF_LOGE("Failed to map allocated frame into vmm vspace");
        vka_free_object(vm->vka, &cookie->frame);
        vm_free_reserved_memory(vm, cookie->reservation);
        ps_free(&ops->malloc_ops, sizeof(struct device_frame_cookie), (void **)&cookie);
        return NULL;
    }

    cookie->vm = vm;
    cookie->addr = addr;
    cookie->rights = rights;

    /* Map the allocated frame */
    err = vm_map_reservation(vm, cookie->reservation, device_frame_iterator, (void *)cookie);
    if (err) {
        ZF_LOGE("Failed to map allocated frame into vm");
        vka_free_object(vm->vka, &cookie->frame);
        vm_free_reserved_memory(vm, cookie->reservation);
        ps_free(&ops->malloc_ops, sizeof(struct device_frame_cookie), (void **)&cookie);
        return NULL;
    }

    return alloc_addr;
}

void *create_device_reservation_frame(vm_t *vm, uintptr_t addr, seL4_CapRights_t rights,
                                      memory_fault_callback_fn fault_callback, void *fault_cookie)
{
    int err;
    struct device_frame_cookie *cookie;
    int page_size = seL4_PageBits;
    void *dev_addr;
    vspace_t *vmm_vspace = &vm->mem.vmm_vspace;
    ps_io_ops_t *ops = vm->io_ops;

    err = ps_calloc(&ops->malloc_ops, 1, sizeof(struct device_frame_cookie), (void **)&cookie);
    if (err) {
        ZF_LOGE("Failed to create device vm frame: Unable to allocate cookie");
        return NULL;
    }

    /* Reserve emulated vm frame */
    cookie->reservation = vm_reserve_memory_at(vm, addr, PAGE_SIZE_4K,
                                               fault_callback, (void *)fault_cookie);
    if (!cookie->reservation) {
        ZF_LOGE("Failed to create device vm frame: Unable to reservate emulated frame");
        ps_free(&ops->malloc_ops, sizeof(struct device_frame_cookie), (void **)&cookie);
        return NULL;
    }

    err = vka_cspace_alloc_path(vm->vka, &cookie->mapped_frame);
    if (err) {
        ZF_LOGE("Failed to create device vm frame: Unable to allocate cslot");
        vm_free_reserved_memory(vm, cookie->reservation);
        ps_free(&ops->malloc_ops, sizeof(struct device_frame_cookie), (void **)&cookie);
        return NULL;
    }

    err = simple_get_frame_cap(vm->simple, (void *)addr, page_size, &cookie->mapped_frame);
    if (err) {
        /* attempt to allocate */
        uintptr_t vka_cookie;
        err = vka_utspace_alloc_at(vm->vka, &cookie->mapped_frame, kobject_get_type(KOBJECT_FRAME, page_size), page_size, addr,
                                   &vka_cookie);
    }
    if (err) {
        ZF_LOGE("Failed to allocate page");
        vka_cspace_free_path(vm->vka, cookie->mapped_frame);
        vm_free_reserved_memory(vm, cookie->reservation);
        ps_free(&ops->malloc_ops, sizeof(struct device_frame_cookie), (void **)&cookie);
        return NULL;
    }
    dev_addr = vspace_map_pages(vmm_vspace, &cookie->mapped_frame.capPtr,
                                NULL, seL4_AllRights, 1, page_size, 0);
    if (!dev_addr) {
        ZF_LOGE("Failed to map device frame into vmm vspace");
        vka_cspace_free_path(vm->vka, cookie->mapped_frame);
        vm_free_reserved_memory(vm, cookie->reservation);
        ps_free(&ops->malloc_ops, sizeof(struct device_frame_cookie), (void **)&cookie);
        return NULL;
    }

    cookie->vm = vm;
    cookie->addr = addr;
    cookie->rights = rights;

    /* Map the emulated frame */
    err = vm_map_reservation(vm, cookie->reservation, device_frame_iterator, (void *)cookie);
    if (err) {
        ZF_LOGE("Failed to map device frame into vm");
        vspace_unmap_pages(vmm_vspace, dev_addr, 1, page_size, NULL);
        vka_cspace_free_path(vm->vka, cookie->mapped_frame);
        vm_free_reserved_memory(vm, cookie->reservation);
        ps_free(&ops->malloc_ops, sizeof(struct device_frame_cookie), (void **)&cookie);
        return NULL;
    }

    return dev_addr;
}

int map_ut_alloc_reservation(vm_t *vm, vm_memory_reservation_t *reservation)
{
    struct ut_alloc_iterator_cookie *cookie;
    ps_io_ops_t *ops = vm->io_ops;
    int err = ps_calloc(&ops->malloc_ops, 1, sizeof(struct ut_alloc_iterator_cookie), (void **)&cookie);
    if (err) {
        ZF_LOGE("Failed to map ut alloc reservation: Unable to allocate cookie");
        return -1;
    }
    cookie->vm = vm;
    cookie->with_paddr = false;
    cookie->reservation = reservation;
    return vm_map_reservation(vm, reservation, ut_alloc_iterator, (void *)cookie);
}

int map_ut_alloc_reservation_with_base_paddr(vm_t *vm, uintptr_t paddr, vm_memory_reservation_t *reservation)
{
    struct ut_alloc_iterator_cookie *cookie;
    ps_io_ops_t *ops = vm->io_ops;
    int err = ps_calloc(&ops->malloc_ops, 1, sizeof(struct ut_alloc_iterator_cookie), (void **)&cookie);
    if (err) {
        ZF_LOGE("Failed to map ut alloc reservation: Unable to allocate cookie");
        return -1;
    }
    cookie->vm = vm;
    cookie->paddr = paddr;
    cookie->with_paddr = true;
    cookie->reservation = reservation;
    return vm_map_reservation(vm, reservation, ut_alloc_iterator, (void *)cookie);
}

int map_frame_alloc_reservation(vm_t *vm, vm_memory_reservation_t *reservation)
{
    return vm_map_reservation(vm, reservation, frame_alloc_iterator, (void *)vm);
}

int map_maybe_device_reservation(vm_t *vm, vm_memory_reservation_t *reservation)
{
    return vm_map_reservation(vm, reservation, maybe_device_alloc_iterator, (void *)vm);
}
