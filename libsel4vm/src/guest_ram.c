/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <autoconf.h>
#include <sel4vm/gen_config.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <sel4/sel4.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_ram.h>
#include <sel4vm/guest_memory.h>

#include "guest_memory.h"

struct guest_mem_touch_params {
    void *data;
    size_t size;
    size_t offset;
    uintptr_t current_addr;
    vm_t *vm;
    ram_touch_callback_fn touch_fn;
};

static int push_guest_ram_region(vm_mem_t *guest_memory, uintptr_t start, size_t size, int allocated)
{
    int last_region = guest_memory->num_ram_regions;
    if (size == 0) {
        return -1;
    }
    vm_ram_region_t *extended_regions = realloc(guest_memory->ram_regions, sizeof(vm_ram_region_t) * (last_region + 1));
    if (extended_regions == NULL) {
        return -1;
    }
    guest_memory->ram_regions = extended_regions;

    guest_memory->ram_regions[last_region].start = start;
    guest_memory->ram_regions[last_region].size = size;
    guest_memory->ram_regions[last_region].allocated = allocated;
    guest_memory->num_ram_regions++;
    return 0;
}

static int ram_region_cmp(const void *a, const void *b)
{
    const vm_ram_region_t *aa = a;
    const vm_ram_region_t *bb = b;
    return aa->start - bb->start;
}

static void sort_guest_ram_regions(vm_mem_t *guest_memory)
{
    qsort(guest_memory->ram_regions, guest_memory->num_ram_regions, sizeof(vm_ram_region_t), ram_region_cmp);
}

static void guest_ram_remove_region(vm_mem_t *guest_memory, int region)
{
    if (region >= guest_memory->num_ram_regions) {
        return;
    }
    guest_memory->num_ram_regions--;
    memmove(&guest_memory->ram_regions[region], &guest_memory->ram_regions[region + 1],
            sizeof(vm_ram_region_t) * (guest_memory->num_ram_regions - region));
    /* realloc it smaller */
    guest_memory->ram_regions = realloc(guest_memory->ram_regions, sizeof(vm_ram_region_t) * guest_memory->num_ram_regions);
}

static void collapse_guest_ram_regions(vm_mem_t *guest_memory)
{
    int i;
    for (i = 1; i < guest_memory->num_ram_regions;) {
        /* Only collapse regions with the same allocation flag that are contiguous */
        if (guest_memory->ram_regions[i - 1].allocated == guest_memory->ram_regions[i].allocated &&
            guest_memory->ram_regions[i - 1].start + guest_memory->ram_regions[i - 1].size == guest_memory->ram_regions[i].start) {

            guest_memory->ram_regions[i - 1].size += guest_memory->ram_regions[i].size;
            guest_ram_remove_region(guest_memory, i);
        } else {
            /* We are satisified that this entry cannot be merged. So now we
             * move onto the next one */
            i++;
        }
    }
}

static int expand_guest_ram_region(vm_t *vm, uintptr_t start, size_t bytes)
{
    int err;
    vm_mem_t *guest_memory = &vm->mem;
    /* blindly put a new region at the end */
    err = push_guest_ram_region(guest_memory, start, bytes, 0);
    if (err) {
        ZF_LOGE("Failed to expand guest ram region");
        return err;
    }
    /* sort the region we just added */
    sort_guest_ram_regions(guest_memory);
    /* collapse any contiguous regions */
    collapse_guest_ram_regions(guest_memory);
    return 0;
}

static bool is_ram_region(vm_t *vm, uintptr_t addr, size_t size)
{
    vm_mem_t *guest_memory = &vm->mem;
    for (int i = 0; i < guest_memory->num_ram_regions; i++) {
        if (guest_memory->ram_regions[i].start <= addr &&
            guest_memory->ram_regions[i].start + guest_memory->ram_regions[i].size >= addr + size) {
            /* We are within a ram region*/
            return true;
        }
    }
    return false;
}

static memory_fault_result_t default_ram_fault_callback(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t fault_addr,
                                                        size_t fault_length, void *cookie)
{
    /* We don't handle RAM faults by default unless the callback is specifically overrided, hence we fail here */
    ZF_LOGE("ERROR: UNHANDLED RAM FAULT");
    return FAULT_ERROR;
}

/* Helpers for use with touch below */
int vm_guest_ram_read_callback(vm_t *vm, uintptr_t addr, void *vaddr, size_t size, size_t offset, void *buf)
{
    memcpy(buf, vaddr, size);
    return 0;
}

int vm_guest_ram_write_callback(vm_t *vm, uintptr_t addr, void *vaddr, size_t size, size_t offset, void *buf)
{
    memcpy(vaddr, buf, size);
    return 0;
}

static int touch_access_callback(void *access_addr, void *vaddr, void *cookie)
{
    struct guest_mem_touch_params *guest_touch = (struct guest_mem_touch_params *)cookie;
    uintptr_t vmm_addr = (uintptr_t)vaddr;
    uintptr_t vm_addr = (uintptr_t)access_addr;
    return guest_touch->touch_fn(guest_touch->vm, vm_addr,
                                 (void *)(vmm_addr + (guest_touch->current_addr - vm_addr)),
                                 guest_touch->size, guest_touch->offset, guest_touch->data);
}

int vm_ram_touch(vm_t *vm, uintptr_t addr, size_t size, ram_touch_callback_fn touch_callback, void *cookie)
{
    struct guest_mem_touch_params access_cookie;
    uintptr_t current_addr;
    uintptr_t next_addr;
    uintptr_t end_addr = (uintptr_t)(addr + size);
    if (!is_ram_region(vm, addr, size)) {
        ZF_LOGE("Failed to touch ram region: Not registered RAM region");
        return -1;
    }
    access_cookie.touch_fn = touch_callback;
    access_cookie.data = cookie;
    access_cookie.vm = vm;
    for (current_addr = addr; current_addr < end_addr; current_addr = next_addr) {
	int sz = seL4_PageBits;

        uintptr_t current_aligned = PAGE_ALIGN_4K(current_addr);
        uintptr_t next_page_start = current_aligned + PAGE_SIZE_4K;

#ifdef CONFIG_LIB_VM_ALIGN_LP_HACK
        /* user-VM resides at 0x48000000 - 0x4fffffff */
        if (current_addr >= 0x48000000 && current_addr < 0x50000000) {
            sz = seL4_LargePageBits;
            current_aligned = PAGE_ALIGN_2M(current_addr);
            next_page_start = current_aligned + PAGE_SIZE_2M;
        }
#endif

        next_addr = MIN(end_addr, next_page_start);
        access_cookie.size = next_addr - current_addr;
        access_cookie.offset = current_addr - addr;
        access_cookie.current_addr = current_addr;
        int result = vspace_access_page_with_callback(&vm->mem.vm_vspace, &vm->mem.vmm_vspace, (void *)current_aligned,
                                                      sz, seL4_AllRights, 1, touch_access_callback, &access_cookie);
        if (result) {
            return result;
        }
    }
    return 0;
}

int vm_ram_find_largest_free_region(vm_t *vm, uintptr_t *addr, size_t *size)
{
    vm_mem_t *guest_memory = &vm->mem;
    int largest = -1;
    size_t largest_size = 0;

    /* find the largest unallocated region */
    for (int i = 0; i < guest_memory->num_ram_regions; i++) {
        if (!guest_memory->ram_regions[i].allocated &&
            guest_memory->ram_regions[i].size > largest_size) {
            largest = i;
            largest_size = guest_memory->ram_regions[i].size;
        }
    }

    if (largest == -1) {
        ZF_LOGE("Failed to find free region");
        return -1;
    }

    *addr = guest_memory->ram_regions[largest].start;
    *size = guest_memory->ram_regions[largest].size;
    return 0;
}

void vm_ram_mark_allocated(vm_t *vm, uintptr_t start, size_t bytes)
{
    vm_mem_t *guest_memory = &vm->mem;
    /* Find the region */
    int i;
    int region = -1;
    for (i = 0; i < guest_memory->num_ram_regions; i++) {
        if (guest_memory->ram_regions[i].start <= start &&
            guest_memory->ram_regions[i].start + guest_memory->ram_regions[i].size >= start + bytes) {
            region = i;
            break;
        }
    }
    if (region == -1 || guest_memory->ram_regions[region].allocated) {
        return;
    }
    /* Remove the region */
    vm_ram_region_t r = guest_memory->ram_regions[region];
    guest_ram_remove_region(guest_memory, region);
    /* Split the region into three pieces and add them */
    push_guest_ram_region(guest_memory, r.start, start - r.start, 0);
    push_guest_ram_region(guest_memory, start, bytes, 1);
    push_guest_ram_region(guest_memory, start + bytes, r.size - bytes - (start - r.start), 0);
    /* sort and collapse */
    sort_guest_ram_regions(guest_memory);
    collapse_guest_ram_regions(guest_memory);
}

uintptr_t vm_ram_allocate(vm_t *vm, size_t bytes)
{
    vm_mem_t *guest_memory = &vm->mem;
    for (int i = 0; i < guest_memory->num_ram_regions; i++) {
        if (!guest_memory->ram_regions[i].allocated && guest_memory->ram_regions[i].size >= bytes) {
            uintptr_t addr = guest_memory->ram_regions[i].start;
            vm_ram_mark_allocated(vm, addr, bytes);
            return addr;
        }
    }
    ZF_LOGE("Failed to allocate %zu bytes of guest RAM", bytes);
    return 0;
}

static vm_frame_t ram_alloc_iterator(uintptr_t addr, void *cookie)
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
        ZF_LOGE("Failed to allocate frame for address 0x%"PRIxPTR, addr);
        return frame_result;
    }
    frame_result.cptr = object.cptr;
    frame_result.rights = seL4_AllRights;
    frame_result.vaddr = frame_start;
    frame_result.size_bits = page_size;
    return frame_result;
}

static vm_frame_t ram_ut_alloc_iterator(uintptr_t addr, void *cookie)
{
    int ret;
    int error;
    vka_object_t object;
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };
    vm_t *vm = (vm_t *)cookie;
    if (!vm) {
        return frame_result;
    }
    int page_size = seL4_PageBits;
    uintptr_t frame_start = ROUND_DOWN(addr, BIT(page_size));
    cspacepath_t path;
    error = vka_cspace_alloc_path(vm->vka, &path);
    if (error) {
        ZF_LOGE("Failed to allocate path");
        return frame_result;
    }
    seL4_Word vka_cookie;
    error = vka_utspace_alloc_at(vm->vka, &path, kobject_get_type(KOBJECT_FRAME, page_size), page_size, frame_start,
                                 &vka_cookie);
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

static int map_ram_reservation(vm_t *vm, vm_memory_reservation_t *ram_reservation, bool untyped)
{
    int err;
    /* We map the reservation immediately, by-passing the deferred mapping functionality
     * This allows us the allocate, touch and manipulate VM RAM prior to the region needing to be
     * faulted upon first */
    if (untyped) {
        err = map_vm_memory_reservation(vm, ram_reservation, ram_ut_alloc_iterator, (void *)vm);
    } else {
        err = map_vm_memory_reservation(vm, ram_reservation, ram_alloc_iterator, (void *)vm);
    }
    if (err) {
        ZF_LOGE("Failed to map new ram reservation");
        return -1;
    }
    return 0;
}

uintptr_t vm_ram_register(vm_t *vm, size_t bytes)
{
    vm_memory_reservation_t *ram_reservation;
    int err;
    uintptr_t base_addr;

    ram_reservation = vm_reserve_anon_memory(vm, bytes, 0x1000, default_ram_fault_callback, NULL, &base_addr);
    if (!ram_reservation) {
        ZF_LOGE("Unable to reserve ram region of size 0x%zx", bytes);
        return 0;
    }
    err = map_ram_reservation(vm, ram_reservation, false);
    if (err) {
        vm_free_reserved_memory(vm, ram_reservation);
        return 0;
    }
    err = expand_guest_ram_region(vm, base_addr, bytes);
    if (err) {
        ZF_LOGE("Failed to register new ram region");
        vm_free_reserved_memory(vm, ram_reservation);
        return 0;
    }

    return base_addr;
}

int vm_ram_register_at(vm_t *vm, uintptr_t start, size_t bytes, bool untyped)
{
    vm_memory_reservation_t *ram_reservation;
    int err;

    ram_reservation = vm_reserve_memory_at(vm, start, bytes, default_ram_fault_callback,
                                           NULL);
    if (!ram_reservation) {
        ZF_LOGE("Unable to reserve ram region at addr 0x%"PRIxPTR" of size 0x%zx", start, bytes);
        return -1;
    }
    err = map_ram_reservation(vm, ram_reservation, untyped);
    if (err) {
        vm_free_reserved_memory(vm, ram_reservation);
        return -1;
    }
    err = expand_guest_ram_region(vm, start, bytes);
    if (err) {
        ZF_LOGE("Failed to register new ram region");
        vm_free_reserved_memory(vm, ram_reservation);
        return -1;
    }
    return 0;
}

int vm_ram_register_at_custom_iterator(vm_t *vm, uintptr_t start, size_t bytes, memory_map_iterator_fn map_iterator,
                                       void *cookie)
{
    vm_memory_reservation_t *ram_reservation;
    int err;

    ram_reservation = vm_reserve_memory_at(vm, start, bytes, default_ram_fault_callback,
                                           NULL);
    if (!ram_reservation) {
        ZF_LOGE("Unable to reserve ram region at addr 0x%"PRIxPTR" of size 0x%zx", start, bytes);
        return -1;
    }
    err = map_vm_memory_reservation(vm, ram_reservation, map_iterator, cookie);
    if (err) {
        ZF_LOGE("failed to map vm memory reservation to dataport\n");
        return -1;
    }
    err = expand_guest_ram_region(vm, start, bytes);
    if (err) {
        ZF_LOGE("Failed to register new ram region");
        vm_free_reserved_memory(vm, ram_reservation);
        return -1;
    }
    return 0;
}

void vm_ram_free(vm_t *vm, uintptr_t start, size_t bytes)
{
    return;
}
