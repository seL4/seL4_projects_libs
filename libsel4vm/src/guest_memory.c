/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <utils/sglib.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_memory.h>

#include "guest_memory.h"

typedef enum reservation_type {
    MEM_REGULAR_RES,
    MEM_ANON_RES
} reservation_type_t;

/* VM Memory reservation object: Represents a reservation in the guest VM's memory */
struct vm_memory_reservation {
    /* Base address of reserved memory region */
    uintptr_t addr;
    /* Size of memory region */
    size_t size;
    /* Callback to be invoked if memory region is faulted on*/
    memory_fault_callback_fn fault_callback;
    /* Iterator to be invoked for performing a map on the reservation region */
    memory_map_iterator_fn memory_map_iterator;
    /* If the reservation is pending to be mapped into the vm's address space */
    bool is_mapped;
    /* Cookies to pass onto callback and iterator functions */
    void *fault_callback_cookie;
    void *memory_iterator_cookie;
    /* The reservation in the vm's vspace object */
    reservation_t vspace_reservation;
    /* The type of reservation i.e regular, anonymous */
    reservation_type_t res_type;
};

typedef struct anon_region {
    uintptr_t addr;
    size_t size;
    uintptr_t alloc_addr;
    reservation_t vspace_reservation;
    int num_reservations;
    vm_memory_reservation_t **reservations;
} anon_region_t;

typedef struct res_tree {
    uintptr_t addr;
    size_t size;
    reservation_type_t res_type;
    void *data;
    char color_field;
    struct res_tree *left;
    struct res_tree *right;
} res_tree;

struct frames_map_iterator_cookie {
    seL4_CPtr *frames;
    size_t num_frames;
    size_t frame_size_bits;
    uintptr_t start;
};

static inline int reservation_node_cmp(res_tree *x, res_tree *y)
{
    if (x->addr < y->addr) {
        if (x->addr + x->size > y->addr) {
            /* The two regions intersect */
            return 0;
        } else {
            return -1;
        }
    }
    if (x->addr < y->addr + y->size) {
        /* The two regions intersect */
        return 0;
    }
    return 1;
}

SGLIB_DEFINE_RBTREE_PROTOTYPES(res_tree, left, right, color_field, reservation_node_cmp);
SGLIB_DEFINE_RBTREE_FUNCTIONS(res_tree, left, right, color_field, reservation_node_cmp);

struct vm_memory_reservation_cookie {
    struct res_tree *regular_res_tree;
    struct res_tree *anon_res_tree;
};

static res_tree *find_memory_reservation_by_addr(vm_t *vm, uintptr_t addr)
{
    res_tree *result_node;
    vm_memory_reservation_cookie_t *res_cookie = vm->mem.reservation_cookie;
    if (!res_cookie) {
        ZF_LOGE("Failed to find memory reservation: VM memory backend not initialised");
        return NULL;
    }
    res_tree search_node;
    search_node.addr = addr;
    search_node.size = 0;
    /* Search the regular reservation tree to begin with */
    result_node = sglib_res_tree_find_member(res_cookie->regular_res_tree, &search_node);
    if (result_node) {
        return result_node;
    }
    /* Search the anonymous reservation tree if nothing found */
    result_node = sglib_res_tree_find_member(res_cookie->anon_res_tree, &search_node);
    return result_node;
}

static void remove_memory_reservation_node(vm_t *vm,  uintptr_t addr, size_t size, reservation_type_t res_type)
{
    res_tree *tree;
    res_tree *result_node;
    vm_memory_reservation_cookie_t *res_cookie = vm->mem.reservation_cookie;
    if (!res_cookie) {
        ZF_LOGE("Failed to find memory reservation: VM memory backend not initialised");
        return;
    }
    if (res_type == MEM_REGULAR_RES) {
        tree = res_cookie->regular_res_tree;
    } else {
        tree = res_cookie->anon_res_tree;
    }
    res_tree search_node;
    search_node.addr = addr;
    search_node.size = size;
    result_node = sglib_res_tree_find_member(tree, &search_node);
    if (!result_node) {
        /* No node found */
        return;
    }
    /* Region needs to be an exact match */
    if ((result_node->addr == search_node.addr) && (result_node->size == search_node.size)) {
        sglib_res_tree_delete(&tree, result_node);
        if (res_type == MEM_REGULAR_RES) {
            res_cookie->regular_res_tree = tree;
        } else {
            res_cookie->anon_res_tree = tree;
        }
    }
    return;
}

static int add_memory_reservation_node(vm_t *vm, uintptr_t addr, size_t size, reservation_type_t res_type, void *data)
{
    int err;
    res_tree *tree;
    res_tree *result_node;
    ps_io_ops_t *ops = vm->io_ops;
    vm_memory_reservation_cookie_t *res_cookie = vm->mem.reservation_cookie;
    if (!res_cookie) {
        ZF_LOGE("Failed to find memory reservation: VM memory backend not initialised");
        return -1;
    }
    if (res_type == MEM_REGULAR_RES) {
        tree = res_cookie->regular_res_tree;
    } else {
        tree = res_cookie->anon_res_tree;
    }
    res_tree search_node;
    search_node.addr = addr;
    search_node.size = size;

    res_tree *found_node = sglib_res_tree_find_member(tree, &search_node);
    if (found_node == NULL) {
        err = ps_calloc(&ops->malloc_ops, 1, sizeof(res_tree), (void **)&result_node);
        if (err) {
            ZF_LOGE("Failed to add memory reservation: Unable to allocate new reservation node");
            return -1;
        }
        result_node->addr = addr;
        result_node->size = size;
        result_node->res_type = res_type;
        result_node->data = data;
        sglib_res_tree_add(&tree, result_node);
        if (res_type == MEM_REGULAR_RES) {
            res_cookie->regular_res_tree = tree;
        } else {
            res_cookie->anon_res_tree = tree;
        }
    } else {
        ZF_LOGE("Failed to add memory reservation: Reservation region already exists");
        return -1;
    }
    return 0;
}

static anon_region_t *find_allocable_anon_region(vm_t *vm, size_t size, size_t align)
{
    res_tree *anon_tree;
    res_tree *anon_node;
    anon_region_t *ret_region = NULL;
    struct sglib_res_tree_iterator it;
    vm_memory_reservation_cookie_t *res_cookie = vm->mem.reservation_cookie;
    if (!res_cookie) {
        return NULL;
    }
    anon_tree = res_cookie->anon_res_tree;
    for (anon_node = sglib_res_tree_it_init_inorder(&it, anon_tree); anon_node != NULL;
         anon_node = sglib_res_tree_it_next(&it)) {
        anon_region_t *curr_region = (anon_region_t *)anon_node->data;
        uintptr_t allocable_addr = ROUND_UP(curr_region->alloc_addr, (uintptr_t) align);
        size_t free_area_size = curr_region->size - (allocable_addr - curr_region->addr);
        if (size <= free_area_size) {
            ret_region = curr_region;
            break;
        }
    }
    return ret_region;
}

static void free_vm_reservation(vm_t *vm, vm_memory_reservation_t *reservation)
{
    if (!vm || !reservation) {
        return;
    }
    ps_io_ops_t *ops = vm->io_ops;
    ps_free(&ops->malloc_ops, sizeof(vm_memory_reservation_t), reservation);
}

static vm_memory_reservation_t *allocate_vm_reservation(vm_t *vm, uintptr_t addr, size_t size,
                                                        reservation_t vspace_reservation)
{
    int err;
    ps_io_ops_t *ops = vm->io_ops;
    vm_memory_reservation_t *new_reservation;
    err = ps_calloc(&ops->malloc_ops, 1, sizeof(vm_memory_reservation_t), (void **)&new_reservation);
    if (err) {
        ZF_LOGE("Failed to allocate vm reservation: Unable to allocate new vm memory reservation");
        return NULL;
    }

    new_reservation->addr = addr;
    new_reservation->size = size;
    new_reservation->is_mapped = false;
    new_reservation->vspace_reservation = vspace_reservation;
    return new_reservation;
}

static vm_memory_reservation_t *find_anon_reservation_by_addr(uintptr_t addr, size_t size,
                                                              anon_region_t *anon_region)
{
    int num_anon_reservations;
    vm_memory_reservation_t **reservations;

    if (!anon_region) {
        ZF_LOGE("Failed to find anonymous reservation: anon region NULL");
        return NULL;
    }
    num_anon_reservations = anon_region->num_reservations;
    reservations = anon_region->reservations;

    if (!reservations) {
        ZF_LOGE("Failed to find anonymous reservation: anon region has no reservations");
        return NULL;
    }

    for (int i = 0; i < num_anon_reservations; i++) {
        vm_memory_reservation_t *curr_res = reservations[i];
        if (curr_res->addr <= addr && curr_res->addr + curr_res->size >= addr + size) {
            return curr_res;
        }
    }

    return NULL;
}

memory_fault_result_t vm_memory_handle_fault(vm_t *vm, vm_vcpu_t *vcpu, uintptr_t addr, size_t size)
{
    int err;
    res_tree *reservation_node = find_memory_reservation_by_addr(vm, addr);
    vm_memory_reservation_t *fault_reservation;

    if (!reservation_node) {
        ZF_LOGW("Unable to find reservation for addr: 0x%x, memory fault left unhandled", addr);
        return FAULT_UNHANDLED;
    }

    if ((reservation_node->addr + size) > (reservation_node->addr + reservation_node->size)) {
        ZF_LOGE("Failed to handle memory fault: Invalid fault region");
        return FAULT_ERROR;
    }

    if (reservation_node->res_type == MEM_REGULAR_RES) {
        fault_reservation = (vm_memory_reservation_t *)reservation_node->data;
    } else {
        fault_reservation = find_anon_reservation_by_addr(addr, size,
                                                          (anon_region_t *)reservation_node->data);
        if (!fault_reservation) {
            ZF_LOGW("Unable to find anoymous reservation for addr: 0x%x, memory fault left unhandled", addr);
            return FAULT_UNHANDLED;
        }
    }

    if (!fault_reservation->is_mapped && fault_reservation->memory_map_iterator) {
        /* Deferred mapping */
        err = map_vm_memory_reservation(vm, fault_reservation,
                                        fault_reservation->memory_map_iterator, fault_reservation->memory_iterator_cookie);
        if (err) {
            ZF_LOGE("Unable to handle memory fault: Failed to map memory");
            return FAULT_ERROR;
        }
        return FAULT_RESTART;
    }

    if (!fault_reservation->fault_callback) {
        return FAULT_ERROR;
    }

    return fault_reservation->fault_callback(vm, vcpu, addr, size, fault_reservation->fault_callback_cookie);
}

vm_memory_reservation_t *vm_reserve_memory_at(vm_t *vm, uintptr_t addr, size_t size,
                                              memory_fault_callback_fn fault_callback, void *cookie)
{
    int err;
    vm_memory_reservation_t *new_reservation;

    if (!fault_callback) {
        ZF_LOGE("Failed to reserve vm reservation: NULL fault callback");
        return NULL;
    }

    /* A placeholder reservation to ensure nothing else takes the range
     * We will update the reservation with the correct rights on mapping */
    reservation_t vspace_reservation = vspace_reserve_deferred_rights_range_at(&vm->mem.vm_vspace, (void *)addr,
                                                                               size, 1);
    if (!vspace_reservation.res) {
        ZF_LOGE("Failed to allocate vm reservation: Unable to create vspace reservation at address 0x%"PRIxPTR" of size %zu",
                addr, size);
        return NULL;
    }
    new_reservation = allocate_vm_reservation(vm, addr, size, vspace_reservation);
    if (!new_reservation) {
        ZF_LOGE("Failed to reserve vm memory: Unable to allocate new vm reservation");
        vspace_free_reservation(&vm->mem.vm_vspace, vspace_reservation);
        return NULL;
    }
    new_reservation->fault_callback = fault_callback;
    new_reservation->fault_callback_cookie = cookie;
    new_reservation->res_type = MEM_REGULAR_RES;

    err = add_memory_reservation_node(vm, addr, size, MEM_REGULAR_RES, (void *)new_reservation);
    if (err) {
        ZF_LOGE("Failed to reserve vm memory: Unable to add vm memory reservation to list");
        vspace_free_reservation(&vm->mem.vm_vspace, vspace_reservation);
        free_vm_reservation(vm, new_reservation);
        return NULL;
    }
    return new_reservation;
}

int vm_memory_make_anon(vm_t *vm, uintptr_t addr, size_t size)
{
    int err;
    reservation_t vspace_reservation = vspace_reserve_deferred_rights_range_at(&vm->mem.vm_vspace, (void *)addr,
                                                                               size, 1);
    if (!vspace_reservation.res) {
        ZF_LOGE("Failed to make anonymous memory region: Unable to create vspace reservation of size %zu", size);
        return -1;
    }

    anon_region_t *region_data;
    ps_io_ops_t *ops = vm->io_ops;
    err = ps_calloc(&ops->malloc_ops, 1, sizeof(anon_region_t), (void **)&region_data);
    if (err) {
        ZF_LOGE("Failed to make anonymous memory region : Unable to allocate anonymous region");
        return -1;
    }
    region_data->addr = addr;
    region_data->size = size;
    region_data->alloc_addr = addr;
    region_data->vspace_reservation = vspace_reservation;
    region_data->num_reservations = 0;

    err = add_memory_reservation_node(vm, addr, size, MEM_ANON_RES, (void *)region_data);
    if (err) {
        ZF_LOGE("Failed to reserve vm memory: Unable to add vm memory reservation to list");
        vspace_free_reservation(&vm->mem.vm_vspace, vspace_reservation);
        ps_free(&ops->malloc_ops, sizeof(anon_region_t), region_data);
        return -1;
    }
    return 0;
}

vm_memory_reservation_t *vm_reserve_anon_memory(vm_t *vm, size_t size, size_t align,
                                                memory_fault_callback_fn fault_callback, void *cookie, uintptr_t *addr)
{
    int err;
    vm_memory_reservation_t *new_reservation;
    anon_region_t *allocable_region;
    uintptr_t reservation_addr;

    if (!fault_callback) {
        ZF_LOGE("Failed to reserve anon memory region: NULL fault callback");
        return NULL;
    }

    allocable_region = find_allocable_anon_region(vm, ROUND_UP(size, BIT(seL4_PageBits)), align);
    if (!allocable_region) {
        ZF_LOGE("Failed to reserve anon memory: No anonymous memory available to cater reservation size");
        return NULL;
    }

    reservation_addr = ROUND_UP(allocable_region->alloc_addr, (uintptr_t) align);

    /* Make a sub-reservation token. */
    new_reservation = allocate_vm_reservation(vm, reservation_addr, size, allocable_region->vspace_reservation);
    if (!new_reservation) {
        ZF_LOGE("Failed to reserve vm memory: Unable to allocate new vm reservation");
        return NULL;
    }
    new_reservation->fault_callback = fault_callback;
    new_reservation->fault_callback_cookie = cookie;
    new_reservation->res_type = MEM_ANON_RES;

    /* Register the sub-reservation token into the region - It will need to iterate over it on faults */
    vm_memory_reservation_t **extended_reservations = realloc(allocable_region->reservations,
                                                              sizeof(vm_memory_reservation_t *) * (allocable_region->num_reservations + 1));
    if (!extended_reservations) {
        free_vm_reservation(vm, new_reservation);
        return NULL;
    }
    allocable_region->reservations = extended_reservations;

    allocable_region->reservations[allocable_region->num_reservations] = new_reservation;
    allocable_region->alloc_addr = reservation_addr + ROUND_UP(size, BIT(seL4_PageBits));
    allocable_region->num_reservations += 1;

    *addr = reservation_addr;
    return new_reservation;
}

int vm_free_reserved_memory(vm_t *vm, vm_memory_reservation_t *reservation)
{
    ps_io_ops_t *ops = vm->io_ops;
    if (!reservation) {
        ZF_LOGE("Failed to free reserved memory: Invalid reservation");
        return -1;
    }

    if (reservation->res_type == MEM_ANON_RES) {
        /* We current don't support free'ing anonymous reservations */
        /* TODO: Support freeing anonymous memory reservations */
        ZF_LOGE("Failed to free reserved memory: Cannot free anonymous memory reservations");
        return -1;
    }

    remove_memory_reservation_node(vm, reservation->addr, reservation->size, reservation->res_type);
    if (reservation->is_mapped) {
        int page_size = seL4_PageBits;
        int num_pages = ROUND_UP(reservation->size, BIT(page_size)) >> page_size;
        vspace_unmap_pages(&vm->mem.vm_vspace, (void *)reservation->addr, num_pages, page_size, vm->vka);
    }
    vspace_free_reservation(&vm->mem.vm_vspace, reservation->vspace_reservation);
    free_vm_reservation(vm, reservation);
    return 0;
}

int map_vm_memory_reservation(vm_t *vm, vm_memory_reservation_t *vm_reservation,
                              memory_map_iterator_fn map_iterator, void *map_cookie)
{
    int err;
    uintptr_t reservation_addr = vm_reservation->addr;
    size_t reservation_size = vm_reservation->size;
    uintptr_t current_addr = vm_reservation->addr;

    while (current_addr < reservation_addr + reservation_size) {
        vm_frame_t reservation_frame = map_iterator(current_addr, map_cookie);
        if (reservation_frame.cptr == seL4_CapNull) {
            ZF_LOGE("Failed to get frame for reservation address 0x%lx", current_addr);
            break;
        }
        int ret = vspace_deferred_rights_map_pages_at_vaddr(&vm->mem.vm_vspace, &reservation_frame.cptr, NULL,
                                                            (void *)reservation_frame.vaddr, 1, reservation_frame.size_bits,
                                                            reservation_frame.rights, vm_reservation->vspace_reservation);
        if (ret) {
            ZF_LOGE("Failed to map address 0x%"PRIxPTR" into guest vm vspace", reservation_frame.vaddr);
            return -1;
        }
        current_addr += BIT(reservation_frame.size_bits);
    }
    vm_reservation->memory_map_iterator = NULL;
    vm_reservation->memory_iterator_cookie = NULL;
    vm_reservation->is_mapped = true;
    return 0;
}

int vm_map_reservation(vm_t *vm, vm_memory_reservation_t *reservation,
                       memory_map_iterator_fn map_iterator, void *cookie)
{
    int err;
    if (!vm) {
        ZF_LOGE("Failed to map vm reservation: Invalid NULL VM handle given");
        return -1;
    } else if (!reservation) {
        ZF_LOGE("Failed to map vm reservation: Invalid NULL reservation given");
        return -1;
    } else if (!map_iterator) {
        ZF_LOGE("Failed to map vm reservation: Invalid map iterator given");
        return -1;
    }

    reservation->memory_map_iterator = map_iterator;
    reservation->memory_iterator_cookie = cookie;
    if (!config_set(CONFIG_LIB_SEL4VM_DEFER_MEMORY_MAP)) {
        err = map_vm_memory_reservation(vm, reservation, map_iterator, cookie);
        /* We remove the iterator after attempting the mapping (regardless of success or fail)
         * If failed its left to the caller to update the memory map iterator */
        if (err) {
            ZF_LOGE("Failed to map vm reservation: Error when mapping into VM's vspace");
            return -1;
        }
    }

    return 0;
}

static vm_frame_t frames_map_memory_iterator(uintptr_t page_start, void *cookie)
{
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };
    struct frames_map_iterator_cookie *it = cookie;

    size_t frame_idx = (page_start - it->start) >> it->frame_size_bits;
    if (frame_idx >= it->num_frames) {
        return frame_result;
    }

    frame_result.cptr = it->frames[frame_idx];
    frame_result.rights = seL4_AllRights;
    frame_result.vaddr = page_start;
    frame_result.size_bits = it->frame_size_bits;

    return frame_result;
}

int vm_map_reservation_frames(vm_t *vm, vm_memory_reservation_t *reservation,
                              seL4_CPtr *frames, size_t num_frames,
                              size_t frame_size_bits)
{
    struct frames_map_iterator_cookie cookie = {
        .frames = frames,
        .num_frames = num_frames,
        .frame_size_bits = frame_size_bits,
        .start = reservation->addr,
    };

    return vm_map_reservation(vm, reservation, frames_map_memory_iterator,
                              &cookie);
}

void vm_get_reservation_memory_region(vm_memory_reservation_t *reservation, uintptr_t *addr, size_t *size)
{
    *addr = reservation->addr;
    *size = reservation->size;
}

int vm_memory_init(vm_t *vm)
{
    ps_io_ops_t *ops = vm->io_ops;
    vm_memory_reservation_cookie_t *cookie;
    int err = ps_calloc(&ops->malloc_ops, 1, sizeof(vm_memory_reservation_cookie_t), (void **)&cookie);
    if (err) {
        ZF_LOGE("Failed to initialise vm memory backend: Unable to allocate vm memory reservation cookie");
        return -1;
    }
    vm->mem.reservation_cookie = cookie;
    return 0;
}
