/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <utils/util.h>
#include <sel4/sel4.h>
#include <vka/object.h>
#include <vka/capops.h>
#include <sel4utils/mapping.h>
#include <sel4utils/api.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_vm_exits.h>

#include <sel4vm/boot.h>
#include <sel4vm/guest_memory.h>
#include <sel4vm/guest_memory_helpers.h>

#include "vm_boot.h"
#include "guest_vspace.h"
#include "guest_memory.h"
#include "guest_state.h"
#include "vmcs.h"
#include "processor/decode.h"
#include "processor/apicdef.h"
#include "processor/lapic.h"
#include "processor/platfeature.h"

#define VM_VMCS_CR0_MASK           (X86_CR0_PG | X86_CR0_PE)
#define VM_VMCS_CR0_VALUE          VM_VMCS_CR0_MASK
/* We need to own the PSE and PAE bits up until the guest has actually turned on paging,
 * then it can control them
 */
#ifdef CONFIG_ARCH_X86_64
#define VM_VMCS_CR4_MASK           (X86_CR4_VMXE)
#define VM_VMCS_CR4_VALUE          (X86_CR4_PAE)
#else
#define VM_VMCS_CR4_MASK           (X86_CR4_PSE | X86_CR4_PAE | X86_CR4_VMXE)
#define VM_VMCS_CR4_VALUE          (X86_CR4_PSE | X86_CR4_VMXE)
#endif

#define PAGE_PRESENT    BIT(0)
#define PAGE_WRITE      BIT(1)
#define PAGE_SUPERVISOR BIT(2)
#define PAGE_SET_SIZE   BIT(7)

#define PAGE_DEFAULT   PAGE_PRESENT | PAGE_WRITE | PAGE_SUPERVISOR
#define PAGE_ENTRY     PAGE_DEFAULT | PAGE_SET_SIZE
#define PAGE_REFERENCE PAGE_DEFAULT

#define PAGE_MASK 0x7FFFFFFFFF000ULL

#define GUEST_VSPACE_ROOT     0x10000000
#define GUEST_VSPACE_PDPT     0x10001000
#define GUEST_VSPACE_PD       0x10002000

static vm_frame_t vspace_alloc_iterator(uintptr_t addr, void *cookie)
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
    ret = vka_alloc_frame_maybe_device(vm->vka, page_size, false, &object);
    if (ret) {
        ZF_LOGE("Failed to allocate frame for address 0x%x", (unsigned int)addr);
        return frame_result;
    }
    frame_result.cptr = object.cptr;
    frame_result.rights = seL4_AllRights;
    frame_result.vaddr = frame_start;
    frame_result.size_bits = page_size;
    return frame_result;
}

static int make_guest_pd_continued(void *access_addr, void *vaddr, void *cookie)
{
    uint64_t *pd = vaddr;
    int num_entries = BIT(seL4_PageBits) / sizeof(pd[0]);

    /* Brute force 1:1 entries. */
    for (int i = 0; i < num_entries; i++) {
        /* Present, write, user, page size 2M */
        pd[i] = ((uint64_t)i) << PAGE_BITS_2M | PAGE_ENTRY;
    }

    return 0;
}

static int make_guest_pdpt_continued(void *access_addr, void *vaddr, void *cookie)
{
    vm_t *vm = (vm_t *)cookie;

    vm_memory_reservation_t *pd_reservation = vm_reserve_memory_at(vm, GUEST_VSPACE_PD,
                                                                   BIT(seL4_PageBits),
                                                                   default_error_fault_callback,
                                                                   NULL);

    if (!pd_reservation) {
        ZF_LOGE("Failed to reserve page for initial guest PD");
        return -1;
    }
    int err = map_vm_memory_reservation(vm, pd_reservation, vspace_alloc_iterator, (void *)vm);
    if (err) {
        ZF_LOGE("Failed to map page for initial guest PD");
    }

    uint64_t *pdpt = vaddr;
    pdpt[0] = (GUEST_VSPACE_PD & PAGE_MASK) | PAGE_REFERENCE;

    return vspace_access_page_with_callback(&vm->mem.vm_vspace, &vm->mem.vmm_vspace,
                                            (void *)GUEST_VSPACE_PD,
                                            seL4_PageBits, seL4_AllRights, 1,
                                            make_guest_pd_continued, NULL);
}

static int make_guest_root_pd_continued(void *access_addr, void *vaddr, void *cookie)
{
#ifdef CONFIG_ARCH_X86_64
    assert(NULL != cookie);

    vm_t *vm = (vm_t *)cookie;

    vm_memory_reservation_t *pdpt_reservation = vm_reserve_memory_at(vm, GUEST_VSPACE_PDPT,
                                                                     BIT(seL4_PageBits),
                                                                     default_error_fault_callback,
                                                                     NULL);

    if (!pdpt_reservation) {
        ZF_LOGE("Failed to reserve page for initial guest PDPT");
        return -1;
    }
    int err = map_vm_memory_reservation(vm, pdpt_reservation, vspace_alloc_iterator, (void *)vm);
    if (err) {
        ZF_LOGE("Failed to map page for initial guest PDPT");
        return -1;
    }

    uint64_t *pml4 = vaddr;
    pml4[0] = (GUEST_VSPACE_PDPT & PAGE_MASK) | PAGE_REFERENCE;

    int error = vspace_access_page_with_callback(&vm->mem.vm_vspace, &vm->mem.vmm_vspace,
                                                 (void *)GUEST_VSPACE_PDPT,
                                                 seL4_PageBits, seL4_AllRights, 1,
                                                 make_guest_pdpt_continued, vm);
    if (error) {
        return error;
    }
#else
    /* Write into this frame as the init page directory: 4M pages, 1 to 1 mapping. */
    uint32_t *pd = vaddr;
    for (int i = 0; i < 1024; i++) {
        /* Present, write, user, page size 4M */
        pd[i] = (i << PAGE_BITS_4M) | PAGE_ENTRY;
    }
#endif
    return 0;
}

static int make_guest_address_space(vm_t *vm)
{
    /* Create a 4K Page to be our 1-1 vspace */
    /* This is constructed with magical new memory that we will not tell Linux about */
    vm_memory_reservation_t *vspace_reservation = vm_reserve_memory_at(vm, GUEST_VSPACE_ROOT,
                                                                       BIT(seL4_PageBits),
                                                                       default_error_fault_callback,
                                                                       NULL);
    if (!vspace_reservation) {
        ZF_LOGE("Failed to reserve page for initial guest vspace");
        return -1;
    }
    int err = map_vm_memory_reservation(vm, vspace_reservation, vspace_alloc_iterator, (void *)vm);
    if (err) {
        ZF_LOGE("Failed to map page for initial guest vspace");
        return -1;
    }
    printf("Guest address space root allocated at 0x%x. Creating 1-1 entries\n", (unsigned int)GUEST_VSPACE_ROOT);
    vm->arch.guest_pd = GUEST_VSPACE_ROOT;

    void *cookie = NULL;

#ifdef CONFIG_ARCH_X86_64
    cookie = (void *) vm;
#endif
    return vspace_access_page_with_callback(&vm->mem.vm_vspace, &vm->mem.vmm_vspace,
                                            (void *)GUEST_VSPACE_ROOT,
                                            seL4_PageBits, seL4_AllRights, 1,
                                            make_guest_root_pd_continued, cookie);
}

int vm_init_arch(vm_t *vm)
{
    int err;

    if (!vm) {
        ZF_LOGE("Failed to initialise vm arch: Invalid vm");
        return -1;
    }

    vm->arch.vmcall_handlers = NULL;
    vm->arch.vmcall_num_handlers = 0;
    vm->arch.ioport_list.num_ioports = 0;
    vm->arch.ioport_list.ioports = NULL;

    /* Create an EPT which is the pd for all the vcpu tcbs */
    err = vka_alloc_ept_pml4(vm->vka, &vm->mem.vm_vspace_root);
    if (err) {
        return -1;
    }
    /* Assign an ASID */
    err = simple_ASIDPool_assign(vm->simple, vm->mem.vm_vspace_root.cptr);
    if (err != seL4_NoError) {
        ZF_LOGE("Failed to assign ASID pool to EPT root");
        return -1;
    }
    /* Install the guest PD */
    err = seL4_TCB_SetEPTRoot(simple_get_tcb(vm->simple), vm->mem.vm_vspace_root.cptr);
    assert(err == seL4_NoError);
    /* Initialize a vspace for the guest */
    err = vm_init_guest_vspace(&vm->mem.vmm_vspace, &vm->mem.vmm_vspace,
                               &vm->mem.vm_vspace, vm->vka, vm->mem.vm_vspace_root.cptr);
    if (err) {
        return err;
    }

    /* Bind our interrupt pending callback */
    err = seL4_TCB_BindNotification(simple_get_init_cap(vm->simple, seL4_CapInitThreadTCB), vm->host_endpoint);
    assert(err == seL4_NoError);
    return err;
}

int vm_create_vcpu_arch(vm_t *vm, vm_vcpu_t *vcpu)
{
    int err;
    err = seL4_X86_VCPU_SetTCB(vcpu->vcpu.cptr, simple_get_tcb(vm->simple));
    assert(err == seL4_NoError);
    /* All LAPICs are created enabled, in virtual wire mode */
    vm_create_lapic(vcpu, 1);
    vcpu->vcpu_arch.guest_state = calloc(1, sizeof(guest_state_t));
    if (!vcpu->vcpu_arch.guest_state) {
        return -1;
    }

    /* Create the guest root vspace */
    err = make_guest_address_space(vm);
    if (err) {
        return -1;
    }

    vm_guest_state_initialise(vcpu->vcpu_arch.guest_state);
    /* Set the initial CR state */
    vcpu->vcpu_arch.guest_state->virt.cr.cr0_mask = VM_VMCS_CR0_MASK;
#ifdef CONFIG_X86_64_VTX_64BIT_GUESTS
    /* In 64-bit mode, PG and PE always need to be enabled, otherwise a fault will occur. */
    vcpu->vcpu_arch.guest_state->virt.cr.cr0_shadow = VM_VMCS_CR0_MASK;
#else
    vcpu->vcpu_arch.guest_state->virt.cr.cr0_shadow = 0;
#endif /* CONFIG_X86_64_VTX_64BIT_GUESTS */
    vcpu->vcpu_arch.guest_state->virt.cr.cr0_host_bits = VM_VMCS_CR0_VALUE;
    vcpu->vcpu_arch.guest_state->virt.cr.cr4_mask = VM_VMCS_CR4_MASK;
    vcpu->vcpu_arch.guest_state->virt.cr.cr4_shadow = 0;
    vcpu->vcpu_arch.guest_state->virt.cr.cr4_host_bits = VM_VMCS_CR4_VALUE;
    /* Set the initial CR states */
    vm_guest_state_set_cr0(vcpu->vcpu_arch.guest_state, vcpu->vcpu_arch.guest_state->virt.cr.cr0_host_bits);
    vm_guest_state_set_cr3(vcpu->vcpu_arch.guest_state, vm->arch.guest_pd);
    vm_guest_state_set_cr4(vcpu->vcpu_arch.guest_state, vcpu->vcpu_arch.guest_state->virt.cr.cr4_host_bits);
    /* Init guest OS vcpu state. */
    vm_vmcs_init_guest(vcpu);
    return 0;
}
