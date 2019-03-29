/*
 * Copyright 2017, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(DATA61_BSD)
 */
#pragma once

#include <autoconf.h>
#include <sel4arm-vmm/gen_config.h>
#include <sel4arm-vmm/devices.h>

#include <sel4arm-vmm/plat/devices.h>

/**
 * Map a device
 * @param[in] vspace  The vspace to map the device to
 * @param[in] vka     Allocator to use for kernel object creation
 * @param[in] simple  Used to retrieve device capabilities
 * @param[in] pa      The physiacl address of the device
 * @param[in] va      The virtual address to map to, or 0 to let the system choose
 * @param[in] rights  Rights to use for the mapping
 * @return            The virtual address of the mapping
 */
void *map_device(vspace_t *vspace, vka_t *vka, simple_t *simple,
                 uintptr_t pa, uintptr_t va, seL4_CapRights_t rights);

/**
 * Map a device into a VM
 * @param[in] vm      The VM to map the device to
 * @param[in] pa      The physiacl address of the device
 * @param[in] va      The virtual address to map to, or 0 to let the system choose
 * @param[in] rights  Rights to use for the mapping
 * @return            The virtual address of the mapping
 */
void *map_vm_device(vm_t *vm, uintptr_t pa, uintptr_t va, seL4_CapRights_t rights);

/**
 * Map RAM with full access rights
 * @param[in] vspace  The vspace to map the RAM to
 * @param[in] vmm_vspace  The vmm vspace to temporarily map the RAM into to zero memory.
 * @param[in] vka     Allocator to use for kernel object creation
 * @param[in] va      The VA (or IPA) to map to, or 0 to let the system choose
 * @return            The virtual address of the mapping
 */
void *map_ram(vspace_t *vspace, vspace_t *vmm_vspace, vka_t *vka, uintptr_t vaddr);

/**
 * Map RAM with full access rights into a VM
 * @param[in] vm      The VM to map the device to
 * @param[in] va      The VA (or IPA) to map to, or 0 to let the system choose
 * @return            The virtual address of the mapping
 */
void *map_vm_ram(vm_t *vm, uintptr_t va);

/**
 * Map a shared page between VM and VMM. The page will not be mapped cacheable
 * to force coherency with VM policy.
 * @param[in] vm     The VM to map the page into
 * @param[in] ipa    The VM IPA that the page should be mapped to
 * @param[in] rights The Access permissions of the mapping for the VM
 * @return           The VMM virtual address off the mapping
 */
void *map_shared_page(vm_t *vm, uintptr_t ipa, seL4_CapRights_t rights);

/**
 * Map a device ready for emulation
 * The device frame will not actually be mapped, read only RAM will be mapped
 * instead. Additionally, the frame will also be mapped into the VMM with
 * full access rights. In this way, the VM can read "device" data without
 * faulting while the VM can trap and arbitrate writes. This method is not
 * suitable for any device that has side effects from a read (e.g. FIFO devices).
 * @param[in] vm   The VM to map to
 * @param[in] d    A description of the device.
 * @return         The virtual address of the mapping to the VMM
 */
void *map_emulated_device(vm_t *vm, const struct device *d);


/**
 * Check if a physical address should be handled by the provided device
 * @param[in] addr The physical address in question
 * @param[in] d    A description of the device to check
 * @return         non-zero if the address belongs to the device
 */
static inline int dev_paddr_in_range(seL4_Word addr, const struct device* d)
{
    return ((addr >= d->pstart) && addr < (d->pstart + d->size));
}
