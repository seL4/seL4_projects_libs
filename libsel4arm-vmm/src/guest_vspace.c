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
#include <autoconf.h>

#include "sel4arm-vmm/guest_vspace.h"
#include <sel4utils/vspace.h>
#include <sel4utils/vspace_internal.h>
#include <vka/capops.h>

#ifdef CONFIG_ARM_SMMU
typedef struct guest_iospace {
    seL4_CPtr iospace;
    struct sel4utils_alloc_data iospace_vspace_data;
    vspace_t iospace_vspace;
} guest_iospace_t;
#endif

typedef struct guest_vspace {
    /* We abuse struct ordering and this member MUST be the first
     * thing in the struct */
    struct sel4utils_alloc_data vspace_data;
#ifdef CONFIG_ARM_SMMU
    /* debug flag for checking if we add io spaces late */
    int done_mapping;
    int num_iospaces;
    guest_iospace_t *iospaces;
#endif
} guest_vspace_t;

static int guest_vspace_map(vspace_t *vspace, seL4_CPtr cap, void *vaddr, seL4_CapRights_t rights,
                            int cacheable, size_t size_bits)
{
    int error;
    /* perfrom the guest mapping */
    error = sel4utils_map_page_pd(vspace, cap, vaddr, rights, cacheable, size_bits);
    if (error) {
        return error;
    }
#ifdef CONFIG_ARM_SMMU
    /* If an object doesn't have Read or Write access, there isn't a reason to put
     * it in DMA space
     */
    if (rights.words[0] != seL4_NoRights.words[0]) {
        struct sel4utils_alloc_data *data = get_alloc_data(vspace);
        /* this type cast works because the alloc data was at the start of the struct
         * so it has the same address.
         * This conversion is guaranteed to work by the C standard */
        guest_vspace_t *guest_vspace = (guest_vspace_t*) data;
        /* set the mapping bit */
        guest_vspace->done_mapping = 1;
        cspacepath_t orig_path;
        vka_cspace_make_path(guest_vspace->vspace_data.vka, cap, &orig_path);
        /* map into all the io spaces */
        for (int i = 0; i < guest_vspace->num_iospaces; i++) {
            cspacepath_t new_path;
            error = vka_cspace_alloc_path(guest_vspace->vspace_data.vka, &new_path);
            if (error) {
                ZF_LOGE("Failed to allocate cslot to duplicate frame cap");
                return error;
            }
            error = vka_cnode_copy(&new_path, &orig_path, seL4_AllRights);

            guest_iospace_t *guest_iospace = &guest_vspace->iospaces[i];


            assert(error == seL4_NoError);
            error = sel4utils_map_iospace_page(guest_vspace->vspace_data.vka, guest_iospace->iospace,
                                               new_path.capPtr, (uintptr_t)vaddr, rights, 1,
                                               size_bits, NULL, NULL);
            if (error) {
                ZF_LOGE("Failed to map page into iospace %d", i);
                return error;
            }

            /* Store the slot of the frame cap copy in a vspace so they can be looked up and
             * freed when this address gets unmapped. */
            error = update_entries(&guest_iospace->iospace_vspace, (uintptr_t)vaddr, new_path.capPtr, size_bits, 0 /* cookie */);
            if (error) {
                ZF_LOGE("Failed to add iospace mapping information");
                return error;
            }
        }
    }
#endif
    return 0;
}

int vmm_get_guest_vspace(vspace_t *loader, vspace_t *new_vspace, vka_t *vka, seL4_CPtr pd)
{
    int error;
    guest_vspace_t *vspace = malloc(sizeof(*vspace));
    if (!vspace) {
        ZF_LOGE("Malloc failed");
        return -1;
    }
#ifdef CONFIG_ARM_SMMU
    vspace->done_mapping = 0;
    vspace->num_iospaces = 0;
    vspace->iospaces = malloc(0);
    assert(vspace->iospaces);
#endif
    error = sel4utils_get_vspace_with_map(loader, new_vspace, &vspace->vspace_data, vka, pd, NULL, NULL, guest_vspace_map);
    if (error) {
        ZF_LOGE("Failed to create guest vspace");
        return error;
    }
    new_vspace->unmap_pages = NULL;
    return 0;
}

#ifdef CONFIG_ARM_SMMU
int vmm_guest_vspace_add_iospace(vspace_t *loader, vspace_t *vspace, seL4_CPtr iospace)
{
    struct sel4utils_alloc_data *data = get_alloc_data(vspace);
    guest_vspace_t *guest_vspace = (guest_vspace_t *) data;

    assert(!guest_vspace->done_mapping);

    guest_vspace->iospaces = realloc(guest_vspace->iospaces, sizeof(guest_iospace_t) * (guest_vspace->num_iospaces + 1));
    assert(guest_vspace->iospaces);

    guest_iospace_t *guest_iospace = &guest_vspace->iospaces[guest_vspace->num_iospaces];
    guest_iospace->iospace = iospace;
    int error = sel4utils_get_vspace(loader, &guest_iospace->iospace_vspace, &guest_iospace->iospace_vspace_data,
                                     guest_vspace->vspace_data.vka, seL4_CapNull, NULL, NULL);
    if (error) {
        ZF_LOGE("Failed to allocate vspace for new iospace");
        return error;
    }

    guest_vspace->num_iospaces++;
    return 0;
}
#endif
