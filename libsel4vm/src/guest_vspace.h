/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4/sel4.h>
#include <vspace/vspace.h>
#include <vka/vka.h>

#include <sel4utils/vspace.h>

#include <sel4vm/gen_config.h>

typedef struct guest_iospace {
    seL4_CPtr iospace;
    struct sel4utils_alloc_data iospace_vspace_data;
    vspace_t iospace_vspace;
} guest_iospace_t;

typedef struct guest_vspace {
    /* We abuse struct ordering and this member MUST be the first
     * thing in the struct */
    struct sel4utils_alloc_data vspace_data;
#ifdef CONFIG_LIB_SEL4VM_USE_TRANSLATION_VSPACE
    /* additional vspace to place all mappings into. We will maintain
     * a translation between this and the guest */
    vspace_t vmm_vspace;
    /* use a vspace implementation as a sparse data structure to track
     * the translation from guest to vmm */
    struct sel4utils_alloc_data translation_vspace_data;
    vspace_t translation_vspace;
#endif
    /* debug flag for checking if we add io spaces late */
    int done_mapping;
    int num_iospaces;
    guest_iospace_t **iospaces;
} guest_vspace_t;


/* Constructs a vspace that will duplicate mappings between a page directory and several IO spaces */
int vm_init_guest_vspace(vspace_t *loader, vspace_t *vmm, vspace_t *new_vspace, vka_t *vka, seL4_CPtr page_directory);
