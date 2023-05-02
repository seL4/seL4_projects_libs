/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

/* Routines for generating guest ACPI tables.
Author: W.A. */

#include <stdlib.h>
#include <string.h>

#include <vka/capops.h>
#include <vka/object.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_ram.h>
#include <sel4vm/guest_memory.h>
#include <sel4vm/guest_memory_helpers.h>
#include <platsupport/plat/acpi/acpi.h>

#include <sel4vmmplatsupport/guest_memory_util.h>
#include <sel4vmmplatsupport/arch/acpi.h>

#define APIC_FLAGS_ENABLED (1)

uint8_t acpi_calc_checksum(const char *table, int length)
{
    uint32_t sum = 0;

    for (int i = 0; i < length; i++) {
        sum += *table++;
    }

    return 0x100 - (sum & 0xFF);
}

static void acpi_fill_table_head(acpi_header_t *head, const char *signature, uint8_t rev)
{
    const char *oem = "NICTA ";
    const char *padd = "    ";
    memcpy(head->signature, signature, sizeof(head->signature));
    memcpy(head->oem_id, oem, sizeof(head->oem_id));
    memcpy(head->creator_id, oem, sizeof(head->creator_id));
    memcpy(head->oem_table_id, signature, sizeof(head->signature));
    memcpy(head->oem_table_id + sizeof(head->signature), padd, 4);

    head->revision = rev;
    head->checksum = 0;
    head->length = sizeof(*head);
    head->oem_revision = rev;
    head->creator_revision = 1;
}

static int make_guest_acpi_tables_continued(vm_t *vm, uintptr_t paddr, void *vaddr,
                                            size_t size, size_t offset, void *cookie)
{
    (void)offset;
    memcpy((char *)vaddr, (char *)cookie, size);
    return 0;
}

struct bios_iterator_cookie {
    cspacepath_t *bios_frames;
    vm_t *vm;
};

static vm_frame_t bios_memory_iterator(uintptr_t addr, void *cookie)
{
    int error;
    cspacepath_t return_frame;
    vm_frame_t frame_result = { seL4_CapNull, seL4_NoRights, 0, 0 };
    struct bios_iterator_cookie *bios_cookie = (struct bios_iterator_cookie *)cookie;
    cspacepath_t *bios_frames = bios_cookie->bios_frames;
    vm_t *vm = bios_cookie->vm;
    int page_size = seL4_PageBits;

    uintptr_t frame_start = ROUND_DOWN(addr, BIT(page_size));
    if (frame_start < LOWER_BIOS_START || frame_start > LOWER_BIOS_START + LOWER_BIOS_SIZE) {
        ZF_LOGE("Error: Not BIOS region");
        return frame_result;
    }

    int page_idx = (frame_start - LOWER_BIOS_START) / BIT(page_size);
    error = vka_cspace_alloc_path(vm->vka, &return_frame);
    if (error) {
        ZF_LOGE("Failed to allocate cspace path for bios frame");
        return frame_result;
    }
    error = vka_cnode_copy(&return_frame, &bios_frames[page_idx], seL4_AllRights);
    if (error) {
        ZF_LOGE("Failed to cnode_copy for device frame");
        vka_cspace_free_path(vm->vka, return_frame);
        return frame_result;
    }
    frame_result.cptr = return_frame.capPtr;
    frame_result.rights = seL4_AllRights;
    frame_result.vaddr = frame_start;
    frame_result.size_bits = page_size;

    return frame_result;
}

static void *alloc_bios_memory(vm_t *vm, cspacepath_t **bios_frames)
{
    int err;
    unsigned int num_pages = ROUND_UP(LOWER_BIOS_SIZE, BIT(seL4_PageBits)) >> seL4_PageBits;
    seL4_CPtr caps[num_pages];
    cspacepath_t *bios_frames_paths = (cspacepath_t *)calloc(1, num_pages * sizeof(cspacepath_t));
    if (!bios_frames_paths) {
        ZF_LOGE("Failed to allocate caps for bios memory");
        return NULL;
    }
    uintptr_t current_addr = LOWER_BIOS_START;
    for (unsigned int i = 0; i < num_pages; i++) {
        vka_object_t frame;
        err = vka_alloc_frame(vm->vka, seL4_PageBits, &frame);
        if (err) {
            ZF_LOGE("Failed to allocate frame for bios addr 0x%x", (unsigned int)current_addr);
            return NULL;
        }
        vka_cspace_make_path(vm->vka, frame.cptr, &bios_frames_paths[i]);
        caps[i] = bios_frames_paths[i].capPtr;
        current_addr += BIT(seL4_PageBits);
    }

    void *bios_addr = vspace_map_pages(&vm->mem.vmm_vspace, caps, NULL, seL4_AllRights,
                                       num_pages, seL4_PageBits, 0);
    if (!bios_addr) {
        ZF_LOGE("Failed to map new pages for bios memory");
        return NULL;
    }
    memset(bios_addr, LOWER_BIOS_SIZE, 0);
    *bios_frames = bios_frames_paths;
    return bios_addr;
}

// Give some ACPI tables to the guest
int make_guest_acpi_tables(vm_t *vm)
{
    ZF_LOGD("Making ACPI tables\n");

    int cpus = vm->num_vcpus;

    int err;

    // XSDT and other tables
    void *tables[MAX_ACPI_TABLES];
    size_t table_sizes[MAX_ACPI_TABLES];
    int num_tables = 1;

    // MADT
    int madt_size = sizeof(acpi_madt_t)
                    /* + sizeof(acpi_madt_ioapic_t)*/
                    + sizeof(acpi_madt_local_apic_t) * cpus;
    acpi_madt_t *madt = calloc(1, madt_size);
    acpi_fill_table_head(&madt->header, "APIC", 3);
    madt->local_int_crt_address = APIC_DEFAULT_PHYS_BASE;
    madt->flags = 1;

    char *madt_entry = (char *)madt + sizeof(acpi_madt_t);

#if 0
    acpi_madt_ioapic_t ioapic = { // MADT IOAPIC entry
        .header = {
            .type = ACPI_APIC_IOAPIC,
            .length = sizeof(acpi_madt_ioapic_t)
        },
        .ioapic_id = 0,
        .address = IOAPIC_DEFAULT_PHYS_BASE,
        .gs_interrupt_base = 0 // TODO set this up?
    };
    memcpy(madt_entry, &ioapic, sizeof(ioapic));
    madt_entry += sizeof(ioapic);
#endif

    for (int i = 0; i < cpus; i++) { // MADT APIC entries
        acpi_madt_local_apic_t apic = {
            .header = {
                .type = ACPI_APIC_LOCAL,
                .length = sizeof(acpi_madt_local_apic_t)
            },
            .processor_id = i + 1,
            .apic_id = i,
            .flags = APIC_FLAGS_ENABLED
        };
        memcpy(madt_entry, &apic, sizeof(apic));
        madt_entry += sizeof(apic);
    }

    madt->header.length = madt_size;
    madt->header.checksum = acpi_calc_checksum((char *)madt, madt_size);

    tables[num_tables] = madt;
    table_sizes[num_tables] = madt_size;
    num_tables++;

    // Could set up other tables here...

    // DSDT
    int dsdt_size = sizeof(acpi_dsdt_t);
    acpi_dsdt_t *dsdt = calloc(1, dsdt_size);
    assert(NULL != dsdt);

    acpi_fill_table_head(&dsdt->header, "DSDT", 1);

    dsdt->header.length = dsdt_size;
    dsdt->header.checksum = acpi_calc_checksum((char *)dsdt, dsdt_size);

    tables[num_tables] = dsdt;
    table_sizes[num_tables] = dsdt_size;

    int dsdt_index = num_tables;

    num_tables++;

    // FADT
    int fadt_size = sizeof(acpi_fadt_t);
    acpi_fadt_t *fadt = calloc(1, fadt_size);
    assert(NULL != fadt);

    acpi_fill_table_head(&fadt->header, "FACP", 1);

    fadt->header.length = fadt_size;

    /* Hardcode some necessary data; Addresses taken from QEMU.
     * Info may vary on hardware platforms, but differences don't seem to affect
     * the guest
     */
    fadt->pm1a_evt_blk = 0x600;
    fadt->pm1_evt_len = 0x4;
    fadt->pm1a_cnt_blk = 0x604;
    fadt->pm1_cnt_len = 0x2;

    fadt->sci_int = 9;

    tables[num_tables] = fadt;
    table_sizes[num_tables] = fadt_size;

    int fadt_index = num_tables;

    num_tables++;

    // XSDT
    size_t xsdt_size = sizeof(acpi_xsdt_t) + sizeof(uint64_t) * (num_tables - 1);

    // How much space will all the tables take up?
    size_t acpi_size = xsdt_size;
    for (int i = 1; i < num_tables; i++) {
        acpi_size += table_sizes[i];
    }

    cspacepath_t *bios_frames;
    uintptr_t lower_bios_addr = (uintptr_t)alloc_bios_memory(vm, &bios_frames);
    if (lower_bios_addr == 0) {
        return -1;
    }

    uintptr_t xsdt_addr = lower_bios_addr + (XSDT_START - LOWER_BIOS_START);
    uintptr_t xsdt_vaddr = XSDT_START;

    acpi_xsdt_t *xsdt = calloc(1, xsdt_size);
    acpi_fill_table_head(&xsdt->header, "XSDT", 1);

    // Add previous tables to XSDT pointer list
    uintptr_t table_vaddr = xsdt_vaddr + xsdt_size;
    uint64_t *entry = (uint64_t *)((char *)xsdt + sizeof(acpi_xsdt_t));
    for (int i = 1; i < num_tables; i++) {
        *entry++ = (uint64_t)table_vaddr;
        table_vaddr += table_sizes[i];
    }

    xsdt->header.length = xsdt_size;
    xsdt->header.checksum = acpi_calc_checksum((char *)xsdt, xsdt_size);

    tables[0] = xsdt;
    table_sizes[0] = xsdt_size;

    // Copy all the tables to guest
    uintptr_t table_paddr = xsdt_addr;
    table_vaddr = xsdt_vaddr;
    for (int i = 0; i < num_tables; i++) {

        // Need to fill in DSDT address
        if (i == fadt_index) {
            fadt->dsdt_address = table_vaddr - table_sizes[dsdt_index];
            fadt->x_dsdt_address = table_vaddr - table_sizes[dsdt_index];
            fadt->header.checksum = acpi_calc_checksum((char *)fadt, fadt_size);
        }

        ZF_LOGD("ACPI table \"%.4s\", addr = %p, size = %zu bytes\n",
                (char *)tables[i], (void *)table_vaddr, table_sizes[i]);
        memcpy((void *)table_paddr, (char *)tables[i], table_sizes[i]);
        table_paddr += table_sizes[i];
        table_vaddr += table_sizes[i];
    }

    // RSDP
    uintptr_t rsdp_addr = lower_bios_addr + (ACPI_START - LOWER_BIOS_START);
    acpi_rsdp_t rsdp = {
        .signature = "RSD PTR ",
        .oem_id = "NICTA ",
        .revision = 2, /* ACPI v3*/
        .checksum = 0,
        .rsdt_address = xsdt_vaddr,
        /* rsdt_addrss will not be inspected as the xsdt is present.
           This is not ACPI 1 compliant */
        .length = sizeof(acpi_rsdp_t),
        .xsdt_address = xsdt_vaddr,
        .extended_checksum = 0,
        .reserved = {0}
    };

    rsdp.checksum = acpi_calc_checksum((char *)&rsdp, 20);
    rsdp.extended_checksum = acpi_calc_checksum((char *)&rsdp, sizeof(rsdp));

    ZF_LOGD("ACPI RSDP addr = %p\n", (void *)rsdp_addr);

    memcpy((void *)rsdp_addr, (char *)&rsdp, sizeof(rsdp));

    struct bios_iterator_cookie *bios_cookie = calloc(1, sizeof(struct bios_iterator_cookie));
    if (!bios_cookie) {
        ZF_LOGE("Failed to allocate bios iterator cookie");
        return -1;
    }
    bios_cookie->vm = vm;
    bios_cookie->bios_frames = bios_frames;
    vm_memory_reservation_t *bios_reservation = vm_reserve_memory_at(vm, LOWER_BIOS_START, LOWER_BIOS_SIZE,
                                                                     default_error_fault_callback, NULL);
    if (!bios_reservation) {
        ZF_LOGE("Failed to reserve LOWER BIOS memory");
        return -1;
    }
    return vm_map_reservation(vm, bios_reservation, bios_memory_iterator, (void *)bios_cookie);
}
