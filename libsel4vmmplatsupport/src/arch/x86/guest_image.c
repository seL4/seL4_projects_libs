/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <elf/elf.h>
#include <sel4utils/mapping.h>
#include <vka/capops.h>
#include <cpio/cpio.h>

#include <sel4vmmplatsupport/guest_image.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_memory.h>
#include <sel4vm/guest_ram.h>

#ifdef CONFIG_ARCH_X86_64
#define ELF_HEADER_SIZE 512
#else
#define ELF_HEADER_SIZE 256
#endif

#define ISELF32(elfFile) ( ((Elf32_Ehdr *)elfFile)->e_ident[EI_CLASS] == ELFCLASS32 )
#define ISELF64(elfFile) ( ((Elf64_Ehdr *)elfFile)->e_ident[EI_CLASS] == ELFCLASS64 )

typedef struct boot_guest_cookie {
    vm_t *vm;
    FILE *file;
} boot_guest_cookie_t;

/* Reads the elf header and elf program headers from a file when given a sufficiently
 * large memory buffer
 */
static int read_elf_headers(void *buf, vm_t *vm, FILE *file, size_t buf_size, elf_t *elf)
{
    size_t result;
    if (buf_size < ELF_HEADER_SIZE) {
        return -1;
    }
    fseek(file, 0, SEEK_SET);
    result = fread(buf, buf_size, 1, file);
    if (result != 1) {
        return -1;
    }

    /* Check for correct ELF version on the current architecture */
#ifdef CONFIG_ARCH_X86_64
    if (ISELF32(buf)) {
        return -1;
    }
#else
    if (ISELF64(buf)) {
        return -1;
    }
#endif

    return elf_newFile_maybe_unsafe(buf, buf_size, true, false, elf);
}

static int guest_elf_write_address(vm_t *vm, uintptr_t paddr, void *vaddr, size_t size, size_t offset, void *cookie)
{
    memcpy(vaddr, cookie + offset, size);
    return 0;
}

static int guest_elf_read_address(vm_t *vm, uintptr_t paddr, void *vaddr, size_t size, size_t offset, void *cookie)
{
    memcpy(cookie + offset, vaddr, size);
    return 0;
}

int guest_elf_relocate(vm_t *vm, const char *relocs_filename, guest_kernel_image_t *image)
{
    int delta = image->kernel_image_arch.relocation_offset;
    if (delta == 0) {
        /* No relocation needed. */
        return 0;
    }

    uintptr_t load_addr = image->kernel_image_arch.link_paddr;
    ZF_LOGI("plat: relocating guest kernel from 0x%x --> 0x%x", (unsigned int)load_addr,
            (unsigned int)(load_addr + delta));

    /* Open the relocs file. */
    ZF_LOGI("plat: opening relocs file %s", relocs_filename);

    size_t relocs_size = 0;
    FILE *file = fopen(relocs_filename, "r");
    if (!file) {
        ZF_LOGE("ERROR: Guest OS kernel relocation is required, but corresponding"
                "%s was not found. This is most likely due to a Makefile"
                "error, or configuration error.\n", relocs_filename);
        return -1;
    }
    fseek(file, 0, SEEK_END);
    relocs_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    /* The relocs file is the same relocs file format used by the Linux kernel decompressor to
     * relocate the Linux kernel:
     *
     *     0 - zero terminator for 64 bit relocations
     *     64 bit relocation repeated
     *     0 - zero terminator for 32 bit relocations
     *     32 bit relocation repeated
     *     <EOF>
     *
     * So we work backwards from the end of the file, and modify the guest kernel OS binary.
     * We only support 32-bit relocations, and ignore the 64-bit data.
     *
     * src: Linux kernel 3.5.3 arch/x86/boot/compressed/misc.c
     */
    uint32_t last_relocated_vaddr = 0xFFFFFFFF;
    uint32_t num_relocations = relocs_size / sizeof(uint32_t) - 1;
    for (int i = 0; ; i++) {
        uint32_t vaddr;
        /* Get the next relocation from the relocs file. */
        uint32_t offset = relocs_size - (sizeof(uint32_t) * (i + 1));
        fseek(file, offset, SEEK_SET);
        size_t result = fread(&vaddr, sizeof(uint32_t), 1, file);
        ZF_LOGF_IF(result != 1, "Read failed unexpectedly");
        if (!vaddr) {
            break;
        }
        assert(i * sizeof(uint32_t) < relocs_size);
        last_relocated_vaddr = vaddr;

        /* Calculate the corresponding guest-physical address at which we have already
           allocated and mapped the ELF contents into. */
        assert(vaddr >= (uint32_t)image->kernel_image_arch.link_vaddr);
        uintptr_t guest_paddr = (uintptr_t)vaddr - (uintptr_t)image->kernel_image_arch.link_vaddr +
                                (uintptr_t)(load_addr + delta);

        /* Perform the relocation. */
        ZF_LOGI("   reloc vaddr 0x%x guest_addr 0x%x", (unsigned int)vaddr, (unsigned int)guest_paddr);
        uint32_t addr;
        vm_ram_touch(vm, guest_paddr, sizeof(int),
                     guest_elf_read_address, &addr);
        addr += delta;
        vm_ram_touch(vm, guest_paddr, sizeof(int),
                     guest_elf_write_address, &addr);

        if (i && i % 50000 == 0) {
            ZF_LOGE("    %u relocs done.", i);
        }
    }
    ZF_LOGI("plat: last relocated addr was %d", last_relocated_vaddr);
    ZF_LOGI("plat: %d kernel relocations completed.", num_relocations);
    (void) last_relocated_vaddr;

    if (num_relocations == 0) {
        ZF_LOGE("Relocation required, but Kernel has not been build with CONFIG_RELOCATABLE.");
        return -1;
    }

    fclose(file);
    return 0;
}

/* TODO: Refactor and stop rewriting fucking elf loading code */
static int load_guest_segment(vm_t *vm, seL4_Word source_offset,
                              seL4_Word dest_addr, unsigned int segment_size, unsigned int file_size, FILE *file)
{

    int ret;
    unsigned int page_size = seL4_PageBits;
    assert(file_size <= segment_size);

    /* Allocate a cslot for duplicating frame caps */
    cspacepath_t dup_slot;
    ret = vka_cspace_alloc_path(vm->vka, &dup_slot);
    if (ret) {
        ZF_LOGE("Failed to allocate slot");
        return ret;
    }

    size_t current = 0;
    size_t remain = file_size;
    while (current < segment_size) {
        /* Retrieve the mapping */
        seL4_CPtr cap;
        cap = vspace_get_cap(&vm->mem.vm_vspace, (void *)dest_addr);
        if (!cap) {
            ZF_LOGE("Failed to find frame cap while loading elf segment at %p", (void *)dest_addr);
            return -1;
        }
        cspacepath_t cap_path;
        vka_cspace_make_path(vm->vka, cap, &cap_path);

        /* Copy cap and map into our vspace */
        vka_cnode_copy(&dup_slot, &cap_path, seL4_AllRights);
        void *map_vaddr = vspace_map_pages(&vm->mem.vmm_vspace, &dup_slot.capPtr, NULL, seL4_AllRights, 1, page_size, 1);
        if (!map_vaddr) {
            ZF_LOGE("Failed to map page into host vspace");
            return -1;
        }

        /* Copy the contents of page from ELF into the mapped frame. */
        size_t offset = dest_addr & ((BIT(page_size)) - 1);

        void *copy_vaddr = map_vaddr + offset;
        size_t copy_len = (BIT(page_size)) - offset;

        if (remain > 0) {
            if (copy_len > remain) {
                /* Don't copy past end of data. */
                copy_len = remain;
            }

            ZF_LOGI("load page src %zu dest %p remain %zu offset %zu copy vaddr %p "
                    "copy len %zu\n", source_offset, (void *)dest_addr, remain, offset, copy_vaddr, copy_len);

            fseek(file, source_offset, SEEK_SET);
            size_t result = fread(copy_vaddr, copy_len, 1, file);
            ZF_LOGF_IF(result != 1, "Read failed unexpectedly");
            source_offset += copy_len;
            remain -= copy_len;
        } else {
            memset(copy_vaddr, 0, copy_len);
        }

        dest_addr += copy_len;

        current += copy_len;

        /* Unamp the page and delete the temporary cap */
        vspace_unmap_pages(&vm->mem.vmm_vspace, map_vaddr, 1, page_size, NULL);
        vka_cnode_delete(&dup_slot);
    }

    return 0;
}

static int load_guest_elf(vm_t *vm, const char *image_name, uintptr_t load_address, size_t alignment,
                          guest_kernel_image_t *guest_image)
{
    elf_t kernel_elf;
    char elf_file[ELF_HEADER_SIZE];
    int ret;
    FILE *file = fopen(image_name, "r");
    if (!file) {
        ZF_LOGE("Guest kernel elf \"%s\" not found.", image_name);
        return -1;
    }

    ret = read_elf_headers(elf_file, vm, file, sizeof(elf_file), &kernel_elf);
    if (ret < 0) {
        ZF_LOGE("Guest elf \"%s\" invalid.", image_name);
        return -1;
    }

    unsigned int n_headers = elf_getNumProgramHeaders(&kernel_elf);

    /* Round up by the alignment. We just hope its still in the memory region.
     * if it isn't we will just fail when we try and get the frame */
    uintptr_t load_addr = ROUND_UP(load_address, alignment);
    /* Calculate relocation offset. */
    uintptr_t guest_kernel_addr = UINTPTR_MAX;
    uintptr_t guest_kernel_vaddr = UINTPTR_MAX;
    for (int i = 0; i < n_headers; i++) {
        if (elf_getProgramHeaderType(&kernel_elf, i) != PT_LOAD) {
            continue;
        }
        seL4_Word addr = elf_getProgramHeaderPaddr(&kernel_elf, i);
        if (addr < guest_kernel_addr) {
            guest_kernel_addr = addr;
        }
        seL4_Word vaddr = elf_getProgramHeaderVaddr(&kernel_elf, i);
        if (vaddr && (vaddr < guest_kernel_vaddr)) {
            guest_kernel_vaddr = vaddr;
        }
    }

    int guest_relocation_offset = (int)((int64_t)load_addr - (int64_t)guest_kernel_addr);

    ZF_LOGI("Guest kernel is compiled to be located at paddr 0x%x vaddr 0x%x",
            (unsigned int)guest_kernel_addr, (unsigned int)guest_kernel_vaddr);
    ZF_LOGI("Guest kernel allocated 1:1 start is at paddr = 0x%x", (unsigned int)load_addr);
    ZF_LOGI("Therefore relocation offset is %d (%s0x%x)",
            guest_relocation_offset,
            guest_relocation_offset < 0 ? "-" : "",
            abs(guest_relocation_offset));

    for (int i = 0; i < n_headers; i++) {
        seL4_Word source_offset, dest_addr;
        unsigned int file_size, segment_size;

        /* Skip unloadable program headers. */
        if (elf_getProgramHeaderType(&kernel_elf, i) != PT_LOAD) {
            continue;
        }

        /* Fetch information about this segment. */
        source_offset = elf_getProgramHeaderOffset(&kernel_elf, i);
        file_size = elf_getProgramHeaderFileSize(&kernel_elf, i);
        segment_size = elf_getProgramHeaderMemorySize(&kernel_elf, i);

        dest_addr = elf_getProgramHeaderPaddr(&kernel_elf, i);
        dest_addr += guest_relocation_offset;

        if (!segment_size) {
            /* Zero sized segment, ignore. */
            continue;
        }

        /* Load this ELf segment. */
        ret = load_guest_segment(vm, source_offset, dest_addr, segment_size, file_size, file);
        if (ret) {
            return ret;
        }

        /* Record it as allocated */
        vm_ram_mark_allocated(vm, dest_addr, segment_size);
    }

    /* Record the entry point. */
    guest_image->kernel_image_arch.entry = elf_getEntryPoint(&kernel_elf);
    guest_image->kernel_image_arch.entry += guest_relocation_offset;

    /* Remember where we started loading the kernel to fix up relocations in future */
    guest_image->kernel_image.load_paddr = load_addr;
    guest_image->kernel_image_arch.link_paddr = guest_kernel_addr;
    guest_image->kernel_image_arch.link_vaddr = guest_kernel_vaddr;
    guest_image->kernel_image_arch.relocation_offset = guest_relocation_offset;
    guest_image->kernel_image.alignment = alignment;

    fclose(file);

    return 0;
}

int vm_load_guest_kernel(vm_t *vm, const char *kernel_name, uintptr_t load_address, size_t alignment,
                         guest_kernel_image_t *guest_kernel_image)
{
    int err;
    err = load_guest_elf(vm, kernel_name, load_address, alignment, guest_kernel_image);
    if (err) {
        ZF_LOGE("Failed to load guest elf");
        return err;
    }
    if (guest_kernel_image->kernel_image_arch.is_reloc_enabled) {
        err = guest_elf_relocate(vm, guest_kernel_image->kernel_image_arch.relocs_file, guest_kernel_image);
        if (err) {
            ZF_LOGE("Failed to relocation guest kernel elf");
        }
    }
    return err;
}

static int load_module_continued(vm_t *vm, uintptr_t paddr, void *addr, size_t size, size_t offset, void *cookie)
{
    boot_guest_cookie_t *pass = (boot_guest_cookie_t *) cookie;
    fseek(pass->file, offset, SEEK_SET);
    size_t result = fread(addr, size, 1, pass->file);
    ZF_LOGF_IF(result != 1, "Read failed unexpectedly");

    return 0;
}

int vm_load_guest_module(vm_t *vm, const char *module_name, uintptr_t load_address, size_t alignment,
                         guest_image_t *guest_image)
{
    ZF_LOGI("Loading module \"%s\" at 0x%x\n", module_name, (unsigned int)load_address);

    size_t module_size = 0;
    FILE *file = fopen(module_name, "r");
    if (!file) {
        ZF_LOGE("Module \"%s\" not found.", module_name);
        return -1;
    }
    fseek(file, 0, SEEK_END);
    module_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    if (!module_size) {
        ZF_LOGE("Module has zero size. This is probably not what you want.");
        return -1;
    }

    vm_ram_mark_allocated(vm, load_address, module_size);
    boot_guest_cookie_t pass = { .vm = vm, .file = file};
    vm_ram_touch(vm, load_address, module_size, load_module_continued, &pass);

    fclose(file);

    guest_image->load_paddr = load_address;
    guest_image->size = module_size;

    return 0;
}
