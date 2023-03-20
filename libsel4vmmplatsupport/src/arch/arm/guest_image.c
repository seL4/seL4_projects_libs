/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <elf/elf.h>
#include <vka/capops.h>
#include <cpio/cpio.h>
#include <sel4utils/sel4_zf_logif.h>

#include <sel4vm/guest_vm.h>
#include <sel4vm/guest_memory.h>
#include <sel4vm/guest_ram.h>

#include <sel4vmmplatsupport/guest_image.h>

#define UIMAGE_MAGIC 0x56190527
#define ZIMAGE_MAGIC 0x016F2818
#define DTB_MAGIC    0xedfe0dd0
#define INITRD_GZ_MAGIC 0x8b1f

struct dtb_hdr {
    uint32_t magic;
#if 0
    uint32_t size;
    uint32_t off_dt_struct;
    uint32_t off_dt_strings;
    uint32_t off_mem_rsvmap;
    uint32_t version;
    uint32_t comp_version;
    uint32_t boot_cpuid;
    uint32_t size_dt_strings;
    uint32_t size_dt_struct;
#endif
};

struct initrd_gz_hdr {
    uint16_t magic;
    uint8_t compression;
    uint8_t flags;
};

struct zimage_hdr {
    uint32_t code[9];
    uint32_t magic;
    uint32_t start;
    uint32_t end;
};

static int is_uImage(void *file)
{
    uint32_t magic = UIMAGE_MAGIC;
    return memcmp(file, &magic, sizeof(magic));
}

static int is_zImage(void *file)
{
    struct zimage_hdr *hdr;
    hdr = (struct zimage_hdr *)file;
    return hdr->magic != ZIMAGE_MAGIC;
}

static int is_dtb(void *file)
{
    struct dtb_hdr *hdr;
    hdr = (struct dtb_hdr *)file;
    return hdr->magic != DTB_MAGIC;
}

static int is_initrd(void *file)
{
    /* We currently only support initrd files in the gzip format */
    struct initrd_gz_hdr *hdr;
    hdr = (struct initrd_gz_hdr *)file;
    return hdr->magic != INITRD_GZ_MAGIC;
}

static enum img_type image_get_type(void *file)
{
    if (elf_check_magic(file) == 0) {
        return IMG_ELF;
    } else if (is_zImage(file) == 0) {
        return IMG_ZIMAGE;
    } else if (is_uImage(file) == 0) {
        return IMG_UIMAGE;
    } else if (is_dtb(file) == 0) {
        return IMG_DTB;
    } else if (is_initrd(file) == 0) {
        return IMG_INITRD_GZ;
    } else {
        return IMG_BIN;
    }
}

static uintptr_t zImage_get_load_address(void *file, uintptr_t ram_base)
{
    struct zimage_hdr *hdr;
    hdr = (struct zimage_hdr *)file;
    return (hdr->start != 0) ? hdr->start : (ram_base + 0x8000);
}

static int get_guest_image_type(const char *image_name, enum img_type *image_type, Elf64_Ehdr *header)
{
    int fd;
    fd = open(image_name, 0);
    if (fd == -1) {
        ZF_LOGE("Error: Unable to open image \'%s\'", image_name);
        return -1;
    }

    size_t len = read(fd, header, sizeof(*header));
    if (len != sizeof(*header)) {
        ZF_LOGE("Could not read len. File is likely corrupt");
        close(fd);
        return -1;
    }
    *image_type = image_get_type(header);
    close(fd);
    return 0;
}

static int guest_write_address(vm_t *vm, uintptr_t paddr, void *vaddr, size_t size, size_t offset, void *cookie)
{
    int fd = *((int *)cookie);

    /* Load the image */
    size_t len = read(fd, vaddr, size);
    if (len != size) {
        ZF_LOGE("Bytes read from the file server (%d) don't match expected length (%d)", len, size);
        return -1;
    }

    if (vm->mem.clean_cache) {
        seL4_CPtr cap = vspace_get_cap(&vm->mem.vmm_vspace, vaddr);
        if (cap == seL4_CapNull) {
            /* Not sure how we would get here, something has gone pretty wrong */
            ZF_LOGE("Failed to get vmm cap for vaddr: %p", vaddr);
            return -1;
        }
        int error = seL4_ARM_Page_CleanInvalidate_Data(cap, 0, PAGE_SIZE_4K);
        ZF_LOGF_IFERR(error, "seL4_ARM_Page_CleanInvalidate_Data failed");
    }
    return 0;
}

static int load_image(vm_t *vm, const char *image_name, uintptr_t load_addr,  size_t *resulting_image_size)
{
    int fd;
    int error;

    fd = open(image_name, 0);
    if (fd == -1) {
        ZF_LOGE("Error: Unable to find image \'%s\'", image_name);
        return -1;
    }

    size_t file_size = lseek(fd, 0, SEEK_END);
    lseek(fd, 0, SEEK_SET);

    if (0 == file_size) {
        ZF_LOGE("Error: \'%s\' has zero size", image_name);
        return -1;
    }

    vm_ram_mark_allocated(vm, load_addr, ROUND_UP(file_size, PAGE_SIZE_4K));
    error = vm_ram_touch(vm, load_addr, file_size, guest_write_address, (void *)&fd);
    if (error) {
        ZF_LOGE("Error: Failed to load \'%s\'", image_name);
        close(fd);
        return -1;
    }

    *resulting_image_size = file_size;
    close(fd);
    return 0;
}

static uintptr_t load_guest_kernel_image(vm_t *vm, const char *kernel_image_name, uintptr_t load_base_addr,
                                         size_t *image_size)
{
    int err;
    uintptr_t load_addr;
    enum img_type ret_file_type;
    Elf64_Ehdr header = {0};
    err = get_guest_image_type(kernel_image_name, &ret_file_type, &header);
    if (err) {
        return 0;
    }
    /* Determine the load address */
    switch (ret_file_type) {
    case IMG_BIN:
        load_addr = vm->entry;
        break;
    case IMG_ZIMAGE:
        load_addr = zImage_get_load_address(&header, load_base_addr);
        break;
    default:
        ZF_LOGE("Error: Unknown Linux image format for \'%s\'", kernel_image_name);
        return 0;
    }
    err = load_image(vm, kernel_image_name, load_addr, image_size);
    if (err) {
        return 0;
    }
    return load_addr;
}

static uintptr_t load_guest_module_image(vm_t *vm, const char *image_name, uintptr_t load_base_addr, size_t *image_size)
{
    int err;
    uintptr_t load_addr;
    enum img_type ret_file_type;
    Elf64_Ehdr header = {0};
    err = get_guest_image_type(image_name, &ret_file_type, &header);
    if (err) {
        return 0;
    }
    /* Determine the load address */
    switch (ret_file_type) {
    case IMG_DTB:
    case IMG_INITRD_GZ:
        load_addr = load_base_addr;
        break;
    default:
        ZF_LOGE("Error: Unknown Linux image format for \'%s\'", image_name);
        return 0;
    }
    err = load_image(vm, image_name, load_addr, image_size);
    if (err) {
        return 0;
    }
    return load_addr;
}

int vm_load_guest_kernel(vm_t *vm, const char *kernel_name, uintptr_t load_address, size_t alignment,
                         guest_kernel_image_t *guest_kernel_image)
{
    uintptr_t load_addr;
    size_t kernel_len;
    if (!guest_kernel_image) {
        ZF_LOGE("Invalid guest_image_t object");
        return -1;
    }
    load_addr = load_guest_kernel_image(vm, kernel_name, load_address, &kernel_len);
    if (0 == load_addr) {
        return -1;
    }
    guest_kernel_image->kernel_image.load_paddr = load_addr;
    guest_kernel_image->kernel_image.size = kernel_len;
    return 0;
}

int vm_load_guest_module(vm_t *vm, const char *module_name, uintptr_t load_address, size_t alignment,
                         guest_image_t *guest_image)
{
    uintptr_t load_addr;
    size_t module_len;

    if (!guest_image) {
        ZF_LOGE("Invalid guest_image_t object");
        return -1;
    }

    load_addr = load_guest_module_image(vm, module_name, load_address, &module_len);
    if (0 == load_addr) {
        return -1;
    }

    guest_image->load_paddr = load_addr;
    guest_image->size = module_len;
    return 0;
}
