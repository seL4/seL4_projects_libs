#
# Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.8.2)

project(libsel4vmmplatsupport C)

add_compile_options(-std=gnu99)

config_option(
    LibSel4VMMPlatsupportVESAFrameBuffer
    VMM_PLATSUPPORT_VESA_FRAMEBUFFER
    "Expose VESA Frame Buffer
        Support for mapping the VESA frame buffer frames into the guest OS.
        This option is useful for working around non-working video drivers otherwise,
        due to missing features or such."
    DEFAULT
    ON
    DEPENDS
    "KernelPlatPC99"
)

mark_as_advanced(LibSel4VMMPlatsupportVESAFrameBuffer)

add_config_library(sel4vmmplatsupport "${configure_string}")

file(
    GLOB
        sources
        src/*.c
        src/drivers/*.c
        src/arch/${KernelArch}/*.c
        src/arch/${KernelArch}/devices/*.c
        src/arch/${KernelArch}/drivers/*.c
        src/sel4_arch/${KernelSel4Arch}/*.c
        src/plat/${KernelPlatform}/*.c
        src/plat/${KernelPlatform}/devices/*.c
)

add_library(sel4vmmplatsupport STATIC EXCLUDE_FROM_ALL ${sources})
target_include_directories(
    sel4vmmplatsupport
    PUBLIC include arch_include/${KernelArch} plat_include/${KernelPlatform}
)
target_include_directories(sel4vmmplatsupport PRIVATE src/sel4_arch/${KernelSel4Arch})
target_include_directories(sel4vmmplatsupport PRIVATE src/arch/${KernelArch})

target_link_libraries(
    sel4vmmplatsupport
    muslc
    sel4
    utils
    sel4utils
    platsupport
    pci
    ethdrivers
    sel4vm
    fdt
    fdtgen
    satadrivers
    ringbuffer
    sel4_autoconf
    sel4vm_Config
    usbdrivers_Config
    sel4vmmplatsupport_Config
)
