#
# Copyright 2018, Data61
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230.
#
# This software may be distributed and modified according to the terms of
# the BSD 2-Clause license. Note that NO WARRANTY is provided.
# See "LICENSE_BSD2.txt" for details.
#
# @TAG(DATA61_BSD)
#

cmake_minimum_required(VERSION 3.8.2)

project(libsel4arm-vmm C)

set(configure_string "")

config_option(
    LibSel4ArmVMMOnDemandDeviceInstall
    ONDEMAND_DEVICE_INSTALL
    "Allow the VMM to install arbitrary devices into the VM as they are accessed."
    DEFAULT
    ON
)

config_option(
    LibSel4ArmVMMVchanSupport
    LIB_SEL4_ARM_VMM_VCHAN_SUPPORT
    "Support vchan connections"
    DEFAULT
    OFF
)

add_config_library(sel4arm-vmm "${configure_string}")

add_compile_options(-std=gnu99)

file(
    GLOB
        sources
        src/*.c
        src/images/*.c
        src/devices/*.c
        src/plat/${KernelPlatform}/*/*.c
        src/sel4_arch/${KernelSel4Arch}/*.c
)
list(APPEND
           sources
           src/arch/arm/devices/vram.c
           src/arch/arm/devices/vgic_v2.c
)

add_library(sel4arm-vmm STATIC EXCLUDE_FROM_ALL ${sources})
target_include_directories(sel4arm-vmm PUBLIC include plat_include/${KernelPlatform})
target_include_directories(sel4arm-vmm PUBLIC include sel4_arch_include/${KernelSel4Arch})
target_link_libraries(
    sel4arm-vmm
    muslc
    sel4
    sel4simple
    utils
    sel4utils
    sel4vka
    sel4vspace
    sel4allocman
    sel4dma
    platsupport
    sel4pci
    sel4vmmcore
    sel4_autoconf
    sel4arm-vmm_Config
    usbdrivers_Config
)
if(LibSel4ArmVMMVchanSupport)
    target_link_libraries(sel4arm-vmm sel4vchan)
endif()
if(LibUSB)
    target_link_libraries(sel4arm-vmm usbdrivers)
endif()
