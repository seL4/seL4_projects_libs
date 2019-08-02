#
# Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.8.2)

project(libsel4vm C)

set(configure_string "")

add_compile_options(-std=gnu99)

config_option(
    LibSel4VMDeferMemoryMap
    LIB_SEL4VM_DEFER_MEMORY_MAP
    "Defer mappings of memory reservations until faulted on"
    DEFAULT
    OFF
)
config_option(LibSel4VMVMXTimerDebug LIB_VM_VMX_TIMER_DEBUG "Use VMX Pre-Emption timer for debugging
    Will cause a regular vmexit to happen based on VMX pre-emption
    timer. At each exit the guest state will be printed out. This
    can be used to aid debugging when running a guest causes nothing
    to appear to happen" DEFAULT OFF DEPENDS "KernelArchX86")

config_string(
    LibSel4VMVMXTimerTimeout
    LIB_VM_VMX_TIMER_TIMEOUT
    "Cycles between VMX timer pre-emptions
    Cycles between timer exits. Time spent loading the VMCS context
    count, so setting this too low may result in the guest making
    no progress"
    DEFAULT
    4000
    DEPENDS
    "LibSel4VMVMXTimerDebug"
)

mark_as_advanced(LibSel4VMDeferMemoryMap LibSel4VMVMXTimerDebug LibSel4VMVMXTimerTimeout)

add_config_library(sel4vm "${configure_string}")

file(
    GLOB
        sources
        src/*.c
        src/arch/${KernelArch}/*.c
        src/arch/${KernelArch}/i8259/*.c
        src/arch/${KernelArch}/processor/*.c
        src/sel4_arch/${KernelSel4Arch}/*.c
)

if(KernelArchARM)
    list(APPEND sources src/arch/arm/vgic/vgic_v2.c)
endif()

add_library(sel4vm STATIC EXCLUDE_FROM_ALL ${sources})
target_include_directories(
    sel4vm
    PUBLIC include arch_include/${KernelArch} sel4_arch_include/${KernelSel4Arch}
)
target_include_directories(
    sel4vm
    PRIVATE src src/arch/${KernelArch} src/sel4_arch/${KernelSel4Arch}
)
target_link_libraries(
    sel4vm
    muslc
    sel4
    sel4simple
    utils
    sel4utils
    sel4vka
    sel4vspace
    platsupport
    sel4_autoconf
    sel4vm_Config
)
