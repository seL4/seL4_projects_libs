#
# Copyright 2018, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.16.0)

project(libsel4keyboard C)

# Exclude this project in ARM builds as it will have no sources and thus not be a valid library

if(KernelArchX86)

    set(deps src/arch-x86/keyboard.c src/codes.c)

    add_library(sel4keyboard STATIC EXCLUDE_FROM_ALL ${deps})
    target_include_directories(sel4keyboard PUBLIC include "arch_include/x86")
    target_link_libraries(sel4keyboard muslc)

endif()
