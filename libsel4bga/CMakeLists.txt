#
# Copyright 2018, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.16.0)

# Exclude this project in ARM builds as it will have no sources and thus not be a valid library

if(KernelArchX86)

    project(libsel4bga C)

    add_library(sel4bga STATIC EXCLUDE_FROM_ALL src/arch-x86/bga.c)
    target_include_directories(sel4bga PUBLIC "arch_include/x86")
    target_link_libraries(sel4bga muslc)

endif()
