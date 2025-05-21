#
# Copyright 2018, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.16.0)

project(libsel4dma C)

add_compile_options(-std=gnu99)

file(GLOB sources src/*.c)

add_library(sel4dma STATIC EXCLUDE_FROM_ALL ${sources})
target_include_directories(sel4dma PUBLIC include)
target_link_libraries(sel4dma PUBLIC muslc platsupport PRIVATE sel4_autoconf)
