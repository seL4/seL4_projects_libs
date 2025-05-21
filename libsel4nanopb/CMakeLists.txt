#
# Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.16.0)

project(libsel4nanopb C)

include(nanopb)

include_directories(${NANOPB_INCLUDE_DIRS})

add_library(sel4nanopb EXCLUDE_FROM_ALL src/common.c)
target_include_directories(sel4nanopb PUBLIC include)
target_link_libraries(sel4nanopb muslc utils sel4 nanopb)
