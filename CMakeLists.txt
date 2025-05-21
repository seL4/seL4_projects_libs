#
# Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.16.0)

add_subdirectory(libsel4vm)
add_subdirectory(libsel4vchan)
add_subdirectory(libsel4dma)
add_subdirectory(libsel4bga)
add_subdirectory(libsel4keyboard)
add_subdirectory(libsel4vmmplatsupport)

set(configure_string "")
config_option(LibNanopb BUILD_NANOPB "Build Nanopb and related libraries" DEFAULT OFF)
mark_as_advanced(LibNanopb)
if(LibNanopb)
    add_subdirectory(libsel4nanopb)
    add_subdirectory(libsel4rpc)
endif()
