/*
 * Copyright 2022, UNSW (ABN 57 195 873 179)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once
/***
 * @module vmm_pci_helper.h
 * This interface presents a series of helpers for timer support on x86.
 */

typedef uint64_t (*timer_tsc_freq_fn_t)(void);
typedef int (*timer_oneshot_absolute_fn_t)(uint64_t ns);
typedef int (*timer_oneshot_relative_fn_t)(uint64_t ns);
typedef int (*timer_stop_fn_t)(void);

struct timer_functions {
    timer_tsc_freq_fn_t tsc_freq;
    timer_oneshot_absolute_fn_t oneshot_absolute;
    timer_oneshot_relative_fn_t oneshot_relative;
    timer_stop_fn_t stop;
};
