/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#define VCPU_BADGE_CREATE(idx)  ((idx) + 1)
#define VCPU_BADGE_IDX(badge)   ((badge) - 1)
#define MAX_VCPU_BADGE          CONFIG_MAX_NUM_NODES
#define MIN_VCPU_BADGE          1
