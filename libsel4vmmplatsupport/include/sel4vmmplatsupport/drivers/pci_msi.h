
/*
 * Copyright 2022, UNSW (ABN 57 195 873 179)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <stdint.h>
#include <sel4/sel4.h>
#include <sel4vmmplatsupport/drivers/pci_helper.h>
#include <utils/util.h>

typedef struct pci_msi_control {
    union {
        struct {
            uint16_t enable             :1,
                     multi_msg_capable  :3,
                     multi_msg_enable   :3,
                     addr_64_bit        :1,
                     reserved		    :8;
        };
        uint16_t value;
    };
} PACKED pci_msi_control_t;

typedef struct pci_msi_data {
    union {
        struct {
            uint32_t vector		        :8,
                     delivery_mode		:3,
                     dest_mode_logical	:1,
                     reserved		    :2,
                     assert             :1,
                     is_level		    :1;
        };
        uint32_t value;
    };
} PACKED pci_msi_data_t;

typedef struct pci_msi_addr_lo {
    union {
        struct {
            uint32_t reserved_0		    :2,
                     dest_mode_logical	:1,
                     redirect_hint		:1,
                     reserved_1		    :1,
                     virt_destid_8_14	:7,
                     destid_0_7		    :8,
                     base_address		:12; /* Always 0xFEE */
        };
        uint32_t value;
    };
} PACKED pci_msi_addr_lo_t;

typedef struct pci_msi_addr_hi {
	uint32_t reserved	    :8,
		     destid_8_31	:24;
} PACKED pci_msi_msg_addr_hi_t;
