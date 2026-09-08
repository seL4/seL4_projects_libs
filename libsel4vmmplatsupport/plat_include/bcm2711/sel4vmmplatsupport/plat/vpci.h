/*
 * Copyright 2022, Technology Innovation Institute
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

/* PCI host bridge configuration space */
#define PCI_CFG_REGION_ADDR 0x100000000ULL
/* PCI host bridge IO space */
#define PCI_IO_REGION_ADDR 0x101000000ULL
/* Size of PCI configuration space */
#define PCI_CFG_REGION_SIZE 0x1000000
/* Size of PCI IO space  */
#define PCI_IO_REGION_SIZE 0x10000
/* PCI memory space */
#define PCI_MEM_REGION_ADDR 0x60000000ULL
/* PCI memory space size */
#define PCI_MEM_REGION_SIZE 0x60000000

/* FDT IRQ controller address cells definition */
#define GIC_ADDRESS_CELLS 0x1
