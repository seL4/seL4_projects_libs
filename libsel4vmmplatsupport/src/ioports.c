/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdio.h>
#include <stdlib.h>

#include <sel4utils/util.h>
#include <sel4vmmplatsupport/ioports.h>

static int io_port_compare_by_range(const void *pkey, const void *pelem)
{
    unsigned int key = (unsigned int)(uintptr_t)pkey;
    const ioport_entry_t *entry = (const ioport_entry_t *)(*(const ioport_entry_t **)pelem);
    const ioport_range_t *elem = &entry->range;
    if (key < elem->start) {
        return -1;
    }
    if (key > elem->end) {
        return 1;
    }
    return 0;
}

static int io_port_compare_by_start(const void *a, const void *b)
{
    const ioport_entry_t *a_entry = (const ioport_entry_t *)(*(const ioport_entry_t **)a);
    const ioport_range_t *a_range = &a_entry->range;
    const ioport_entry_t *b_entry = (const ioport_entry_t *)(*(const ioport_entry_t **)b);
    const ioport_range_t *b_range = &b_entry->range;
    return a_range->start - b_range->start;
}

static ioport_entry_t **search_port(vmm_io_port_list_t *io_port, unsigned int port_no)
{
    return (ioport_entry_t **)bsearch((void *)(uintptr_t)port_no, io_port->ioports, io_port->num_ioports,
                                      sizeof(ioport_entry_t *), io_port_compare_by_range);
}

/* Debug helper function for port no. */
static const char *vmm_debug_io_portno_desc(vmm_io_port_list_t *io_port, int port_no)
{
    ioport_entry_t **res_port = search_port(io_port, port_no);
    return res_port ? (*res_port)->interface.desc : "Unknown IO Port";
}

/* IO execution handler. */
int emulate_io_handler(vmm_io_port_list_t *io_port, unsigned int port_no, bool is_in, size_t size, unsigned int *data)
{
    if (io_port == NULL) {
        ZF_LOGE("Unable to emulate port - io port list is uninitalised");
        return -1;
    }

    ZF_LOGD("exit io request: in %d  port no 0x%x (%s) size %d",
            is_in, port_no, vmm_debug_io_portno_desc(io_port, port_no), size);

    ioport_entry_t **res_port = search_port(io_port, port_no);
    if (!res_port) {
        static int last_port = -1;
        if (last_port != port_no) {
            ZF_LOGW("exit io request: WARNING - ignoring unsupported ioport 0x%x (%s)", port_no,
                    vmm_debug_io_portno_desc(io_port, port_no));
            last_port = port_no;
        }
        return 1;
    }
    ioport_entry_t *port = *res_port;
    int ret = 0;
    if (is_in) {
        ret = port->interface.port_in(port->interface.cookie, port_no, size, data);
    } else {
        ret = port->interface.port_out(port->interface.cookie, port_no, size, *data);
    }

    if (ret) {
        ZF_LOGE("exit io request: handler returned error.");
        ZF_LOGE("exit io ERROR: string %d  in %d rep %d  port no 0x%x (%s) size %zd", 0,
                is_in, 0, port_no, vmm_debug_io_portno_desc(io_port, port_no), size);
        return -1;
    }

    return 0;
}

static int add_io_port_range(vmm_io_port_list_t *io_list, ioport_entry_t *port)
{
    if (io_list == NULL) {
        ZF_LOGE("Unable to add port - io port list is uninitalised");
        return -1;
    }
    /* ensure this range does not overlap */
    for (int i = 0; i < io_list->num_ioports; i++) {
        if (io_list->ioports[i]->range.end >= port->range.start && io_list->ioports[i]->range.start <= port->range.end) {
            ZF_LOGE("Requested ioport range 0x%x-0x%x for %s overlaps with existing range 0x%x-0x%x for %s",
                    port->range.start, port->range.end, port->interface.desc ? port->interface.desc : "Unknown IO Port",
                    io_list->ioports[i]->range.start, io_list->ioports[i]->range.end,
                    io_list->ioports[i]->interface.desc ? io_list->ioports[i]->interface.desc : "Unknown IO Port");
            return -1;
        }
    }
    /* grow the array */
    io_list->ioports = realloc(io_list->ioports, sizeof(ioport_entry_t *) * (io_list->num_ioports + 1));
    assert(io_list->ioports);
    /* add the new entry */
    io_list->ioports[io_list->num_ioports] = port;
    io_list->num_ioports++;
    /* sort */
    qsort(io_list->ioports, io_list->num_ioports, sizeof(ioport_entry_t *), io_port_compare_by_start);
    return 0;
}

static int alloc_free_io_port_range(vmm_io_port_list_t *io_list, ioport_range_t *io_range)
{
    uint16_t free_port_addr = io_list->alloc_addr;
    if (free_port_addr + io_range->size < free_port_addr) {
        /* Possible overflow */
        return -1;
    }
    io_list->alloc_addr += io_range->size;
    io_range->start = free_port_addr;
    io_range->end = free_port_addr + io_range->size - 1;
    return 0;
}

static void free_io_port_range(vmm_io_port_list_t *io_list, ioport_range_t *io_range)
{
    io_list->alloc_addr -= io_range->size;
}

/* Add an io port range for emulation */
ioport_entry_t *vmm_io_port_add_handler(vmm_io_port_list_t *io_list, ioport_range_t io_range,
                                        ioport_interface_t io_interface, ioport_type_t port_type)
{
    int err;
    if (port_type == IOPORT_FREE) {
        err = alloc_free_io_port_range(io_list, &io_range);
        if (err) {
            return NULL;
        }
    }
    ioport_entry_t *entry = calloc(1, sizeof(ioport_entry_t));
    if (!entry) {
        free_io_port_range(io_list, &io_range);
        return NULL;
    }
    *entry = (ioport_entry_t) {
        io_range, io_interface
    };
    err = add_io_port_range(io_list, entry);
    if (err) {
        free_io_port_range(io_list, &io_range);
        return NULL;
    }
    return entry;
}

int vmm_io_port_init(vmm_io_port_list_t **io_list, uint16_t ioport_alloc_addr)
{
    vmm_io_port_list_t *init_iolist = (vmm_io_port_list_t *)calloc(1, sizeof(vmm_io_port_list_t));
    if (init_iolist == NULL) {
        ZF_LOGE("Failed to calloc memory for io port list");
        return -1;
    }

    init_iolist->num_ioports = 0;
    init_iolist->ioports = NULL;
    init_iolist->alloc_addr = ioport_alloc_addr;
    *io_list = init_iolist;
    return 0;
}
