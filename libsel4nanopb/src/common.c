/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <stdio.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <utils/fence.h>
#include <sel4nanopb/sel4nanopb.h>
#include <sel4/sel4.h>

pb_ostream_t pb_ostream_from_IPC(seL4_Word offset)
{
    pb_byte_t *msg_buffer = (pb_byte_t *) & (seL4_GetIPCBuffer()->msg[offset]);
    size_t size = seL4_MsgMaxLength * sizeof(seL4_Word) - offset;
    return pb_ostream_from_buffer(msg_buffer, size);
}

pb_istream_t pb_istream_from_IPC(seL4_Word offset)
{
    pb_byte_t *msg_buffer = (pb_byte_t *) & (seL4_GetIPCBuffer()->msg[offset]);
    size_t size = seL4_MsgMaxLength * sizeof(seL4_Word) - offset;
    return pb_istream_from_buffer(msg_buffer, size);
}
