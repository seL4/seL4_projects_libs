/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <rpc.pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <sel4nanopb/sel4nanopb.h>
#include <sel4rpc/client.h>
#include <sel4rpc/server.h>

#include <utils/zf_log.h>

#define IPC_RESERVED_WORDS (1)

int sel4rpc_client_init(sel4rpc_client_t *client, seL4_CPtr server_ep, seL4_Word magic)
{
    client->server_ep = server_ep;
    client->magic = magic;
    return 0;
}

int sel4rpc_call(sel4rpc_client_t *client, RpcMessage *msg, seL4_CPtr root,
                 seL4_CPtr capPtr, seL4_Word capDepth)
{
    pb_ostream_t stream = pb_ostream_from_IPC(IPC_RESERVED_WORDS);
    bool ret = pb_encode_delimited(&stream, &RpcMessage_msg, msg);
    if (!ret) {
        ZF_LOGE("Failed to encode message (%s)", PB_GET_ERROR(&stream));
        return -1;
    }

    size_t stream_size = stream.bytes_written / sizeof(seL4_Word);
    /* add an extra word if bytes_written is not divisible by sizeof(seL4_Word). */
    if (stream.bytes_written % sizeof(seL4_Word)) {
        stream_size += 1;
    }

    /* add an extra word for the magic header */
    stream_size += IPC_RESERVED_WORDS;

    /* set cap receive path */
    seL4_SetCapReceivePath(root, capPtr, capDepth);
    /* set magic header */
    seL4_SetMR(0, client->magic);
    seL4_Call(client->server_ep, seL4_MessageInfo_new(0, 0, 0, stream_size));

    pb_istream_t istream = pb_istream_from_IPC(0);
    ret = pb_decode_delimited(&istream, &RpcMessage_msg, msg);
    if (!ret) {
        ZF_LOGE("Failed to decode server reply (%s)", PB_GET_ERROR(&stream));
        return -1;
    }

    return 0;
}
