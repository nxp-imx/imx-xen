/*
** ###################################################################
**
**     Copyright (c) 2016 Freescale Semiconductor, Inc.
**     Copyright 2017-2018 NXP
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** ###################################################################
*/

/*==========================================================================*/
/*!
 * @file
 *
 * Header file for the RPC implementation.
 */
/*==========================================================================*/

#ifndef SC_RPC_H
#define SC_RPC_H

/* Includes */

#include "main/types.h"
#include "main/ipc.h"

/* Defines */

#define SC_RPC_VERSION          1U

#define SC_RPC_MAX_MSG          8U

#define RPC_VER(MSG)            ((MSG)->version)
#define RPC_SIZE(MSG)           ((MSG)->size)
#define RPC_SVC(MSG)            ((MSG)->svc)
#define RPC_FUNC(MSG)           ((MSG)->func)
#define RPC_R8(MSG)             ((MSG)->func)
#define RPC_I32(MSG, IDX)       ((MSG)->DATA.i32[(IDX) / 4U])
#define RPC_I16(MSG, IDX)       ((MSG)->DATA.i16[(IDX) / 2U])
#define RPC_I8(MSG, IDX)        ((MSG)->DATA.i8[(IDX)])
#define RPC_U32(MSG, IDX)       ((MSG)->DATA.u32[(IDX) / 4U])
#define RPC_U16(MSG, IDX)       ((MSG)->DATA.u16[(IDX) / 2U])
#define RPC_U8(MSG, IDX)        ((MSG)->DATA.u8[(IDX)])

#define SC_RPC_SVC_UNKNOWN      0U
#define SC_RPC_SVC_RETURN       1U
#define SC_RPC_SVC_PM           2U
#define SC_RPC_SVC_RM           3U
#define SC_RPC_SVC_TIMER        5U
#define SC_RPC_SVC_PAD          6U
#define SC_RPC_SVC_MISC         7U
#define SC_RPC_SVC_IRQ          8U
#define SC_RPC_SVC_ABORT        9U

#define SC_RPC_ASYNC_STATE_RD_START      0U
#define SC_RPC_ASYNC_STATE_RD_ACTIVE     1U
#define SC_RPC_ASYNC_STATE_RD_DONE       2U
#define SC_RPC_ASYNC_STATE_WR_START      3U
#define SC_RPC_ASYNC_STATE_WR_ACTIVE     4U
#define SC_RPC_ASYNC_STATE_WR_DONE       5U

#define SC_RPC_MU_GIR_SVC       0x1U
#define SC_RPC_MU_GIR_DBG       0x8U

/* Types */

typedef uint8_t sc_rpc_svc_t;

typedef struct sc_rpc_msg_s
{
    uint8_t version;
    uint8_t size;
    uint8_t svc;
    uint8_t func;
    union
    {
        int32_t i32[(SC_RPC_MAX_MSG - 1U)];
        int16_t i16[(SC_RPC_MAX_MSG - 1U) * 2U];
        int8_t i8[(SC_RPC_MAX_MSG - 1U) * 4U];
        uint32_t u32[(SC_RPC_MAX_MSG - 1U)];
        uint16_t u16[(SC_RPC_MAX_MSG - 1U) * 2U];
        uint8_t u8[(SC_RPC_MAX_MSG - 1U) * 4U];
    } DATA;
} sc_rpc_msg_t;

typedef uint8_t sc_rpc_async_state_t;

typedef struct sc_rpc_async_msg_s
{
    sc_rpc_async_state_t state;
    uint8_t wordIdx;
    sc_rpc_msg_t msg;
    uint32_t timeStamp;
} sc_rpc_async_msg_t;

/* Functions */

/*!
 * This is an internal function to send an RPC message over an IPC
 * channel. It is called by client-side SCFW API function shims.
 *
 * @param[in]     ipc         IPC handle
 * @param[in,out] msg         handle to a message
 * @param[in]     no_resp     response flag
 *
 * If \a no_resp is SC_FALSE then this function waits for a response
 * and returns the result in \a msg.
 */
void sc_call_rpc(sc_ipc_t ipc, sc_rpc_msg_t *msg, sc_bool_t no_resp);

/*!
 * This is an internal function to dispath an RPC call that has
 * arrived via IPC over an MU. It is called by server-side SCFW.
 *
 * @param[in]     mu          MU message arrived on
 * @param[in,out] msg         handle to a message
 *
 * The function result is returned in \a msg.
 */
void sc_rpc_dispatch(sc_rsrc_t mu, sc_rpc_msg_t *msg);

/*!
 * This function translates an RPC message and forwards on to the
 * normal RPC API.  It is used only by hypervisors.
 *
 * @param[in]     ipc         IPC handle
 * @param[in,out] msg         handle to a message
 *
 * This function decodes a message, calls macros to translate the
 * resources, pads, addresses, partitions, memory regions, etc. and
 * then forwards on to the hypervisors SCFW API.Return results are
 * translated back abd placed back into the message to be returned
 * to the original API.
 */
void sc_rpc_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg);

#endif /* SC_RPC_H */

