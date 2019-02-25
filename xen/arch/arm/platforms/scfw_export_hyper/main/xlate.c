/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * Implementation of the xlate dispatch function.
 */

/* Includes */

#include "main/scfw.h"
#include "main/rpc.h"
#include "svc/pm/pm_rpc.h"
#include "svc/rm/rm_rpc.h"
#include "svc/timer/timer_rpc.h"
#include "svc/pad/pad_rpc.h"
#include "svc/misc/misc_rpc.h"
#include "svc/irq/irq_rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

/* Local Variables */

/*--------------------------------------------------------------------------*/
/* Dispatch xlate call                                                      */
/*--------------------------------------------------------------------------*/
void sc_rpc_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg)
{
    /* Check request size */
    if (RPC_SIZE(msg) == 0)
    {
        RPC_VER(msg) = SC_RPC_VERSION;
        RPC_SIZE(msg) = 1U;
        RPC_SVC(msg) = SC_RPC_SVC_RETURN;
        RPC_R8(msg) = SC_ERR_IPC;

        return;
    }

    /* Check request API version */
    if (RPC_VER(msg) != SC_RPC_VERSION)
    {
        RPC_VER(msg) = SC_RPC_VERSION;
        RPC_SIZE(msg) = 1U;
        RPC_SVC(msg) = SC_RPC_SVC_RETURN;
        RPC_R8(msg) = SC_ERR_VERSION;

        return;
    }

    /* Call requested service xlate */
    switch (RPC_SVC(msg))
    {
        case SC_RPC_SVC_PM :
            pm_xlate(ipc, msg);
            return;
        case SC_RPC_SVC_RM :
            rm_xlate(ipc, msg);
            return;
        case SC_RPC_SVC_TIMER :
            timer_xlate(ipc, msg);
            return;
        case SC_RPC_SVC_PAD :
            pad_xlate(ipc, msg);
            return;
        case SC_RPC_SVC_MISC :
            misc_xlate(ipc, msg);
            return;
        case SC_RPC_SVC_IRQ :
            irq_xlate(ipc, msg);
            return;
        default :
            ; /* Intentional empty default */
            break;
    }

    /* Service not found */
    RPC_VER(msg) = SC_RPC_VERSION;
    RPC_SIZE(msg) = 1U;
    RPC_SVC(msg) = SC_RPC_SVC_RETURN;
    RPC_R8(msg) = SC_ERR_NOTFOUND;
}

