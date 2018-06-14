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
 * File containing hypervisor RPC functions for the IRQ service.
 *
 * @addtogroup IRQ_SVC
 * @{
 */

/* Includes */

#include "main/types.h"
#include "svc/irq/irq_api.h"
#include "../../main/rpc.h"
#include "svc/irq/irq_rpc.h"

/* Local Defines */

/* Local Types */

/*--------------------------------------------------------------------------*/
/* Translate and forward RPC call                                           */
/*--------------------------------------------------------------------------*/
void irq_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg)
{
    uint8_t func = RPC_FUNC(msg);
    sc_err_t err = SC_ERR_NONE;

    switch (func)
    {
        case IRQ_FUNC_UNKNOWN :
            {
                RPC_SIZE(msg) = 1;
                RPC_R8(msg) = SC_ERR_NOTFOUND;
            }
            break;
        case IRQ_FUNC_ENABLE :
            {
                uint8_t result;
                uint32_t mask = RPC_U32(msg, 0);
                uint16_t resource = RPC_U16(msg, 4);
                uint8_t group = RPC_U8(msg, 6);
                sc_bool_t enable = RPC_U8(msg, 7);

                V2P_RESOURCE(ipc, &resource);

                result = sc_irq_enable(ipc, resource, group, mask, enable);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case IRQ_FUNC_STATUS :
            {
                uint8_t result;
                uint32_t status = 0;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t group = RPC_U8(msg, 2);

                V2P_RESOURCE(ipc, &resource);

                result = sc_irq_status(ipc, resource, group, &status);

                RPC_U32(msg, 0) = status;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        default :
            {
                RPC_SIZE(msg) = 1;
                RPC_R8(msg) = SC_ERR_NOTFOUND;
            }
            break;
    }

    RPC_VER(msg) = SC_RPC_VERSION;
    RPC_SVC(msg) = SC_RPC_SVC_RETURN;
}

/**@}*/

