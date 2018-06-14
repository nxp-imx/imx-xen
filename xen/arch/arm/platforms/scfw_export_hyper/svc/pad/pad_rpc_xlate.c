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
 * File containing hypervisor RPC functions for the PAD service.
 *
 * @addtogroup PAD_SVC
 * @{
 */

/* Includes */

#include "main/types.h"
#include "svc/pad/pad_api.h"
#include "../../main/rpc.h"
#include "svc/pad/pad_rpc.h"

/* Local Defines */

/* Local Types */

/*--------------------------------------------------------------------------*/
/* Translate and forward RPC call                                           */
/*--------------------------------------------------------------------------*/
void pad_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg)
{
    uint8_t func = RPC_FUNC(msg);
    sc_err_t err = SC_ERR_NONE;

    switch (func)
    {
        case PAD_FUNC_UNKNOWN :
            {
                RPC_SIZE(msg) = 1;
                RPC_R8(msg) = SC_ERR_NOTFOUND;
            }
            break;
        case PAD_FUNC_SET_MUX :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t mux = RPC_U8(msg, 2);
                uint8_t config = RPC_U8(msg, 3);
                uint8_t iso = RPC_U8(msg, 4);

                V2P_PAD(ipc, &pad);

                result = sc_pad_set_mux(ipc, pad, mux, config, iso);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PAD_FUNC_GET_MUX :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t mux = 0;
                uint8_t config = 0;
                uint8_t iso = 0;

                V2P_PAD(ipc, &pad);

                result = sc_pad_get_mux(ipc, pad, &mux, &config, &iso);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = mux;
                RPC_U8(msg, 1) = config;
                RPC_U8(msg, 2) = iso;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PAD_FUNC_SET_GP :
            {
                uint8_t result;
                uint32_t ctrl = RPC_U32(msg, 0);
                uint16_t pad = RPC_U16(msg, 4);

                V2P_PAD(ipc, &pad);

                result = sc_pad_set_gp(ipc, pad, ctrl);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PAD_FUNC_GET_GP :
            {
                uint8_t result;
                uint32_t ctrl = 0;
                uint16_t pad = RPC_U16(msg, 0);

                V2P_PAD(ipc, &pad);

                result = sc_pad_get_gp(ipc, pad, &ctrl);

                RPC_U32(msg, 0) = ctrl;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PAD_FUNC_SET_WAKEUP :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t wakeup = RPC_U8(msg, 2);

                V2P_PAD(ipc, &pad);

                result = sc_pad_set_wakeup(ipc, pad, wakeup);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PAD_FUNC_GET_WAKEUP :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t wakeup = 0;

                V2P_PAD(ipc, &pad);

                result = sc_pad_get_wakeup(ipc, pad, &wakeup);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = wakeup;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PAD_FUNC_SET_ALL :
            {
                uint8_t result;
                uint32_t ctrl = RPC_U32(msg, 0);
                uint16_t pad = RPC_U16(msg, 4);
                uint8_t mux = RPC_U8(msg, 6);
                uint8_t config = RPC_U8(msg, 7);
                uint8_t iso = RPC_U8(msg, 8);
                uint8_t wakeup = RPC_U8(msg, 9);

                V2P_PAD(ipc, &pad);

                result = sc_pad_set_all(ipc, pad, mux, config, iso, ctrl, wakeup);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PAD_FUNC_GET_ALL :
            {
                uint8_t result;
                uint32_t ctrl = 0;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t mux = 0;
                uint8_t config = 0;
                uint8_t iso = 0;
                uint8_t wakeup = 0;

                V2P_PAD(ipc, &pad);

                result = sc_pad_get_all(ipc, pad, &mux, &config, &iso, &ctrl, &wakeup);

                RPC_U32(msg, 0) = ctrl;
                RPC_R8(msg) = result;
                RPC_U8(msg, 4) = mux;
                RPC_U8(msg, 5) = config;
                RPC_U8(msg, 6) = iso;
                RPC_U8(msg, 7) = wakeup;
                RPC_SIZE(msg) = 3;
            }
            break;
        case PAD_FUNC_SET :
            {
                uint8_t result;
                uint32_t val = RPC_U32(msg, 0);
                uint16_t pad = RPC_U16(msg, 4);

                V2P_PAD(ipc, &pad);

                result = sc_pad_set(ipc, pad, val);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PAD_FUNC_GET :
            {
                uint8_t result;
                uint32_t val = 0;
                uint16_t pad = RPC_U16(msg, 0);

                V2P_PAD(ipc, &pad);

                result = sc_pad_get(ipc, pad, &val);

                RPC_U32(msg, 0) = val;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PAD_FUNC_SET_GP_28FDSOI :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t dse = RPC_U8(msg, 2);
                uint8_t ps = RPC_U8(msg, 3);

                V2P_PAD(ipc, &pad);

                result = sc_pad_set_gp_28fdsoi(ipc, pad, dse, ps);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PAD_FUNC_GET_GP_28FDSOI :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t dse = 0;
                uint8_t ps = 0;

                V2P_PAD(ipc, &pad);

                result = sc_pad_get_gp_28fdsoi(ipc, pad, &dse, &ps);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = dse;
                RPC_U8(msg, 1) = ps;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PAD_FUNC_SET_GP_28FDSOI_HSIC :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t dse = RPC_U8(msg, 2);
                uint8_t pus = RPC_U8(msg, 3);
                sc_bool_t hys = RPC_U8(msg, 4);
                sc_bool_t pke = RPC_U8(msg, 5);
                sc_bool_t pue = RPC_U8(msg, 6);

                V2P_PAD(ipc, &pad);

                result = sc_pad_set_gp_28fdsoi_hsic(ipc, pad, dse, hys, pus, pke, pue);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PAD_FUNC_GET_GP_28FDSOI_HSIC :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t dse = 0;
                uint8_t pus = 0;
                sc_bool_t hys = 0;
                sc_bool_t pke = 0;
                sc_bool_t pue = 0;

                V2P_PAD(ipc, &pad);

                result = sc_pad_get_gp_28fdsoi_hsic(ipc, pad, &dse, &hys, &pus, &pke, &pue);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = dse;
                RPC_U8(msg, 1) = pus;
                RPC_U8(msg, 2) = hys;
                RPC_U8(msg, 3) = pke;
                RPC_U8(msg, 4) = pue;
                RPC_SIZE(msg) = 3;
            }
            break;
        case PAD_FUNC_SET_GP_28FDSOI_COMP :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t compen = RPC_U8(msg, 2);
                uint8_t rasrcp = RPC_U8(msg, 3);
                uint8_t rasrcn = RPC_U8(msg, 4);
                sc_bool_t fastfrz = RPC_U8(msg, 5);
                sc_bool_t nasrc_sel = RPC_U8(msg, 6);
                sc_bool_t psw_ovr = RPC_U8(msg, 7);

                V2P_PAD(ipc, &pad);

                result = sc_pad_set_gp_28fdsoi_comp(ipc, pad, compen, fastfrz, rasrcp, rasrcn, nasrc_sel, psw_ovr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PAD_FUNC_GET_GP_28FDSOI_COMP :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t compen = 0;
                uint8_t rasrcp = 0;
                uint8_t rasrcn = 0;
                uint8_t nasrc = 0;
                sc_bool_t fastfrz = 0;
                sc_bool_t nasrc_sel = 0;
                sc_bool_t compok = 0;
                sc_bool_t psw_ovr = 0;

                V2P_PAD(ipc, &pad);

                result = sc_pad_get_gp_28fdsoi_comp(ipc, pad, &compen, &fastfrz, &rasrcp, &rasrcn, &nasrc_sel, &compok, &nasrc, &psw_ovr);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = compen;
                RPC_U8(msg, 1) = rasrcp;
                RPC_U8(msg, 2) = rasrcn;
                RPC_U8(msg, 3) = nasrc;
                RPC_U8(msg, 4) = fastfrz;
                RPC_U8(msg, 5) = nasrc_sel;
                RPC_U8(msg, 6) = compok;
                RPC_U8(msg, 7) = psw_ovr;
                RPC_SIZE(msg) = 3;
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

