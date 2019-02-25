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
 * File containing hypervisor RPC functions for the SECO service.
 *
 * @addtogroup SECO_SVC
 * @{
 */

/* Includes */

#include "main/types.h"
#include "svc/seco/seco_api.h"
#include "../../main/rpc.h"
#include "svc/seco/seco_rpc.h"

/* Local Defines */

/* Local Types */

/*--------------------------------------------------------------------------*/
/* Translate and forward RPC call                                           */
/*--------------------------------------------------------------------------*/
void seco_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg)
{
    uint8_t func = RPC_FUNC(msg);
    sc_err_t err = SC_ERR_NONE;

    switch (func)
    {
        case SECO_FUNC_UNKNOWN :
            {
                RPC_SIZE(msg) = 1;
                RPC_R8(msg) = SC_ERR_NOTFOUND;
            }
            break;
        case SECO_FUNC_IMAGE_LOAD :
            {
                uint8_t result;
                uint64_t addr_src = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint64_t addr_dst = ((uint64_t) RPC_U32(msg, 8) << 32) | RPC_U32(msg, 12);
                uint32_t len = RPC_U32(msg, 16);
                sc_bool_t fw = RPC_U8(msg, 20);

                V2P_ADDR(ipc, &addr_src);
                V2P_ADDR(ipc, &addr_dst);

                result = sc_seco_image_load(ipc, addr_src, addr_dst, len, fw);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_AUTHENTICATE :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint8_t cmd = RPC_U8(msg, 8);

                V2P_ADDR(ipc, &addr);

                result = sc_seco_authenticate(ipc, cmd, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_FORWARD_LIFECYCLE :
            {
                uint8_t result;
                uint32_t change = RPC_U32(msg, 0);

                result = sc_seco_forward_lifecycle(ipc, change);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_RETURN_LIFECYCLE :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                V2P_ADDR(ipc, &addr);

                result = sc_seco_return_lifecycle(ipc, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_COMMIT :
            {
                uint8_t result;
                uint32_t info = RPC_U32(msg, 0);

                result = sc_seco_commit(ipc, &info);

                RPC_U32(msg, 0) = info;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        case SECO_FUNC_ATTEST_MODE :
            {
                uint8_t result;
                uint32_t mode = RPC_U32(msg, 0);

                result = sc_seco_attest_mode(ipc, mode);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_ATTEST :
            {
                uint8_t result;
                uint64_t nonce = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                result = sc_seco_attest(ipc, nonce);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_GET_ATTEST_PKEY :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                V2P_ADDR(ipc, &addr);

                result = sc_seco_get_attest_pkey(ipc, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_GET_ATTEST_SIGN :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                V2P_ADDR(ipc, &addr);

                result = sc_seco_get_attest_sign(ipc, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_ATTEST_VERIFY :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                V2P_ADDR(ipc, &addr);

                result = sc_seco_attest_verify(ipc, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_GEN_KEY_BLOB :
            {
                uint8_t result;
                uint64_t load_addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint64_t export_addr = ((uint64_t) RPC_U32(msg, 8) << 32) | RPC_U32(msg, 12);
                uint32_t id = RPC_U32(msg, 16);
                uint16_t max_size = RPC_U16(msg, 20);

                result = sc_seco_gen_key_blob(ipc, id, load_addr, export_addr, max_size);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_LOAD_KEY :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint32_t id = RPC_U32(msg, 8);

                V2P_ADDR(ipc, &addr);

                result = sc_seco_load_key(ipc, id, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_GET_MP_KEY :
            {
                uint8_t result;
                uint64_t dst_addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint16_t dst_size = RPC_U16(msg, 8);

                result = sc_seco_get_mp_key(ipc, dst_addr, dst_size);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_UPDATE_MPMR :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint8_t size = RPC_U8(msg, 8);
                uint8_t lock = RPC_U8(msg, 9);

                V2P_ADDR(ipc, &addr);

                result = sc_seco_update_mpmr(ipc, addr, size, lock);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_GET_MP_SIGN :
            {
                uint8_t result;
                uint64_t msg_addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint64_t dst_addr = ((uint64_t) RPC_U32(msg, 8) << 32) | RPC_U32(msg, 12);
                uint16_t msg_size = RPC_U16(msg, 16);
                uint16_t dst_size = RPC_U16(msg, 18);

                result = sc_seco_get_mp_sign(ipc, msg_addr, msg_size, dst_addr, dst_size);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_BUILD_INFO :
            {
                uint32_t version = 0;
                uint32_t commit = 0;

                seco_build_info(ipc, &version, &commit);

                RPC_U32(msg, 0) = version;
                RPC_U32(msg, 4) = commit;
                RPC_SIZE(msg) = 3;
            }
            break;
        case SECO_FUNC_CHIP_INFO :
            {
                uint8_t result;
                uint32_t uid_l = 0;
                uint32_t uid_h = 0;
                uint16_t lc = 0;
                uint16_t monotonic = 0;

                result = sc_seco_chip_info(ipc, &lc, &monotonic, &uid_l, &uid_h);

                RPC_U32(msg, 0) = uid_l;
                RPC_U32(msg, 4) = uid_h;
                RPC_U16(msg, 8) = lc;
                RPC_U16(msg, 10) = monotonic;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 4;
            }
            break;
        case SECO_FUNC_ENABLE_DEBUG :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                V2P_ADDR(ipc, &addr);

                result = sc_seco_enable_debug(ipc, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case SECO_FUNC_GET_EVENT :
            {
                uint8_t result;
                uint32_t event = 0;
                uint8_t idx = RPC_U8(msg, 0);

                result = sc_seco_get_event(ipc, idx, &event);

                RPC_U32(msg, 0) = event;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        case SECO_FUNC_FUSE_WRITE :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                V2P_ADDR(ipc, &addr);

                result = sc_seco_fuse_write(ipc, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
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

