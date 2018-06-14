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
 * File containing hypervisor RPC functions for the MISC service.
 *
 * @addtogroup MISC_SVC
 * @{
 */

/* Includes */

#include "main/types.h"
#include "svc/misc/misc_api.h"
#include "../../main/rpc.h"
#include "svc/misc/misc_rpc.h"

/* Local Defines */

/* Local Types */

/*--------------------------------------------------------------------------*/
/* Translate and forward RPC call                                           */
/*--------------------------------------------------------------------------*/
void misc_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg)
{
    uint8_t func = RPC_FUNC(msg);
    sc_err_t err = SC_ERR_NONE;

    switch (func)
    {
        case MISC_FUNC_UNKNOWN :
            {
                RPC_SIZE(msg) = 1;
                RPC_R8(msg) = SC_ERR_NOTFOUND;
            }
            break;
        case MISC_FUNC_SET_CONTROL :
            {
                uint8_t result;
                uint32_t ctrl = RPC_U32(msg, 0);
                uint32_t val = RPC_U32(msg, 4);
                uint16_t resource = RPC_U16(msg, 8);

                V2P_RESOURCE(ipc, &resource);

                result = sc_misc_set_control(ipc, resource, ctrl, val);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_GET_CONTROL :
            {
                uint8_t result;
                uint32_t ctrl = RPC_U32(msg, 0);
                uint32_t val = 0;
                uint16_t resource = RPC_U16(msg, 4);

                V2P_RESOURCE(ipc, &resource);

                result = sc_misc_get_control(ipc, resource, ctrl, &val);

                RPC_U32(msg, 0) = val;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        case MISC_FUNC_SET_MAX_DMA_GROUP :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);
                uint8_t max = RPC_U8(msg, 1);

                V2P_PT(ipc, &pt);

                result = sc_misc_set_max_dma_group(ipc, pt, max);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_SET_DMA_GROUP :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t group = RPC_U8(msg, 2);

                V2P_RESOURCE(ipc, &resource);

                result = sc_misc_set_dma_group(ipc, resource, group);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_SECO_IMAGE_LOAD :
            {
                uint8_t result;
                uint64_t addr_src = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint64_t addr_dst = ((uint64_t) RPC_U32(msg, 8) << 32) | RPC_U32(msg, 12);
                uint32_t len = RPC_U32(msg, 16);
                sc_bool_t fw = RPC_U8(msg, 20);

                V2P_ADDR(ipc, &addr_src);
                V2P_ADDR(ipc, &addr_dst);

                result = sc_misc_seco_image_load(ipc, addr_src, addr_dst, len, fw);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_SECO_AUTHENTICATE :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint8_t cmd = RPC_U8(msg, 8);

                V2P_ADDR(ipc, &addr);

                result = sc_misc_seco_authenticate(ipc, cmd, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_SECO_FUSE_WRITE :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                V2P_ADDR(ipc, &addr);

                result = sc_misc_seco_fuse_write(ipc, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_SECO_ENABLE_DEBUG :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                V2P_ADDR(ipc, &addr);

                result = sc_misc_seco_enable_debug(ipc, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_SECO_FORWARD_LIFECYCLE :
            {
                uint8_t result;
                uint32_t lifecycle = RPC_U32(msg, 0);

                result = sc_misc_seco_forward_lifecycle(ipc, lifecycle);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_SECO_RETURN_LIFECYCLE :
            {
                uint8_t result;
                uint64_t addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                V2P_ADDR(ipc, &addr);

                result = sc_misc_seco_return_lifecycle(ipc, addr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_SECO_BUILD_INFO :
            {
                uint32_t version = 0;
                uint32_t commit = 0;

                misc_seco_build_info(ipc, &version, &commit);

                RPC_U32(msg, 0) = version;
                RPC_U32(msg, 4) = commit;
                RPC_SIZE(msg) = 3;
            }
            break;
        case MISC_FUNC_DEBUG_OUT :
            {
                uint8_t ch = RPC_U8(msg, 0);

                misc_debug_out(ipc, ch);

                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_WAVEFORM_CAPTURE :
            {
                uint8_t result;
                sc_bool_t enable = RPC_U8(msg, 0);

                result = sc_misc_waveform_capture(ipc, enable);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_BUILD_INFO :
            {
                uint32_t build = 0;
                uint32_t commit = 0;

                misc_build_info(ipc, &build, &commit);

                RPC_U32(msg, 0) = build;
                RPC_U32(msg, 4) = commit;
                RPC_SIZE(msg) = 3;
            }
            break;
        case MISC_FUNC_UNIQUE_ID :
            {
                uint32_t id_l = 0;
                uint32_t id_h = 0;

                misc_unique_id(ipc, &id_l, &id_h);

                RPC_U32(msg, 0) = id_l;
                RPC_U32(msg, 4) = id_h;
                RPC_SIZE(msg) = 3;
            }
            break;
        case MISC_FUNC_SET_ARI :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint16_t resource_mst = RPC_U16(msg, 2);
                uint16_t ari = RPC_U16(msg, 4);
                sc_bool_t enable = RPC_U8(msg, 6);

                V2P_RESOURCE(ipc, &resource);
                V2P_RESOURCE(ipc, &resource_mst);

                result = sc_misc_set_ari(ipc, resource, resource_mst, ari, enable);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_BOOT_STATUS :
            {
                uint8_t status = RPC_U8(msg, 0);

                misc_boot_status(ipc, status);

                RPC_SIZE(msg) = 0;
            }
            break;
        case MISC_FUNC_BOOT_DONE :
            {
                uint8_t result;
                uint16_t cpu = RPC_U16(msg, 0);

                result = sc_misc_boot_done(ipc, cpu);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_OTP_FUSE_READ :
            {
                uint8_t result;
                uint32_t word = RPC_U32(msg, 0);
                uint32_t val = 0;

                result = sc_misc_otp_fuse_read(ipc, word, &val);

                RPC_U32(msg, 0) = val;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        case MISC_FUNC_OTP_FUSE_WRITE :
            {
                uint8_t result;
                uint32_t word = RPC_U32(msg, 0);
                uint32_t val = RPC_U32(msg, 4);

                result = sc_misc_otp_fuse_write(ipc, word, val);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_SET_TEMP :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                int16_t celsius = RPC_I16(msg, 2);
                uint8_t temp = RPC_U8(msg, 4);
                int8_t tenths = RPC_I8(msg, 5);

                V2P_RESOURCE(ipc, &resource);

                result = sc_misc_set_temp(ipc, resource, temp, celsius, tenths);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case MISC_FUNC_GET_TEMP :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                int16_t celsius = 0;
                uint8_t temp = RPC_U8(msg, 2);
                int8_t tenths = 0;

                V2P_RESOURCE(ipc, &resource);

                result = sc_misc_get_temp(ipc, resource, temp, &celsius, &tenths);

                RPC_I16(msg, 0) = celsius;
                RPC_R8(msg) = result;
                RPC_I8(msg, 2) = tenths;
                RPC_SIZE(msg) = 2;
            }
            break;
        case MISC_FUNC_GET_BOOT_DEV :
            {
                uint16_t dev = 0;

                misc_get_boot_dev(ipc, &dev);

                RPC_U16(msg, 0) = dev;
                RPC_SIZE(msg) = 2;
            }
            break;
        case MISC_FUNC_GET_BUTTON_STATUS :
            {
                sc_bool_t status = 0;

                misc_get_button_status(ipc, &status);

                RPC_U8(msg, 0) = status;
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

