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
 * File containing hypervisor RPC functions for the RM service.
 *
 * @addtogroup RM_SVC
 * @{
 */

/* Includes */

#include "main/types.h"
#include "svc/rm/rm_api.h"
#include "../../main/rpc.h"
#include "svc/rm/rm_rpc.h"

/* Local Defines */

/* Local Types */

/*--------------------------------------------------------------------------*/
/* Translate and forward RPC call                                           */
/*--------------------------------------------------------------------------*/
void rm_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg)
{
    uint8_t func = RPC_FUNC(msg);
    sc_err_t err = SC_ERR_NONE;

    switch (func)
    {
        case RM_FUNC_UNKNOWN :
            {
                RPC_SIZE(msg) = 1;
                RPC_R8(msg) = SC_ERR_NOTFOUND;
            }
            break;
        case RM_FUNC_PARTITION_ALLOC :
            {
                uint8_t result;
                uint8_t pt = 0;
                sc_bool_t secure = RPC_U8(msg, 0);
                sc_bool_t isolated = RPC_U8(msg, 1);
                sc_bool_t restricted = RPC_U8(msg, 2);
                sc_bool_t grant = RPC_U8(msg, 3);
                sc_bool_t coherent = RPC_U8(msg, 4);

                result = sc_rm_partition_alloc(ipc, &pt, secure, isolated, restricted, grant, coherent);

                P2V_PT(ipc, &pt);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = pt;
                RPC_SIZE(msg) = 2;
            }
            break;
        case RM_FUNC_SET_CONFIDENTIAL :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);
                sc_bool_t retro = RPC_U8(msg, 1);

                V2P_PT(ipc, &pt);

                result = sc_rm_set_confidential(ipc, pt, retro);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_PARTITION_FREE :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);

                V2P_PT(ipc, &pt);

                result = sc_rm_partition_free(ipc, pt);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_GET_DID :
            {
                uint8_t result;

                result = sc_rm_get_did(ipc);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_PARTITION_STATIC :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);
                uint8_t did = RPC_U8(msg, 1);

                V2P_PT(ipc, &pt);
                V2P_DID(ipc, &did);

                result = sc_rm_partition_static(ipc, pt, did);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_PARTITION_LOCK :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);

                V2P_PT(ipc, &pt);

                result = sc_rm_partition_lock(ipc, pt);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_GET_PARTITION :
            {
                uint8_t result;
                uint8_t pt = 0;

                result = sc_rm_get_partition(ipc, &pt);

                P2V_PT(ipc, &pt);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = pt;
                RPC_SIZE(msg) = 2;
            }
            break;
        case RM_FUNC_SET_PARENT :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);
                uint8_t pt_parent = RPC_U8(msg, 1);

                V2P_PT(ipc, &pt);
                V2P_PT(ipc, &pt_parent);

                result = sc_rm_set_parent(ipc, pt, pt_parent);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_MOVE_ALL :
            {
                uint8_t result;
                uint8_t pt_src = RPC_U8(msg, 0);
                uint8_t pt_dst = RPC_U8(msg, 1);
                sc_bool_t move_rsrc = RPC_U8(msg, 2);
                sc_bool_t move_pads = RPC_U8(msg, 3);

                V2P_PT(ipc, &pt_src);
                V2P_PT(ipc, &pt_dst);

                result = sc_rm_move_all(ipc, pt_src, pt_dst, move_rsrc, move_pads);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_ASSIGN_RESOURCE :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t pt = RPC_U8(msg, 2);

                V2P_PT(ipc, &pt);
                V2P_RESOURCE(ipc, &resource);

                result = sc_rm_assign_resource(ipc, pt, resource);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_SET_RESOURCE_MOVABLE :
            {
                uint8_t result;
                uint16_t resource_fst = RPC_U16(msg, 0);
                uint16_t resource_lst = RPC_U16(msg, 2);
                sc_bool_t movable = RPC_U8(msg, 4);

                V2P_RESOURCE(ipc, &resource_fst);
                V2P_RESOURCE(ipc, &resource_lst);

                result = sc_rm_set_resource_movable(ipc, resource_fst, resource_lst, movable);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_SET_SUBSYS_RSRC_MOVABLE :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                sc_bool_t movable = RPC_U8(msg, 2);

                V2P_RESOURCE(ipc, &resource);

                result = sc_rm_set_subsys_rsrc_movable(ipc, resource, movable);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_SET_MASTER_ATTRIBUTES :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t sa = RPC_U8(msg, 2);
                uint8_t pa = RPC_U8(msg, 3);
                sc_bool_t smmu_bypass = RPC_U8(msg, 4);

                V2P_RESOURCE(ipc, &resource);

                result = sc_rm_set_master_attributes(ipc, resource, sa, pa, smmu_bypass);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_SET_MASTER_SID :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint16_t sid = RPC_U16(msg, 2);

                V2P_RESOURCE(ipc, &resource);
                V2P_SID(ipc, &sid);

                result = sc_rm_set_master_sid(ipc, resource, sid);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_SET_PERIPHERAL_PERMISSIONS :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t pt = RPC_U8(msg, 2);
                uint8_t perm = RPC_U8(msg, 3);

                V2P_RESOURCE(ipc, &resource);
                V2P_PT(ipc, &pt);

                result = sc_rm_set_peripheral_permissions(ipc, resource, pt, perm);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_IS_RESOURCE_OWNED :
            {
                int8_t result;
                uint16_t resource = RPC_U16(msg, 0);

                V2P_RESOURCE(ipc, &resource);

                result = sc_rm_is_resource_owned(ipc, resource);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_GET_RESOURCE_OWNER :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t pt = 0;

                V2P_RESOURCE(ipc, &resource);

                result = sc_rm_get_resource_owner(ipc, resource, &pt);

                P2V_PT(ipc, &pt);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = pt;
                RPC_SIZE(msg) = 2;
            }
            break;
        case RM_FUNC_IS_RESOURCE_MASTER :
            {
                int8_t result;
                uint16_t resource = RPC_U16(msg, 0);

                V2P_RESOURCE(ipc, &resource);

                result = sc_rm_is_resource_master(ipc, resource);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_IS_RESOURCE_PERIPHERAL :
            {
                int8_t result;
                uint16_t resource = RPC_U16(msg, 0);

                V2P_RESOURCE(ipc, &resource);

                result = sc_rm_is_resource_peripheral(ipc, resource);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_GET_RESOURCE_INFO :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint16_t sid = 0;

                V2P_RESOURCE(ipc, &resource);

                result = sc_rm_get_resource_info(ipc, resource, &sid);

                P2V_SID(ipc, &sid);

                RPC_U16(msg, 0) = sid;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        case RM_FUNC_MEMREG_ALLOC :
            {
                uint8_t result;
                uint64_t addr_start = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint64_t addr_end = ((uint64_t) RPC_U32(msg, 8) << 32) | RPC_U32(msg, 12);
                uint8_t mr = 0;

                V2P_ADDR(ipc, &addr_start);
                V2P_ADDR(ipc, &addr_end);

                result = sc_rm_memreg_alloc(ipc, &mr, addr_start, addr_end);

                P2V_MR(ipc, &mr);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = mr;
                RPC_SIZE(msg) = 2;
            }
            break;
        case RM_FUNC_MEMREG_SPLIT :
            {
                uint8_t result;
                uint64_t addr_start = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint64_t addr_end = ((uint64_t) RPC_U32(msg, 8) << 32) | RPC_U32(msg, 12);
                uint8_t mr = RPC_U8(msg, 16);
                uint8_t mr_ret = 0;

                V2P_MR(ipc, &mr);
                V2P_ADDR(ipc, &addr_start);
                V2P_ADDR(ipc, &addr_end);

                result = sc_rm_memreg_split(ipc, mr, &mr_ret, addr_start, addr_end);

                P2V_MR(ipc, &mr_ret);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = mr_ret;
                RPC_SIZE(msg) = 2;
            }
            break;
        case RM_FUNC_MEMREG_FRAG :
            {
                uint8_t result;
                uint64_t addr_start = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint64_t addr_end = ((uint64_t) RPC_U32(msg, 8) << 32) | RPC_U32(msg, 12);
                uint8_t mr_ret = 0;

                V2P_ADDR(ipc, &addr_start);
                V2P_ADDR(ipc, &addr_end);

                result = sc_rm_memreg_frag(ipc, &mr_ret, addr_start, addr_end);

                P2V_MR(ipc, &mr_ret);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = mr_ret;
                RPC_SIZE(msg) = 2;
            }
            break;
        case RM_FUNC_MEMREG_FREE :
            {
                uint8_t result;
                uint8_t mr = RPC_U8(msg, 0);

                V2P_MR(ipc, &mr);

                result = sc_rm_memreg_free(ipc, mr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_FIND_MEMREG :
            {
                uint8_t result;
                uint64_t addr_start = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint64_t addr_end = ((uint64_t) RPC_U32(msg, 8) << 32) | RPC_U32(msg, 12);
                uint8_t mr = 0;

                V2P_ADDR(ipc, &addr_start);
                V2P_ADDR(ipc, &addr_end);

                result = sc_rm_find_memreg(ipc, &mr, addr_start, addr_end);

                P2V_MR(ipc, &mr);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = mr;
                RPC_SIZE(msg) = 2;
            }
            break;
        case RM_FUNC_ASSIGN_MEMREG :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);
                uint8_t mr = RPC_U8(msg, 1);

                V2P_PT(ipc, &pt);
                V2P_MR(ipc, &mr);

                result = sc_rm_assign_memreg(ipc, pt, mr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_SET_MEMREG_PERMISSIONS :
            {
                uint8_t result;
                uint8_t mr = RPC_U8(msg, 0);
                uint8_t pt = RPC_U8(msg, 1);
                uint8_t perm = RPC_U8(msg, 2);

                V2P_MR(ipc, &mr);
                V2P_PT(ipc, &pt);

                result = sc_rm_set_memreg_permissions(ipc, mr, pt, perm);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_IS_MEMREG_OWNED :
            {
                int8_t result;
                uint8_t mr = RPC_U8(msg, 0);

                V2P_MR(ipc, &mr);

                result = sc_rm_is_memreg_owned(ipc, mr);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_GET_MEMREG_INFO :
            {
                uint8_t result;
                uint64_t addr_start = 0;
                uint64_t addr_end = 0;
                uint8_t mr = RPC_U8(msg, 0);

                V2P_MR(ipc, &mr);

                result = sc_rm_get_memreg_info(ipc, mr, &addr_start, &addr_end);

                P2V_ADDR(ipc, &addr_start);
                P2V_ADDR(ipc, &addr_end);

                RPC_U32(msg, 0) = addr_start >> 32;
                RPC_U32(msg, 4) = addr_start;
                RPC_U32(msg, 8) = addr_end >> 32;
                RPC_U32(msg, 12) = addr_end;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 5;
            }
            break;
        case RM_FUNC_ASSIGN_PAD :
            {
                uint8_t result;
                uint16_t pad = RPC_U16(msg, 0);
                uint8_t pt = RPC_U8(msg, 2);

                V2P_PT(ipc, &pt);
                V2P_PAD(ipc, &pad);

                result = sc_rm_assign_pad(ipc, pt, pad);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_SET_PAD_MOVABLE :
            {
                uint8_t result;
                uint16_t pad_fst = RPC_U16(msg, 0);
                uint16_t pad_lst = RPC_U16(msg, 2);
                sc_bool_t movable = RPC_U8(msg, 4);

                V2P_PAD(ipc, &pad_fst);
                V2P_PAD(ipc, &pad_lst);

                result = sc_rm_set_pad_movable(ipc, pad_fst, pad_lst, movable);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_IS_PAD_OWNED :
            {
                int8_t result;
                uint8_t pad = RPC_U8(msg, 0);

                result = sc_rm_is_pad_owned(ipc, pad);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case RM_FUNC_DUMP :
            {

                rm_dump(ipc);

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

