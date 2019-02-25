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
 * File containing hypervisor RPC functions for the PM service.
 *
 * @addtogroup PM_SVC
 * @{
 */

/* Includes */

#include "main/types.h"
#include "svc/pm/pm_api.h"
#include "../../main/rpc.h"
#include "svc/pm/pm_rpc.h"

/* Local Defines */

/* Local Types */

/*--------------------------------------------------------------------------*/
/* Translate and forward RPC call                                           */
/*--------------------------------------------------------------------------*/
void pm_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg)
{
    uint8_t func = RPC_FUNC(msg);
    sc_err_t err = SC_ERR_NONE;

    switch (func)
    {
        case PM_FUNC_UNKNOWN :
            {
                RPC_SIZE(msg) = 1;
                RPC_R8(msg) = SC_ERR_NOTFOUND;
            }
            break;
        case PM_FUNC_SET_SYS_POWER_MODE :
            {
                uint8_t result;
                uint8_t mode = RPC_U8(msg, 0);

                result = sc_pm_set_sys_power_mode(ipc, mode);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_SET_PARTITION_POWER_MODE :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);
                uint8_t mode = RPC_U8(msg, 1);

                V2P_PT(ipc, &pt);

                result = sc_pm_set_partition_power_mode(ipc, pt, mode);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_GET_SYS_POWER_MODE :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);
                uint8_t mode = 0;

                V2P_PT(ipc, &pt);

                result = sc_pm_get_sys_power_mode(ipc, pt, &mode);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = mode;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PM_FUNC_SET_RESOURCE_POWER_MODE :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t mode = RPC_U8(msg, 2);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_set_resource_power_mode(ipc, resource, mode);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_SET_RESOURCE_POWER_MODE_ALL :
            {
                uint8_t result;
                uint16_t exclude = RPC_U16(msg, 0);
                uint8_t pt = RPC_U8(msg, 2);
                uint8_t mode = RPC_U8(msg, 3);

                V2P_PT(ipc, &pt);

                result = sc_pm_set_resource_power_mode_all(ipc, pt, mode, exclude);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_GET_RESOURCE_POWER_MODE :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t mode = 0;

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_get_resource_power_mode(ipc, resource, &mode);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = mode;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PM_FUNC_REQ_LOW_POWER_MODE :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t mode = RPC_U8(msg, 2);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_req_low_power_mode(ipc, resource, mode);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_REQ_CPU_LOW_POWER_MODE :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t mode = RPC_U8(msg, 2);
                uint8_t wake_src = RPC_U8(msg, 3);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_req_cpu_low_power_mode(ipc, resource, mode, wake_src);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_SET_CPU_RESUME_ADDR :
            {
                uint8_t result;
                uint64_t address = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint16_t resource = RPC_U16(msg, 8);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_set_cpu_resume_addr(ipc, resource, address);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_SET_CPU_RESUME :
            {
                uint8_t result;
                uint64_t address = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint16_t resource = RPC_U16(msg, 8);
                sc_bool_t isPrimary = RPC_U8(msg, 10);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_set_cpu_resume(ipc, resource, isPrimary, address);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_REQ_SYS_IF_POWER_MODE :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t sys_if = RPC_U8(msg, 2);
                uint8_t hpm = RPC_U8(msg, 3);
                uint8_t lpm = RPC_U8(msg, 4);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_req_sys_if_power_mode(ipc, resource, sys_if, hpm, lpm);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_SET_CLOCK_RATE :
            {
                uint8_t result;
                uint32_t rate = RPC_U32(msg, 0);
                uint16_t resource = RPC_U16(msg, 4);
                uint8_t clk = RPC_U8(msg, 6);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_set_clock_rate(ipc, resource, clk, &rate);

                RPC_U32(msg, 0) = rate;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PM_FUNC_GET_CLOCK_RATE :
            {
                uint8_t result;
                uint32_t rate = 0;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t clk = RPC_U8(msg, 2);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_get_clock_rate(ipc, resource, clk, &rate);

                RPC_U32(msg, 0) = rate;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PM_FUNC_CLOCK_ENABLE :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t clk = RPC_U8(msg, 2);
                sc_bool_t enable = RPC_U8(msg, 3);
                sc_bool_t autog = RPC_U8(msg, 4);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_clock_enable(ipc, resource, clk, enable, autog);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_SET_CLOCK_PARENT :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t clk = RPC_U8(msg, 2);
                uint8_t parent = RPC_U8(msg, 3);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_set_clock_parent(ipc, resource, clk, parent);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_GET_CLOCK_PARENT :
            {
                uint8_t result;
                uint16_t resource = RPC_U16(msg, 0);
                uint8_t clk = RPC_U8(msg, 2);
                uint8_t parent = 0;

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_get_clock_parent(ipc, resource, clk, &parent);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = parent;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PM_FUNC_RESET :
            {
                uint8_t result;
                uint8_t type = RPC_U8(msg, 0);

                result = sc_pm_reset(ipc, type);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_RESET_REASON :
            {
                uint8_t result;
                uint8_t reason = 0;

                result = sc_pm_reset_reason(ipc, &reason);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = reason;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PM_FUNC_GET_RESET_PART :
            {
                uint8_t result;
                uint8_t pt = 0;

                result = sc_pm_get_reset_part(ipc, &pt);

                P2V_PT(ipc, &pt);

                RPC_R8(msg) = result;
                RPC_U8(msg, 0) = pt;
                RPC_SIZE(msg) = 2;
            }
            break;
        case PM_FUNC_BOOT :
            {
                uint8_t result;
                uint64_t boot_addr = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint16_t resource_cpu = RPC_U16(msg, 8);
                uint16_t resource_mu = RPC_U16(msg, 10);
                uint16_t resource_dev = RPC_U16(msg, 12);
                uint8_t pt = RPC_U8(msg, 14);

                V2P_PT(ipc, &pt);
                V2P_RESOURCE(ipc, &resource_cpu);
                V2P_RESOURCE(ipc, &resource_mu);
                V2P_RESOURCE(ipc, &resource_dev);

                result = sc_pm_boot(ipc, pt, resource_cpu, boot_addr, resource_mu, resource_dev);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_REBOOT :
            {
                uint8_t type = RPC_U8(msg, 0);

                pm_reboot(ipc, type);

                RPC_SIZE(msg) = 0;
            }
            break;
        case PM_FUNC_REBOOT_PARTITION :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);
                uint8_t type = RPC_U8(msg, 1);

                V2P_PT(ipc, &pt);

                result = sc_pm_reboot_partition(ipc, pt, type);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_REBOOT_CONTINUE :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);

                V2P_PT(ipc, &pt);

                result = sc_pm_reboot_continue(ipc, pt);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_CPU_START :
            {
                uint8_t result;
                uint64_t address = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint16_t resource = RPC_U16(msg, 8);
                sc_bool_t enable = RPC_U8(msg, 10);

                V2P_RESOURCE(ipc, &resource);

                result = sc_pm_cpu_start(ipc, resource, enable, address);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case PM_FUNC_CPU_RESET :
            {
                uint64_t address = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);
                uint16_t resource = RPC_U16(msg, 8);

                V2P_RESOURCE(ipc, &resource);

                pm_cpu_reset(ipc, resource, address);

                RPC_SIZE(msg) = 0;
            }
            break;
        case PM_FUNC_IS_PARTITION_STARTED :
            {
                int8_t result;
                uint8_t pt = RPC_U8(msg, 0);

                V2P_PT(ipc, &pt);

                result = sc_pm_is_partition_started(ipc, pt);

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

