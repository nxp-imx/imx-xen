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
 * File containing hypervisor RPC functions for the TIMER service.
 *
 * @addtogroup TIMER_SVC
 * @{
 */

/* Includes */

#include "main/types.h"
#include "svc/timer/timer_api.h"
#include "../../main/rpc.h"
#include "svc/timer/timer_rpc.h"

/* Local Defines */

/* Local Types */

/*--------------------------------------------------------------------------*/
/* Translate and forward RPC call                                           */
/*--------------------------------------------------------------------------*/
void timer_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg)
{
    uint8_t func = RPC_FUNC(msg);
    sc_err_t err = SC_ERR_NONE;

    switch (func)
    {
        case TIMER_FUNC_UNKNOWN :
            {
                RPC_SIZE(msg) = 1;
                RPC_R8(msg) = SC_ERR_NOTFOUND;
            }
            break;
        case TIMER_FUNC_SET_WDOG_TIMEOUT :
            {
                uint8_t result;
                uint32_t timeout = RPC_U32(msg, 0);

                result = sc_timer_set_wdog_timeout(ipc, timeout);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_SET_WDOG_PRE_TIMEOUT :
            {
                uint8_t result;
                uint32_t pre_timeout = RPC_U32(msg, 0);

                result = sc_timer_set_wdog_pre_timeout(ipc, pre_timeout);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_START_WDOG :
            {
                uint8_t result;
                sc_bool_t lock = RPC_U8(msg, 0);

                result = sc_timer_start_wdog(ipc, lock);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_STOP_WDOG :
            {
                uint8_t result;

                result = sc_timer_stop_wdog(ipc);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_PING_WDOG :
            {
                uint8_t result;

                result = sc_timer_ping_wdog(ipc);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_GET_WDOG_STATUS :
            {
                uint8_t result;
                uint32_t timeout = 0;
                uint32_t max_timeout = 0;
                uint32_t remaining_time = 0;

                result = sc_timer_get_wdog_status(ipc, &timeout, &max_timeout, &remaining_time);

                RPC_U32(msg, 0) = timeout;
                RPC_U32(msg, 4) = max_timeout;
                RPC_U32(msg, 8) = remaining_time;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 4;
            }
            break;
        case TIMER_FUNC_PT_GET_WDOG_STATUS :
            {
                uint8_t result;
                uint32_t timeout = 0;
                uint32_t remaining_time = 0;
                uint8_t pt = RPC_U8(msg, 0);
                sc_bool_t enb = 0;

                V2P_PT(ipc, &pt);

                result = sc_timer_pt_get_wdog_status(ipc, pt, &enb, &timeout, &remaining_time);

                RPC_U32(msg, 0) = timeout;
                RPC_U32(msg, 4) = remaining_time;
                RPC_R8(msg) = result;
                RPC_U8(msg, 8) = enb;
                RPC_SIZE(msg) = 4;
            }
            break;
        case TIMER_FUNC_SET_WDOG_ACTION :
            {
                uint8_t result;
                uint8_t pt = RPC_U8(msg, 0);
                uint8_t action = RPC_U8(msg, 1);

                V2P_PT(ipc, &pt);

                result = sc_timer_set_wdog_action(ipc, pt, action);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_SET_RTC_TIME :
            {
                uint8_t result;
                uint16_t year = RPC_U16(msg, 0);
                uint8_t mon = RPC_U8(msg, 2);
                uint8_t day = RPC_U8(msg, 3);
                uint8_t hour = RPC_U8(msg, 4);
                uint8_t min = RPC_U8(msg, 5);
                uint8_t sec = RPC_U8(msg, 6);

                result = sc_timer_set_rtc_time(ipc, year, mon, day, hour, min, sec);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_GET_RTC_TIME :
            {
                uint8_t result;
                uint16_t year = 0;
                uint8_t mon = 0;
                uint8_t day = 0;
                uint8_t hour = 0;
                uint8_t min = 0;
                uint8_t sec = 0;

                result = sc_timer_get_rtc_time(ipc, &year, &mon, &day, &hour, &min, &sec);

                RPC_U16(msg, 0) = year;
                RPC_R8(msg) = result;
                RPC_U8(msg, 2) = mon;
                RPC_U8(msg, 3) = day;
                RPC_U8(msg, 4) = hour;
                RPC_U8(msg, 5) = min;
                RPC_U8(msg, 6) = sec;
                RPC_SIZE(msg) = 3;
            }
            break;
        case TIMER_FUNC_GET_RTC_SEC1970 :
            {
                uint8_t result;
                uint32_t sec = 0;

                result = sc_timer_get_rtc_sec1970(ipc, &sec);

                RPC_U32(msg, 0) = sec;
                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 2;
            }
            break;
        case TIMER_FUNC_SET_RTC_ALARM :
            {
                uint8_t result;
                uint16_t year = RPC_U16(msg, 0);
                uint8_t mon = RPC_U8(msg, 2);
                uint8_t day = RPC_U8(msg, 3);
                uint8_t hour = RPC_U8(msg, 4);
                uint8_t min = RPC_U8(msg, 5);
                uint8_t sec = RPC_U8(msg, 6);

                result = sc_timer_set_rtc_alarm(ipc, year, mon, day, hour, min, sec);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_SET_RTC_PERIODIC_ALARM :
            {
                uint8_t result;
                uint32_t sec = RPC_U32(msg, 0);

                result = sc_timer_set_rtc_periodic_alarm(ipc, sec);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_CANCEL_RTC_ALARM :
            {
                uint8_t result;

                result = sc_timer_cancel_rtc_alarm(ipc);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_SET_RTC_CALB :
            {
                uint8_t result;
                int8_t count = RPC_I8(msg, 0);

                result = sc_timer_set_rtc_calb(ipc, count);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_SET_SYSCTR_ALARM :
            {
                uint8_t result;
                uint64_t ticks = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                result = sc_timer_set_sysctr_alarm(ipc, ticks);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_SET_SYSCTR_PERIODIC_ALARM :
            {
                uint8_t result;
                uint64_t ticks = ((uint64_t) RPC_U32(msg, 0) << 32) | RPC_U32(msg, 4);

                result = sc_timer_set_sysctr_periodic_alarm(ipc, ticks);

                RPC_R8(msg) = result;
                RPC_SIZE(msg) = 1;
            }
            break;
        case TIMER_FUNC_CANCEL_SYSCTR_ALARM :
            {
                uint8_t result;

                result = sc_timer_cancel_sysctr_alarm(ipc);

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

