/*
 * xen/include/asm-arm/imx8-lpuart.h
 *
 * Common constant definition between early printk and the LPUART driver
 * for the i.MX8 LPUART
 *
 * Peng Fan <peng.fan@nxp.com>
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARM_IMX8_LPUART_H
#define __ASM_ARM_IMX8_LPUART_H

/* 32-bit register definition */
#define UARTBAUD          (0x10)
#define UARTSTAT          (0x14)
#define UARTCTRL          (0x18)
#define UARTDATA          (0x1C)
#define UARTMATCH         (0x20)
#define UARTMODIR         (0x24)
#define UARTFIFO          (0x28)
#define UARTWATER         (0x2c)

#define UARTSTAT_TDRE     (1 << 23)
#define UARTSTAT_TC       (1 << 22)
#define UARTSTAT_RDRF     (1 << 21)
#define UARTSTAT_OR       (1 << 19)

#define UARTBAUD_OSR_SHIFT (24)
#define UARTBAUD_OSR_MASK (0x1f)
#define UARTBAUD_SBR_MASK (0x1fff)
#define UARTBAUD_BOTHEDGE (0x00020000)
#define UARTBAUD_TDMAE    (0x00800000)
#define UARTBAUD_RDMAE    (0x00200000)

#define UARTCTRL_TIE      (1 << 23)
#define UARTCTRL_TCIE     (1 << 22)
#define UARTCTRL_RIE      (1 << 21)
#define UARTCTRL_ILIE     (1 << 20)
#define UARTCTRL_TE       (1 << 19)
#define UARTCTRL_RE       (1 << 18)
#define UARTCTRL_M        (1 << 4)

#define UARTWATER_RXCNT_OFF     24

#endif /* __ASM_ARM_IMX8_LPUART_H */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
