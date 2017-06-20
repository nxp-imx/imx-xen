/*
 * xen/drivers/char/imx8-lpuart.c
 *
 * Driver for i.MX8 LPUART.
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

#include <xen/config.h>
#include <xen/console.h>
#include <xen/errno.h>
#include <xen/serial.h>
#include <xen/init.h>
#include <xen/irq.h>
#include <xen/mm.h>
#include <asm/device.h>
#include <asm/imx8-lpuart.h>
#include <asm/io.h>

//#define CONFIG_IMX8_ZEBU
#define imx8_lpuart_read(uart, off)       readl((uart)->regs + off)
#define imx8_lpuart_write(uart, off, val) writel((val), (uart)->regs + off)

static struct imx8_lpuart {
    unsigned int baud, clock_hz, data_bits, parity, stop_bits, fifo_size;
    unsigned int irq;
    char __iomem *regs;
    struct irqaction irqaction;
    struct vuart_info vuart;
} imx8_com = {0};

static void imx8_lpuart_interrupt(int irq, void *data,
                                  struct cpu_user_regs *regs)
{
    struct serial_port *port = data;
    struct imx8_lpuart *uart = port->uart;
    unsigned int sts, rxcnt;

    sts = imx8_lpuart_read(uart, UARTSTAT);
    rxcnt = imx8_lpuart_read(uart, UARTWATER) >> UARTWATER_RXCNT_OFF;

    if ((sts & UARTSTAT_RDRF) || (rxcnt > 0))
	    serial_rx_interrupt(port, regs);

    if ((sts & UARTSTAT_TDRE) &&
        !(imx8_lpuart_read(uart, UARTBAUD) & UARTBAUD_TDMAE))
	    serial_tx_interrupt(port, regs);

    imx8_lpuart_write(uart, UARTSTAT, sts);
}

static void __init imx8_lpuart_init_preirq(struct serial_port *port)
{
    struct imx8_lpuart *uart = port->uart;
    u32 sbr, osr;
    u32 ctrl, old_ctrl, bd;
    u32 baud;

    dprintk(XENLOG_ERR, "%s\n", __func__);
    ctrl = old_ctrl = imx8_lpuart_read(uart, UARTCTRL);
    ctrl = (old_ctrl & ~UARTCTRL_M) | UARTCTRL_TE | UARTCTRL_RE;
    bd = imx8_lpuart_read(uart, UARTBAUD);
    baud = uart->baud;

    while ( !(imx8_lpuart_read(uart, UARTSTAT) & UARTSTAT_TC))
	    barrier();

    /* Disable trasmit and receive */
    imx8_lpuart_write(uart, UARTCTRL, old_ctrl & ~(UARTCTRL_TE | UARTCTRL_RE));

    osr = (bd >> UARTBAUD_OSR_SHIFT) & UARTBAUD_OSR_MASK;
    sbr = uart->clock_hz / (baud * (osr + 1));

    bd &= ~ UARTBAUD_SBR_MASK;
    bd |= sbr & UARTBAUD_SBR_MASK;
    bd |= UARTBAUD_BOTHEDGE;
    bd &= ~(UARTBAUD_TDMAE | UARTBAUD_RDMAE);

    imx8_lpuart_write(uart, UARTMODIR, 0);
    imx8_lpuart_write(uart, UARTBAUD, bd);
    imx8_lpuart_write(uart, UARTCTRL, ctrl);
    //asm volatile("tmp: b tmp\r\n");
    dprintk(XENLOG_ERR, "%s\n", __func__);
    //asm volatile("tmp1: b tmp1\r\n");
}

static void __init imx8_lpuart_init_postirq(struct serial_port *port)
{
    struct imx8_lpuart *uart = port->uart;
    unsigned int temp;

    uart->irqaction.handler = imx8_lpuart_interrupt;
    uart->irqaction.name = "imx8_lpuart";
    uart->irqaction.dev_id = port;

    if ( setup_irq(uart->irq, 0, &uart->irqaction) != 0 )
    {
        dprintk(XENLOG_ERR, "Failed to allocate imx8_lpuart IRQ %d\n",
                uart->irq);
        return;
    }

    /* Enable interrupte */
    dprintk(XENLOG_ERR, "%s\n", __func__);
    temp = imx8_lpuart_read(uart, UARTCTRL);
    temp |= (UARTCTRL_RIE | UARTCTRL_TIE);
    temp |= UARTCTRL_ILIE;
    imx8_lpuart_write(uart, UARTCTRL, temp);
}

static void imx8_lpuart_suspend(struct serial_port *port)
{
    BUG();
}

static void imx8_lpuart_resume(struct serial_port *port)
{
    BUG();
}

static int imx8_lpuart_tx_ready(struct serial_port *port)
{
    struct imx8_lpuart *uart = port->uart;

    return (imx8_lpuart_read(uart, UARTSTAT) & UARTSTAT_TC) ? 1 : 0;
}

static void imx8_lpuart_putc(struct serial_port *port, char c)
{
    struct imx8_lpuart *uart = port->uart;

    while ( !(imx8_lpuart_read(uart, UARTSTAT) & UARTSTAT_TDRE))
        barrier();

    imx8_lpuart_write(uart, UARTDATA, c);
}

static int imx8_lpuart_getc(struct serial_port *port, char *pc)
{
    struct imx8_lpuart *uart = port->uart;
    int ch;

    while ( !(imx8_lpuart_read(uart, UARTSTAT) & UARTSTAT_RDRF))
        barrier();

    ch = imx8_lpuart_read(uart, UARTDATA);
    *pc = ch & 0xff;

    if (imx8_lpuart_read(uart, UARTSTAT) &  UARTSTAT_OR)
        imx8_lpuart_write(uart, UARTSTAT, UARTSTAT_OR);

    return 1;
}

static int __init imx8_lpuart_irq(struct serial_port *port)
{
    struct imx8_lpuart *uart = port->uart;

    return ((uart->irq >0) ? uart->irq : -1);
}

static const struct vuart_info *imx8_lpuart_vuart_info(struct serial_port *port)
{
    struct imx8_lpuart *uart = port->uart;

    return &uart->vuart;
}

static void imx8_lpuart_start_tx(struct serial_port *port)
{
    struct imx8_lpuart *uart = port->uart;
    unsigned int temp;

    temp = imx8_lpuart_read(uart, UARTSTAT);
    /* Wait until empty */
    while (!(temp & UARTSTAT_TDRE))
	    barrier();

    temp = imx8_lpuart_read(uart, UARTCTRL);
    imx8_lpuart_write(uart, UARTCTRL, (temp | UARTCTRL_TIE));

    return;
}

static void imx8_lpuart_stop_tx(struct serial_port *port)
{
    struct imx8_lpuart *uart = port->uart;
    unsigned int temp;

    temp = imx8_lpuart_read(uart, UARTCTRL);
    temp &= ~(UARTCTRL_TIE | UARTCTRL_TCIE);
    imx8_lpuart_write(uart, UARTCTRL, temp);

    return;
}

static struct uart_driver __read_mostly imx8_lpuart_driver = {
    .init_preirq = imx8_lpuart_init_preirq,
    .init_postirq = imx8_lpuart_init_postirq,
    .endboot = NULL,
    .suspend = imx8_lpuart_suspend,
    .resume = imx8_lpuart_resume,
    .tx_ready = imx8_lpuart_tx_ready,
    .putc = imx8_lpuart_putc,
    .getc = imx8_lpuart_getc,
    .irq = imx8_lpuart_irq,
    .start_tx = imx8_lpuart_start_tx,
    .stop_tx = imx8_lpuart_stop_tx,
    .vuart_info = imx8_lpuart_vuart_info,
};

static int __init imx8_lpuart_init(struct dt_device_node *dev,
                                     const void *data)
{
    const char *config = data;
    struct imx8_lpuart *uart;
    u32 clkspec;
    int res;
    u64 addr, size;

    dprintk(XENLOG_ERR, "xx %x\n", EARLY_UART_BASE_ADDRESS);
    if ( strcmp(config, "") )
        printk("WARNING: UART configuration is not supported\n");

    uart = &imx8_com;

#ifdef CONFIG_IMX8_ZEBU
    /* 80M */
    clkspec = 80000000;
#else
    res = dt_property_read_u32(dev, "clock-frequency", &clkspec);
    if ( !res )
    {
	res = dt_property_read_u32(dev, "assigned-clock-rates", &clkspec);
	if ( !res )
	{
		printk("imx-uart: Unable to retrieve the clock frequency\n");
		return -EINVAL;
	}
    }
#endif

#define PARITY_NONE  (0)
    uart->clock_hz = clkspec;
    uart->baud = 115200;
    /* For emulation */
#ifdef CONFIG_IMX8_ZEBU
    uart->baud = 1250000;
#else
    uart->baud = 115200;
#endif
    uart->data_bits = 8;
    uart->parity = PARITY_NONE;
    uart->stop_bits = 1;

    res = dt_device_get_address(dev, 0, &addr, &size);
    if ( res )
    {
        printk("imx8-lpuart: Unable to retrieve the base"
               " address of the UART\n");
        return res;
    }

    res = platform_get_irq(dev, 0);
    if ( res < 0 )
    {
        printk("imx8-lpuart: Unable to retrieve the IRQ\n");
        return -EINVAL;
    }
    uart->irq = res;

    uart->regs = ioremap_nocache(addr, size);
    if ( !uart->regs )
    {
        printk("imx8-lpuart: Unable to map the UART memory\n");
        return -ENOMEM;
    }

    uart->vuart.base_addr = addr;
    uart->vuart.size = size;
    uart->vuart.data_off = UARTDATA;
    /* tmp from uboot */
    uart->vuart.status_off = UARTSTAT;
    uart->vuart.status = UARTSTAT_TDRE;

    dprintk(XENLOG_ERR, "11\n");
    /* Register with generic serial driver */
    serial_register_uart(SERHND_DTUART, &imx8_lpuart_driver, uart);

    dt_device_set_used_by(dev, DOMID_XEN);

    return 0;
}

static const struct dt_device_match imx8_lpuart_dt_compat[] __initconst =
{
    DT_MATCH_COMPATIBLE("fsl,imx8qm-lpuart"),
    {},
};

DT_DEVICE_START(imx8_lpuart, "i.MX8 LPUART", DEVICE_SERIAL)
    .dt_match = imx8_lpuart_dt_compat,
    .init = imx8_lpuart_init,
DT_DEVICE_END
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
