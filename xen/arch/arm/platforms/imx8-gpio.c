/*
 * xen/arch/arm/platforms/imx8qm.c
 *
 * i.MX 8 gpio
 *
 * Copyright 2018 NXP
 *
 * Peng Fan <peng.fan@nxp.com>
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

#include <asm/io.h>
#include <asm/imx8.h>
#include <asm/mmio.h>
#include <xen/sched.h>
#include <xen/spinlock.h>
#include <xen/libfdt/libfdt.h>

struct imx_vgpio
{
    spinlock_t lock;
    paddr_t base;
    paddr_t size;
    void __iomem *mapbase;
    struct gpio_dom *gpio_dom;
    struct domain *dom;
};

static int vgpio_mmio_read(struct vcpu *v, mmio_info_t *info, register_t *r,
                           void *priv)
{
    struct imx_vgpio *vgpio = priv;
    struct domain *d = vgpio->dom;
    struct gpio_dom *gpio_dom = vgpio->gpio_dom;
    paddr_t off = info->gpa - vgpio->base;
    int i;

    /* No need to block read */
    if (is_control_domain(d))
    {
        spin_lock(&vgpio->lock);
        *r = readl(vgpio->mapbase + off);
        spin_unlock(&vgpio->lock);
    }
    else
    {
        for (i = 0; i < MAX_GPIO_CONTROLLER; i++)
        {
            if (gpio_dom->gpios[i].base == vgpio->base)
                break;
        }
        if (i == MAX_GPIO_CONTROLLER)
	{
            dprintk(XENLOG_INFO, "Touched address %lx not allowed, ignore\n", info->gpa);
            *r = 0;
            return 1;
	}

        spin_lock(&vgpio->lock);
        *r = readl(vgpio->mapbase + off) & gpio_dom->gpios[i].bits;
        spin_unlock(&vgpio->lock);
    }

    return 1;
}

static int vgpio_mmio_write(struct vcpu *v, mmio_info_t *info, register_t r,
                            void *priv)
{
    struct imx_vgpio *vgpio = priv;
    struct domain *d = vgpio->dom;
    struct gpio_dom *gpio_dom = vgpio->gpio_dom;
    paddr_t off = info->gpa - vgpio->base;
    u32 val;
    int i;

    if (is_control_domain(d))
    {
        spin_lock(&vgpio->lock);
        writel(r, vgpio->mapbase + off);
        spin_unlock(&vgpio->lock);
    }
    else
    {
        if (off < 0x14)
        {
            for (i = 0; i < MAX_GPIO_CONTROLLER; i++)
            {
                if (gpio_dom->gpios[i].base == vgpio->base)
                    break;
	    }
	    if (i == MAX_GPIO_CONTROLLER)
            {
                /* Ignore write */
                return 1;
            }
	    if (r & ~gpio_dom->gpios[i].bits)
	    {
                dprintk(XENLOG_ERR, " Touched address %lx not allowed bits %lx, allowed bits %x\n", info->gpa, r, gpio_dom->gpios[i].bits);
                return 0;
	    }
            spin_lock(&vgpio->lock);
            val = (readl(vgpio->mapbase + off) & ~(gpio_dom->gpios[i].bits)) | (r & gpio_dom->gpios[i].bits);
            writel(val, vgpio->mapbase + off);
            spin_unlock(&vgpio->lock);
	}
	else
	{
            dprintk(XENLOG_INFO, "Touched address %lx not allowed, ignore\n", info->gpa);
            /* Ignore the write for DomUs */
            return 1;
	}
    }

    return 1;
}

static const struct mmio_handler_ops vgpio_mmio_handler = {
    .read = vgpio_mmio_read,
    .write = vgpio_mmio_write,
};

int domain_vgpio_init(struct domain *d, struct gpio_dom *gpio_dom)
{
    struct imx_vgpio *vgpio, *p;
    struct dt_device_node *dn;
    u64 reg_base, reg_size;
    int i = 0, rc;

    dt_for_each_compatible_node(NULL, dn, NULL, "fsl,imx8qm-gpio")
    {
        i++;
    }

    printk("%s gpio controllers %d\n", __func__, i);

    vgpio = xzalloc_array(struct imx_vgpio, i);
    if (!vgpio)
    {
        dprintk(XENLOG_ERR, "Failed alloc vgpio\n");
    }

    i = 0;
    dt_for_each_compatible_node(NULL, dn, NULL, "fsl,imx8qm-gpio")
    {
        p = &vgpio[i];
        rc = dt_device_get_address(dn, 0, &reg_base, &reg_size);
        if ( rc )
        {
            dprintk(XENLOG_ERR, "%s: missing reg prop\n", __func__);
	    return rc;
	}

        p->base = reg_base;
        p->size = reg_size;
        p->mapbase = ioremap_nocache(reg_base, reg_size);
	p->dom = d;
	p->gpio_dom = gpio_dom;
	if ( !p->mapbase )
        {
            dprintk(XENLOG_ERR, "ioremap err\n");
	}

	spin_lock_init(&p->lock);

	register_mmio_handler(d, &vgpio_mmio_handler, reg_base, reg_size, p);

	i++;
    }

    return 0;
}
