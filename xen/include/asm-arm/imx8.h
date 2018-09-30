/*
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

#ifndef __ASM_ARM_IMX8_H__
#define __ASM_ARM_IMX8_H__

#define MAX_GPIO_CONTROLLER 10

struct gpio_dom {
    struct {
        paddr_t base;
        u32 bits;
	/* each bits extended to 2 bits in ext_bits */
	u64 ext_bits;
    } gpios[MAX_GPIO_CONTROLLER];
};

int domain_vgpio_init(struct domain *d, struct gpio_dom *gpio_dom);
int domain_vgpio_free(struct domain *d);

#endif
