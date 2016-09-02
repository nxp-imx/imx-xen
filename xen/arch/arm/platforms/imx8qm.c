/*
 * xen/arch/arm/platforms/imx8qm.c
 *
 * i.MX 8QM setup
 *
 * Copyright (c) 2016 Freescale Inc.
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
#include <asm/p2m.h>
#include <asm/platform.h>
#include <asm/platforms/imx8qm.h>
#include <xen/config.h>
#include <xen/lib.h>
#include <xen/vmap.h>
#include <xen/mm.h>

static const char * const imx8qm_dt_compat[] __initconst =
{
    "fsl,imx8qm",
    NULL
};

static int imx8qm_system_init(void)
{
    /* TBD */
    return 0;
}

/* Additional mappings for dom0 (not in the DTS) */
static int imx8qm_specific_mapping(struct domain *d)
{
    int i;
    unsigned long lpcg_array[] = LPCG_ARRAY;

    for (i = 0; i < ARRAY_SIZE(lpcg_array); i++)
    {
        map_mmio_regions(d, _gfn(paddr_to_pfn(lpcg_array[i])), 16,
			 _mfn(paddr_to_pfn(lpcg_array[i])));
    }

    return 0;
}

static void imx8qm_system_reset(void)
{
    /* Add PSCI interface */
}

static void imx8qm_system_off(void)
{
  /* Add PSCI interface */
}

PLATFORM_START(imx8qm, "i.MX 8")
    .compatible = imx8qm_dt_compat,
    .init = imx8qm_system_init,
    .specific_mapping = imx8qm_specific_mapping,
    .reset = imx8qm_system_reset,
    .poweroff = imx8qm_system_off,
PLATFORM_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
