/*
 * xen/arch/arm/platforms/imx8qm.c
 *
 * i.MX 8QM setup
 *
 * Copyright (c) 2016 Freescale Inc.
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
#include <asm/sci.h>
#include <asm/p2m.h>
#include <asm/platform.h>
#include <asm/platforms/imx8qm.h>
#include <asm/platforms/imx8qxp.h>
#include <xen/config.h>
#include <xen/lib.h>
#include <xen/vmap.h>
#include <xen/mm.h>
#include <xen/libfdt/libfdt.h>

static const char * const imx8qm_dt_compat[] __initconst =
{
    "fsl,imx8qm",
    "fsl,imx8qxp",
    NULL
};

/*
 * Used for non-privileged domain, the dom_name needs to be same in dom0 dts
 * and cfg file.
 */
struct imx8qm_domain {
    domid_t domain_id;
    u32 partition_id;
    char dom_name[256];
    u32 init_on_num_rsrc;
    u32 init_on_rsrcs[32];
};

#define QM_NUM_DOMAIN	8
/* 8 user domains */
static struct imx8qm_domain imx8qm_doms[QM_NUM_DOMAIN];

static int imx8qm_system_init(void)
{
    struct dt_device_node *np = NULL;
    unsigned int i, rsrc_size;
    int ret;

    while ((np = dt_find_compatible_node(np, NULL, "xen,domu")))
    {
        for (i = 0; i < QM_NUM_DOMAIN; i++)
        {
	    /* 0 means unused, we ignore dom0 */
            if (!imx8qm_doms[i].domain_id)
                break;
	}
        if (i < QM_NUM_DOMAIN)
        {
            const __be32 *prop;
	    const char *name_str;
            prop = dt_get_property(np, "reg", NULL);
            if ( !prop )
            {
                printk("Unable to find reg property\n");
		continue;
            }
	    imx8qm_doms[i].partition_id = fdt32_to_cpu(*prop);
	    printk("partition id %d\n", fdt32_to_cpu(*prop));
            ret = dt_property_read_string(np, "domain_name", &name_str);
	    if (ret)
            {
                printk("No name property\n");
                continue;
            }
            safe_strcpy(imx8qm_doms[i].dom_name, name_str);
	    printk("Domain name %s\n", imx8qm_doms[i].dom_name);

	    prop = dt_get_property(np, "init_on_rsrcs", &rsrc_size);
	    if (prop)
            {
                if (!dt_property_read_u32_array(np, "init_on_rsrcs",
                                                imx8qm_doms[i].init_on_rsrcs,
                                                rsrc_size >> 2))
                    panic("Reading init_on_rsrcs Error\n");
                imx8qm_doms[i].init_on_num_rsrc = rsrc_size >> 2;
	    }
        }
    }

    imx8_mu_init();

    return 0;
}

/* Additional mappings for dom0 (not in the DTS) */
static int imx8qm_specific_mapping(struct domain *d)
{
    unsigned long lpcg_array[] = LPCG_ARRAY;
    unsigned long lpcg_array_8qxp[] = LPCG_ARRAY_8QXP;
    int lpcg_size = ARRAY_SIZE(lpcg_array);
    int i;
    unsigned long *p = lpcg_array;

        dprintk(XENLOG_ERR, "%s: xxxxxxxx\n", __func__);
    if(dt_machine_is_compatible("fsl,imx8qxp")) {
        lpcg_size = ARRAY_SIZE(lpcg_array_8qxp);
        p = lpcg_array_8qxp;
        dprintk(XENLOG_ERR, "%s: 111111xxxxxxxx\n", __func__);
    }

    for (i = 0; i < lpcg_size; i++)
    {
        map_mmio_regions(d, _gfn(paddr_to_pfn(p[i])), 16,
                         _mfn(paddr_to_pfn(p[i])));
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

static bool imx8qm_handle_sip(struct cpu_user_regs *regs)
{
    register_t ret[4] = { 0 };

    call_smcc64(regs->x0, regs->x1, regs->x2, regs->x3, regs->x4,
		regs->x5, regs->x6, ret);

    regs->x0 = ret[0];
    regs->x1 = ret[1];
    regs->x2 = ret[2];
    regs->x3 = ret[3];

    return true;
}

#define FSL_HVC_SC	0xc6000000
extern int imx8_sc_rpc(unsigned long x1, unsigned long x2);
static bool imx8qm_handle_hvc(struct cpu_user_regs *regs)
{
    int err;

    switch (regs->x0)
    {
    case FSL_HVC_SC:
        err = imx8_sc_rpc(regs->x1, regs->x2);
        break;
    default:
        err = -ENOENT;
        break;
    }

    regs->x0 = err;

    return true;
}

static int imx8qm_domain_create(struct domain *d, unsigned int domcr_flags,
                                struct xen_arch_domainconfig *config)
{
    unsigned int i, j;
    sc_err_t sci_err;

    /* No need for control domain */
    if (d->domain_id == 0)
        return 0;

    for (i = 0; i < QM_NUM_DOMAIN; i++)
    {
        if (!strncmp(imx8qm_doms[i].dom_name, config->dom_name, 256))
        {
            imx8qm_doms[i].domain_id = d->domain_id;
	    break;
	}
    }

    if (i == QM_NUM_DOMAIN)
    {
        printk("****************************************************\n");
        printk("NOT FOUND A entry to power off passthrough resources\n");
        printk("The dts node name needs to be same as name = \"xxx\"\n");
        printk("in vm configuration file\n");
        printk("****************************************************\n");
    } else {
        for (j = 0; j < imx8qm_doms[i].init_on_num_rsrc; j++)
        {
            if (is_control_domain(current->domain))
            {
                printk("Power on resource %d\n", imx8qm_doms[i].init_on_rsrcs[j]);
                sci_err = sc_pm_set_resource_power_mode(mu_ipcHandle, imx8qm_doms[i].init_on_rsrcs[j], SC_PM_PW_MODE_ON);
                if (sci_err != SC_ERR_NONE)
                    printk("power on resource %d err: %d\n", imx8qm_doms[i].init_on_rsrcs[j], sci_err);
            }
        }
    }

    return 0;
}

static int imx8qm_domain_destroy(struct domain *d)
{
    unsigned int i;
    sc_err_t sci_err;
    /* No need for control domain */
    if (is_control_domain(d))
        return 0;

    for (i = 0; i < QM_NUM_DOMAIN; i++)
    {
        /* Find out the related resources */
        if (imx8qm_doms[i].domain_id == d->domain_id)
        {
		printk("let's shutdown the domain resources\n");
	        printk("partition id %d; domain_id %d\n", imx8qm_doms[i].partition_id, d->domain_id);
		break;
	}
    }

    if (i == QM_NUM_DOMAIN)
        return 0;

    sci_err = sc_pm_set_resource_power_mode_all(mu_ipcHandle, imx8qm_doms[i].partition_id, SC_PM_PW_MODE_OFF, SC_R_LAST);
    if (sci_err != SC_ERR_NONE)
	    printk("off partition %d err %d\n", imx8qm_doms[i].partition_id, sci_err);

    return 0;
}

PLATFORM_START(imx8qm, "i.MX 8")
    .compatible = imx8qm_dt_compat,
    .init = imx8qm_system_init,
    .specific_mapping = imx8qm_specific_mapping,
    .reset = imx8qm_system_reset,
    .poweroff = imx8qm_system_off,
    .handle_sip = imx8qm_handle_sip,
    .handle_hvc = imx8qm_handle_hvc,
    .domain_destroy = imx8qm_domain_destroy,
    .domain_create = imx8qm_domain_create,
PLATFORM_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
