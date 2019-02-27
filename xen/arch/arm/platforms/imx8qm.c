/*
 * xen/arch/arm/platforms/imx8qm.c
 *
 * i.MX 8QM setup
 *
 * Copyright (c) 2016 Freescale Inc.
 * Copyright 2018-2019 NXP
 *
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
#include <asm/sci.h>
#include <asm/psci.h>
#include <asm/p2m.h>
#include <asm/platform.h>
#include <asm/platforms/imx8qm.h>
#include <asm/platforms/imx8qxp.h>
#include <xen/config.h>
#include <xen/lib.h>
#include <xen/vmap.h>
#include <xen/mm.h>
#include <asm/smccc.h>
#include <asm/sci.h>
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
struct imx8qm_rsrc_sid {
	uint32_t domain_id;
	uint32_t partition_id;
	uint32_t rsrc_index;
	struct dt_device_node *node[SC_R_LAST];
	uint32_t rsrc[SC_R_LAST];
	uint32_t sid[SC_R_LAST];
};

struct imx8qm_domain {
    domid_t domain_id;
    u32 partition_id;
    u32 partition_id_parent;
    char dom_name[256];
    u32 init_on_num_rsrc;
    u32 init_on_rsrcs[32];
    u32 num_rsrc;
    u32 rsrcs[SC_R_LAST];
    u32 num_rsrc_m41_ap;
    u32 m41_ap_rsrcs[SC_R_LAST];
    u32 num_pad;
    u32 pads[512];
    u32 num_pad_m41_ap;
    u32 m41_ap_pads[512];
    /* Each i.MX8QM domain has such a gpio_dom entry */
    struct gpio_dom gpio_dom;
    int *magic_num;
    int android_auto;
};

#define QM_NUM_DOMAIN	8
/*
 * 8 user domains
 * TODo: use locks to protect the data, currently we only has 2 domains,
 * so it is ok.
 */
static struct imx8qm_domain imx8qm_doms[QM_NUM_DOMAIN];
static struct imx8qm_rsrc_sid rsrc_sid[QM_NUM_DOMAIN];

static int register_gpio_check(struct dt_phandle_args *gpiospec, struct imx8qm_domain *imx8qm_dom)
{
    struct gpio_dom *gpio_dom = &imx8qm_dom->gpio_dom;
    struct dt_device_node *gpio_node;
    paddr_t reg_base;
    u64 bit, rc, i;

    gpio_node = gpiospec->np;
    bit = gpiospec->args[0];
    rc = dt_device_get_address(gpio_node, 0, &reg_base, NULL);
    if (rc)
    {
        dprintk(XENLOG_ERR, "err gpio reg\n");
        return rc;
    }

    for (i = 0; i < MAX_GPIO_CONTROLLER; i++)
    {
        if (gpio_dom->gpios[i].base == reg_base)
            break;
    }
    if (i == MAX_GPIO_CONTROLLER)
    {
        for (i = 0; i < MAX_GPIO_CONTROLLER; i++)
        {
            if (gpio_dom->gpios[i].base == 0)
                break;
        }
    }

    gpio_dom->gpios[i].base = reg_base;
    set_bit(bit, &gpio_dom->gpios[i].bits);
    set_bit(bit << 1, &gpio_dom->gpios[i].ext_bits);
    set_bit((bit << 1) + 1, &gpio_dom->gpios[i].ext_bits);

    dprintk(XENLOG_DEBUG, "Enable GPIO for %s %lx %x %lx %ld\n", imx8qm_dom->dom_name, reg_base, gpio_dom->gpios[i].bits, gpio_dom->gpios[i].ext_bits, bit);

    return 0;
}

static int imx8qm_system_init(void)
{
    struct dt_device_node *np = NULL;
    unsigned int i, j, rsrc_size;
    struct dt_phandle_args gpiospec;
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

            prop = dt_get_property(np, "android-auto", NULL);
            if ( prop )
            {
		imx8qm_doms[i].android_auto = fdt32_to_cpu(*prop);
		printk("Android Auto = %d\n", imx8qm_doms[i].android_auto);
            }

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
	    prop = dt_get_property(np, "rsrcs", &rsrc_size);
	    if (prop)
            {
                if (!dt_property_read_u32_array(np, "rsrcs",
                                                imx8qm_doms[i].rsrcs,
                                                rsrc_size >> 2))
                    panic("Reading rsrcs Error\n");
                imx8qm_doms[i].num_rsrc = rsrc_size >> 2;
	    }
	    prop = dt_get_property(np, "rsrcs-m41-ap", &rsrc_size);
	    if (prop)
            {
                if (!dt_property_read_u32_array(np, "rsrcs-m41-ap",
                                                imx8qm_doms[i].m41_ap_rsrcs,
                                                rsrc_size >> 2))
                    panic("Reading rsrcs Error\n");
                imx8qm_doms[i].num_rsrc_m41_ap = rsrc_size >> 2;
	    }
	    prop = dt_get_property(np, "pads", &rsrc_size);
	    if (prop)
            {
                if (!dt_property_read_u32_array(np, "pads",
                                                imx8qm_doms[i].pads,
                                                rsrc_size >> 2))
                    panic("Reading rsrcs Error\n");
                imx8qm_doms[i].num_pad = rsrc_size >> 2;
	    }
	    prop = dt_get_property(np, "pads-m41-ap", &rsrc_size);
	    if (prop)
            {
                if (!dt_property_read_u32_array(np, "pads-m41-ap",
                                                imx8qm_doms[i].m41_ap_pads,
                                                rsrc_size >> 2))
                    panic("Reading rsrcs Error\n");
                imx8qm_doms[i].num_pad_m41_ap = rsrc_size >> 2;
	    }
	    j = 0;
            while (!dt_parse_phandle_with_args(np, "gpios",
                                               "#gpio-cells", j,
                                               &gpiospec))
	    {
                ret = register_gpio_check(&gpiospec, &imx8qm_doms[i]);
                if (ret)
                {
                    dprintk(XENLOG_ERR, "failed to add gpio %s\n",
                            gpiospec.np->name);
                    return ret;
                }
                j++;
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
    int i;
    for (i = 0; i < QM_NUM_DOMAIN; i++)
    {
        if (imx8qm_doms[i].partition_id)
            sc_rm_partition_free(mu_ipcHandle, imx8qm_doms[i].partition_id);
    }
    /* This is mainly for PSCI-0.2, which does not return if success. */
    call_psci_system_reset();
}

static void imx8qm_system_off(void)
{
  /* Add PSCI interface */
}

static bool imx8qm_smc(struct cpu_user_regs *regs)
{
    struct arm_smccc_res res;

    if ( !cpus_have_const_cap(ARM_SMCCC_1_1) )
    {
        printk_once(XENLOG_WARNING "no SMCCC 1.1 support. Disabling firmware calls\n");

        return false;
    }

    arm_smccc_1_1_smc(get_user_reg(regs, 0),
                      get_user_reg(regs, 1),
                      get_user_reg(regs, 2),
                      get_user_reg(regs, 3),
                      get_user_reg(regs, 4),
                      get_user_reg(regs, 5),
                      get_user_reg(regs, 6),
                      get_user_reg(regs, 7),
                      &res);

    set_user_reg(regs, 0, res.a0);
    set_user_reg(regs, 1, res.a1);
    set_user_reg(regs, 2, res.a2);
    set_user_reg(regs, 3, res.a3);

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

struct imx_lsio_mu1 {
	spinlock_t lock;
	paddr_t base;
	paddr_t size;
	uint32_t regs[10];
};


struct imx_lsio_mu1 lsio_mu1;

static int lsio_mu1_mmio_read(struct vcpu *v, mmio_info_t *info,
                                   register_t *r, void *priv)
{
    paddr_t off = info->gpa - lsio_mu1.base;

    spin_lock(&lsio_mu1.lock);
    if (off <= lsio_mu1.size)
	    *r = lsio_mu1.regs[off / 4];
    spin_unlock(&lsio_mu1.lock);

    return 1;
}

static int lsio_mu1_mmio_write(struct vcpu *v, mmio_info_t *info,
                                    register_t r, void *priv)
{
    paddr_t off = info->gpa - lsio_mu1.base;

    spin_lock(&lsio_mu1.lock);
    if (off <= lsio_mu1.size)
	    lsio_mu1.regs[off / 4] = r;
    spin_unlock(&lsio_mu1.lock);

    return 1;
}

static const struct mmio_handler_ops lsio_mu1_mmio_handler = {
    .read  = lsio_mu1_mmio_read,
    .write = lsio_mu1_mmio_write,
};

static int element_in_array(struct imx8qm_domain *dom, uint32_t ele, uint32_t *array, uint32_t size)
{
	uint32_t i;

	if (dom && dom->android_auto)
	{
		for (i = 0; i < size; i++)
			if (ele == array[i])
				return 1;
	}

	return 0;
}

static int imx8qm_domain_create(struct domain *d,
                                struct xen_domctl_createdomain *config)
{
    unsigned int i, j;
    sc_err_t sci_err;

    /* No need for control domain */
    if (d->domain_id == 0) {
        if (dt_machine_is_compatible("fsl,imx8qm"))
        {
		/*
		 * Since dom0 Linux use lsio_mu1, for scu firmware
		 * to probe correctly, it use interrupt, however
		 * we could not use interrupt, so ignore the real write.
		 */
		spin_lock_init(&lsio_mu1.lock);
		register_mmio_handler(d, &lsio_mu1_mmio_handler, 0x5d1c0000, 0x10000, NULL);
		lsio_mu1.base = 0x5d1c0000;
		lsio_mu1.size = 0x28;
            domain_vgpio_init(d, NULL);
	}
        return 0;
    }

    for (i = 0; i < QM_NUM_DOMAIN; i++)
    {
        if (!strncmp(imx8qm_doms[i].dom_name, config->arch.dom_name, 256))
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

	return 0;
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
    /* Not control domain */
    if (dt_machine_is_compatible("fsl,imx8qm"))
    {
	sc_rm_pt_t parent_part, os_part;

        domain_vgpio_init(d, &imx8qm_doms[i].gpio_dom);

	sci_err = sc_rm_get_partition(mu_ipcHandle, &parent_part);
	sci_err = sc_rm_partition_alloc(mu_ipcHandle, &os_part, false, false,
                                                    false, true, false);

	/* Overwrite uboot partition id */
	imx8qm_doms[i].partition_id = os_part;
	imx8qm_doms[i].partition_id_parent = parent_part;

	sci_err = sc_rm_set_parent(mu_ipcHandle, os_part, parent_part);
        for (j = 0; j < imx8qm_doms[i].num_rsrc; j++)
        {
            if (is_control_domain(current->domain))
            {
		/* Currently owned by M41, ignore the assign in XEN, when android booting, M41 will assign the resource to android */
                if (element_in_array(&imx8qm_doms[i], imx8qm_doms[i].rsrcs[j], imx8qm_doms[i].m41_ap_rsrcs, imx8qm_doms[i].num_rsrc_m41_ap))
		{
			if (!sc_rm_is_resource_owned(mu_ipcHandle, imx8qm_doms[i].rsrcs[j]))
			{
				printk("ignore assign resource %d\n", imx8qm_doms[i].rsrcs[j]);
				continue;
			}
		}
                sci_err = sc_rm_assign_resource(mu_ipcHandle, os_part, imx8qm_doms[i].rsrcs[j]);
		if (sci_err != SC_ERR_NONE)
			printk("assign resource error %d %d\n", imx8qm_doms[i].rsrcs[j], sci_err);
	    }
        }

        for (j = 0; j < imx8qm_doms[i].num_pad; j++)
        {
            if (is_control_domain(current->domain))
            {
		/* Currently owned by M41, ignore the assign in XEN, when android booting, M41 will assign the pad to android */
                if (element_in_array(&imx8qm_doms[i], imx8qm_doms[i].pads[j], imx8qm_doms[i].m41_ap_pads, imx8qm_doms[i].num_pad_m41_ap))
		{
			if (!sc_rm_is_pad_owned(mu_ipcHandle, imx8qm_doms[i].pads[j]))
			{
				printk("ignore assign pad %d\n", imx8qm_doms[i].pads[j]);
				continue;
			}
		}
                sci_err = sc_rm_assign_pad(mu_ipcHandle, os_part, imx8qm_doms[i].pads[j]);
		if (sci_err != SC_ERR_NONE)
			printk("assign pad error %d %d\n", imx8qm_doms[i].pads[j], sci_err);
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

    if (imx8qm_doms[i].partition_id)
    {
        sc_rm_partition_free(mu_ipcHandle, imx8qm_doms[i].partition_id);

        sci_err = sc_pm_set_resource_power_mode_all(mu_ipcHandle, imx8qm_doms[i].partition_id, SC_PM_PW_MODE_OFF, SC_R_LAST);
        if (sci_err != SC_ERR_NONE)
    	    printk("off partition %d err %d\n", imx8qm_doms[i].partition_id, sci_err);
    }

    for (i = 0; i < QM_NUM_DOMAIN; i++)
    {
        if (rsrc_sid[i].domain_id == d->domain_id)
		memset(&rsrc_sid[i], 0, sizeof(rsrc_sid[0]));
    }

    return 0;
}

int platform_deassign_dev(struct domain *d, struct dt_device_node *dev)
{
    int i, j;
    sc_err_t sci_err;
    bool ignore_power_off = false;

    if (!dt_machine_is_compatible("fsl,imx8qm"))
    {
        return 0;
    }
    if (d->domain_id == 0)
	    return 0;

    for (i = 0; i < QM_NUM_DOMAIN; i++)
    {
        /* Find out the related resources */
        if (imx8qm_doms[i].domain_id == d->domain_id)
        {
		break;
	}
    }

    if (i == QM_NUM_DOMAIN)
	    return 0;
    if (!imx8qm_doms[i].partition_id)
	    return 0;

    if (mfn_eq(gfn_to_mfn(d, GUEST_RAM0_BASE >> PAGE_SHIFT), INVALID_MFN) )
    {
        printk("no valid guest ram0 base mfn\n");
    }
    else
    {
        imx8qm_doms[i].magic_num = mfn_to_virt(gfn_to_mfn(d, GUEST_RAM0_BASE >> PAGE_SHIFT));
        if (imx8qm_doms[i].magic_num)
        {
            if (*imx8qm_doms[i].magic_num == 0xF53535F5)
            {
                    *imx8qm_doms[i].magic_num = 0;
		    ignore_power_off = true;
            }
	}
    }

    if (!ignore_power_off)
    {
        sci_err = sc_pm_set_resource_power_mode_all(mu_ipcHandle, imx8qm_doms[i].partition_id, SC_PM_PW_MODE_OFF, SC_R_LAST);
        if (sci_err != SC_ERR_NONE)
                printk("off partition %d err %d\n", imx8qm_doms[i].partition_id, sci_err);

        printk("let's shutdown the domain resources\n");
    }
    printk("partition id %d; domain_id %d\n", imx8qm_doms[i].partition_id, d->domain_id);

    imx8qm_doms[i].domain_id = 0xFFFF;

    if (imx8qm_doms[i].partition_id)
    {
        for (j = 0; j < imx8qm_doms[i].num_rsrc_m41_ap; j++)
        {
                sci_err = sc_rm_assign_resource(mu_ipcHandle, 4, imx8qm_doms[i].m41_ap_rsrcs[j]);
		if (sci_err != SC_ERR_NONE)
			printk("assign resource error parent %d %d\n", imx8qm_doms[i].m41_ap_rsrcs[j], sci_err);
        }

        for (j = 0; j < imx8qm_doms[i].num_rsrc; j++)
        {
                if (element_in_array(&imx8qm_doms[i], imx8qm_doms[i].rsrcs[j], imx8qm_doms[i].m41_ap_rsrcs, imx8qm_doms[i].num_rsrc_m41_ap))
		{
			printk("2: ignore assign resource %d\n", imx8qm_doms[i].rsrcs[j]);
			continue;
		}
                sci_err = sc_rm_assign_resource(mu_ipcHandle, imx8qm_doms[i].partition_id_parent, imx8qm_doms[i].rsrcs[j]);
		if (sci_err != SC_ERR_NONE)
			printk("assign resource error parent %d %d\n", imx8qm_doms[i].rsrcs[j], sci_err);
        }

        for (j = 0; j < imx8qm_doms[i].num_pad_m41_ap; j++)
        {
                sci_err = sc_rm_assign_pad(mu_ipcHandle, 4, imx8qm_doms[i].m41_ap_pads[j]);
		if (sci_err != SC_ERR_NONE)
			printk("assign pad error parent %d %d\n", imx8qm_doms[i].m41_ap_pads[j], sci_err);
        }

        for (j = 0; j < imx8qm_doms[i].num_pad; j++)
        {
                if (element_in_array(&imx8qm_doms[i], imx8qm_doms[i].pads[j], imx8qm_doms[i].m41_ap_pads, imx8qm_doms[i].num_pad_m41_ap))
		{
			printk("2: ignore assign pad %d\n", imx8qm_doms[i].pads[j]);
			continue;
		}
                sci_err = sc_rm_assign_pad(mu_ipcHandle, imx8qm_doms[i].partition_id_parent, imx8qm_doms[i].pads[j]);
		if (sci_err != SC_ERR_NONE)
			printk("assign pad error parent %d %d\n", imx8qm_doms[i].pads[j], sci_err);
        }

        sc_rm_partition_free(mu_ipcHandle, imx8qm_doms[i].partition_id);

        imx8qm_doms[i].partition_id = 0;
    }

    /* Android Auto */
    if (imx8qm_doms[i].android_auto)
    {
	uint32_t parm1 = 1;
	uint32_t parm2 = 0;
	uint32_t parm3 = 0;
	sci_err = sc_misc_board_ioctl(mu_ipcHandle, &parm1, &parm2, &parm3);
	if (sci_err != SC_ERR_NONE)
		printk("board ioctl failed %d\n", sci_err);
    }

    for (i = 0; i < QM_NUM_DOMAIN; i++)
    {
        if (rsrc_sid[i].domain_id == d->domain_id)
		memset(&rsrc_sid[i], 0, sizeof(rsrc_sid[0]));
    }

    return 0;
}

int platform_assign_dev(struct domain *d, u8 devfn, struct dt_device_node *dev, u32 flag)
{
    const __be32 *prop;
    uint32_t rsrcs[32]; 
    uint32_t rsrc_size;
    struct dt_device_node *smmu_np;
    struct dt_phandle_args masterspec;
    uint32_t i;
    sc_err_t sci_err;
    uint32_t index, rsrc_index;

    if (!dt_machine_is_compatible("fsl,imx8qm"))
    {
        return 0;
    }

    for (index = 0; index < QM_NUM_DOMAIN; index++)
    {
        if (rsrc_sid[index].domain_id == d->domain_id)
		break;
    }

    if (index == QM_NUM_DOMAIN)
    {
        for (index = 0; index < QM_NUM_DOMAIN; index++)
        {
            if (rsrc_sid[index].domain_id == 0)
            	break;
        }
    }

    if (index == QM_NUM_DOMAIN)
        return -1;

    rsrc_sid[index].domain_id = d->domain_id;
    rsrc_index = rsrc_sid[index].rsrc_index;

    smmu_np = dt_find_compatible_node(NULL, NULL, "arm,mmu-500");
    if (!smmu_np)
	    return 0;

    prop = dt_get_property(dev, "fsl,sc_rsrc_id", &rsrc_size);
    if (prop)
    {
        if (!dt_property_read_u32_array(dev, "fsl,sc_rsrc_id", rsrcs, rsrc_size >> 2))
        {
            printk("%s failed to get resource list\n", __func__);
	    return -1;
	}
    }
    else
    {
        struct dt_device_node *pnode = NULL;

        prop = dt_get_property(dev, "power-domains", NULL);
	if (!prop)
        {
            printk("%s no power domains\n", __func__);
            return -1;
	}
        pnode = dt_find_node_by_phandle(be32_to_cpup(prop));
	if (!pnode)
        {
            printk("%s no power domain node\n", __func__);
            return -1;
	}
	if (!dt_property_read_u32(pnode, "reg", rsrcs))
        {
            printk("%s no reg node\n", __func__);
            return -1;
	}
	rsrc_size = 4;
    }

    i = 0;
    while (!dt_parse_phandle_with_args(smmu_np, "mmu-masters",
                                       "#stream-id-cells", i, &masterspec))
    {
        if (masterspec.np == dev)
        {
            u16 streamid = masterspec.args[0];
	    int j;
            /* Only 1 SID supported on i.MX8QM */
	    for (j = 0; j < (rsrc_size >> 2); j++)
            {
                rsrc_sid[index].rsrc[rsrc_index] = rsrcs[j];
                rsrc_sid[index].sid[rsrc_index] = streamid;
                rsrc_sid[index].node[rsrc_index] = dev;
		rsrc_index++;
                sci_err = sc_rm_set_master_sid(mu_ipcHandle, rsrcs[j], streamid);
                if (sci_err != SC_ERR_NONE)
                    printk("set_master_sid resource %d sid 0x%x, err: %d\n", rsrcs[j], streamid, sci_err);
	    }
	}
        i++;
    }

    rsrc_sid[index].rsrc_index = rsrc_index;

    return 0;
}

PLATFORM_START(imx8qm, "i.MX 8")
    .compatible = imx8qm_dt_compat,
    .smc = imx8qm_smc,
    .handle_hvc = imx8qm_handle_hvc,
    .init = imx8qm_system_init,
    .specific_mapping = imx8qm_specific_mapping,
    .reset = imx8qm_system_reset,
    .poweroff = imx8qm_system_off,
    .domain_create = imx8qm_domain_create,
    .domain_destroy = imx8qm_domain_destroy,
PLATFORM_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
