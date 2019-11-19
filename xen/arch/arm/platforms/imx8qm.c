/*
 * xen/arch/arm/platforms/imx8qm.c
 *
 * i.MX 8QM setup
 *
 * Copyright (c) 2016 Freescale Inc.
 *
 * Copyright 2019 NXP
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
#include <asm/platforms/imx8qxp.h>
#include <xen/config.h>
#include <xen/lib.h>
#include <xen/vmap.h>
#include <xen/mm.h>
#include <asm/smccc.h>
#include <asm/sci.h>

static const char * const imx8qm_dt_compat[] __initconst =
{
    "fsl,imx8qm",
    "fsl,imx8qxp",
    NULL
};

static int imx8qm_system_init(void)
{
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

static int imx8qm_domain_create(struct domain *d,
                                struct xen_domctl_createdomain *config)
{
    /* No need for control domain */
    if (d->domain_id == 0)
    {
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
        }
    }

    return 0;
}

static int imx8qm_domain_destroy(struct domain *d)
{
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
