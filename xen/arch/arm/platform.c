/*
 * xen/arch/arm/platform.c
 *
 * Helpers to execute platform specific code.
 *
 * Julien Grall <julien.grall@linaro.org>
 * Copyright (C) 2013 Linaro Limited.
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

#include <asm/platform.h>
#include <xen/device_tree.h>
#include <xen/init.h>
#include <asm/psci.h>

extern const struct platform_desc _splatform[], _eplatform[];

/* Pointer to the current platform description */
static const struct platform_desc *platform;


static bool __init platform_is_compatible(const struct platform_desc *plat)
{
    const char *const *compat;

    if ( !plat->compatible )
        return false;

    for ( compat = plat->compatible; *compat; compat++ )
    {
        if ( dt_machine_is_compatible(*compat) )
            return true;
    }

    return false;
}

void __init platform_init(void)
{
    int res = 0;

    ASSERT(platform == NULL);

    /* Looking for the platform description */
    for ( platform = _splatform; platform != _eplatform; platform++ )
    {
        if ( platform_is_compatible(platform) )
            break;
    }

    /* We don't have specific operations for this platform */
    if ( platform == _eplatform )
    {
        /* TODO: dump DT machine compatible node */
        printk(XENLOG_INFO "Platform: Generic System\n");
        platform = NULL;
    }
    else
        printk(XENLOG_INFO "Platform: %s\n", platform->name);

    if ( platform && platform->init )
        res = platform->init();

    if ( res )
        panic("Unable to initialize the platform");
}

int __init platform_init_time(void)
{
    int res = 0;

    if ( platform && platform->init_time )
        res = platform->init_time();

    return res;
}

int __init platform_specific_mapping(struct domain *d)
{
    int res = 0;

    if ( platform && platform->specific_mapping )
        res = platform->specific_mapping(d);

    return res;
}

#ifdef CONFIG_ARM_32
int __init platform_cpu_up(int cpu)
{
    if ( psci_ver )
        return call_psci_cpu_on(cpu);

    if ( platform && platform->cpu_up )
        return platform->cpu_up(cpu);

    return -ENODEV;
}

int __init platform_smp_init(void)
{
    if ( platform && platform->smp_init )
        return platform->smp_init();

    return 0;
}
#endif

void platform_reset(void)
{
    if ( platform && platform->reset )
        platform->reset();
}

void platform_poweroff(void)
{
    if ( platform && platform->poweroff )
        platform->poweroff();
}

bool platform_has_quirk(uint32_t quirk)
{
    uint32_t quirks = 0;

    if ( platform && platform->quirks )
        quirks = platform->quirks();

    return (quirks & quirk);
}

bool platform_device_is_blacklisted(const struct dt_device_node *node)
{
    const struct dt_device_match *blacklist = NULL;

    if ( platform && platform->blacklist_dev )
        blacklist = platform->blacklist_dev;

    return (dt_match_node(blacklist, node) != NULL);
}

bool platform_handle_hvc(struct cpu_user_regs *regs)
{
    if (is_hardware_domain(current->domain))
    {
        if (platform && platform->handle_hvc)
            return platform->handle_hvc(regs);
    }

    return false;
}
bool platform_handle_sip(struct cpu_user_regs *regs)
{
    if (is_hardware_domain(current->domain))
    {
        if (platform && platform->handle_sip)
            return platform->handle_sip(regs);
    }

    return false;
}

int platform_domain_create(struct domain *d, unsigned int domcr_flags,
                           struct xen_arch_domainconfig *config)
{
    if (current->domain == d)
	    return -EINVAL;

    if (platform && platform->domain_create)
        return platform->domain_create(d, domcr_flags, config);

    return 0;
}
int platform_domain_destroy(struct domain *d)
{
    if (current->domain == d)
	    return -EINVAL;

    if (platform && platform->domain_destroy)
        return platform->domain_destroy(d);

    return 0;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
