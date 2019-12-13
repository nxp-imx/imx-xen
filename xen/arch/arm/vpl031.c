/*
 * ARM AMBA PrimeCell PL031 RTC
 *
 * Copyright (c) 2007 CodeSourcery
 * Copyright 2018 NXP
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include <asm/current.h>
#include <xen/trace.h>
#include <asm/mmio.h>
#include <xen/time.h>
#include <xen/timer.h>
#include <xen/kernel.h>
#include <asm/current.h>
#include <asm/gic.h>
#include <asm/vgic.h>
#include <asm/vpl031.h>
#include <xen/sched.h>

#define USEC_PER_SEC    1000000UL
#define NS_PER_USEC     1000UL
#define NS_PER_SEC      1000000000ULL

#define SEC_PER_MIN     60
#define SEC_PER_HOUR    3600
#define MIN_PER_HOUR    60
#define HOUR_PER_DAY    24

#define RTC_DR      0x00    /* Data read register */
#define RTC_MR      0x04    /* Match register */
#define RTC_LR      0x08    /* Data load register */
#define RTC_CR      0x0c    /* Control register */
#define RTC_IMSC    0x10    /* Interrupt mask and set register */
#define RTC_RIS     0x14    /* Raw interrupt status register */
#define RTC_MIS     0x18    /* Masked interrupt status register */
#define RTC_ICR     0x1c    /* Interrupt clear register */

#define NANOSECONDS_PER_SECOND 1000000000LL

#define vrtc_domain(x) (x->d)

static const unsigned char pl031_id[] = {
    0x31, 0x10, 0x14, 0x00,         /* Device ID        */
    0x0d, 0xf0, 0x05, 0xb1          /* Cell ID      */
};

static void pl031_update(PL031State *s)
{
    struct domain *d = vrtc_domain(s);

    if (s->is & s->im)
        vgic_inject_irq(d, d->vcpu[0], s->irq, 1);
}

static void pl031_interrupt(void * opaque)
{
    PL031State *s = (PL031State *)opaque;

    spin_lock(&s->lock);
    s->is = 1;
    gdprintk(XENLOG_INFO, "Alarm raised\n");
    pl031_update(s);
    spin_unlock(&s->lock);
}

static uint32_t pl031_get_count(PL031State *s)
{
    struct domain *d = vrtc_domain(s);
    int64_t now = get_localtime_us(d);

    return s->tick_offset + now / USEC_PER_SEC - d->time_offset_seconds;
}

static void pl031_set_alarm(PL031State *s)
{
    uint32_t ticks;

    /* The timer wraps around.  This subtraction also wraps in the same way,
       and gives correct results when alarm < now_ticks.  */
    ticks = s->mr - pl031_get_count(s);
    gdprintk(XENLOG_INFO, "Alarm set in %ud ticks\n", ticks);
    if (ticks == 0) {
        stop_timer(&s->timer);
	s->is = 1;
        pl031_update(s);
    } else {
        spin_unlock(&s->lock);
        set_timer(&s->timer, NOW() + (int64_t)ticks * NANOSECONDS_PER_SECOND);
        spin_lock(&s->lock);
    }
}

static uint64_t pl031_read(void *opaque, uint32_t offset,
                           unsigned size)
{
    PL031State *s = (PL031State *)opaque;

    if (offset >= 0xfe0  &&  offset < 0x1000)
        return pl031_id[(offset - 0xfe0) >> 2];

    switch (offset) {
    case RTC_DR:
        return pl031_get_count(s);
    case RTC_MR:
        return s->mr;
    case RTC_IMSC:
        return s->im;
    case RTC_RIS:
        return s->is;
    case RTC_LR:
        return s->lr;
    case RTC_CR:
        /* RTC is permanently enabled.  */
        return 1;
    case RTC_MIS:
        return s->is & s->im;
    case RTC_ICR:
        gdprintk(XENLOG_ERR,
                 "pl031: read of write-only register at offset 0x%x\n",
                 (int)offset);
        break;
    default:
        gdprintk(XENLOG_ERR,
                 "pl031_read: Bad offset 0x%x\n", (int)offset);
        break;
    }

    return 0;
}

static void pl031_write(void * opaque, uint32_t offset,
                        uint64_t value, unsigned size)
{
    PL031State *s = (PL031State *)opaque;

    switch (offset) {
    case RTC_LR:
        s->tick_offset += value - pl031_get_count(s);
        pl031_set_alarm(s);
        break;
    case RTC_MR:
        s->mr = value;
        pl031_set_alarm(s);
        break;
    case RTC_IMSC:
        s->im = value & 1;
        gdprintk(XENLOG_INFO, "Interrupt mask %d\n", s->im);
        pl031_update(s);
        break;
    case RTC_ICR:
        /* The PL031 documentation (DDI0224B) states that the interrupt is
           cleared when bit 0 of the written value is set.  However the
           arm926e documentation (DDI0287B) states that the interrupt is
           cleared when any value is written.  */
        gdprintk(XENLOG_INFO, "Interrupt cleared");
        s->is = 0;
        pl031_update(s);
        break;
    case RTC_CR:
        /* Written value is ignored.  */
        break;

    case RTC_DR:
    case RTC_MIS:
    case RTC_RIS:
        gdprintk(XENLOG_ERR,
                 "pl031: write to read-only register at offset 0x%x\n",
                 (int)offset);
        break;

    default:
        gdprintk(XENLOG_ERR,
                 "pl031_write: Bad offset 0x%x\n", (int)offset);
        break;
    }
}

static int vrtc_mmio_read(struct vcpu *v, mmio_info_t *info, register_t *r,
			  void *priv)
{
	PL031State *s = priv;

	spin_lock(&s->lock);
	*r = pl031_read(s, info->gpa - s->base, 4);
	spin_unlock(&s->lock);

	return 1;
}

static int vrtc_mmio_write(struct vcpu *v, mmio_info_t *info, register_t r,
			   void *priv)
{
	PL031State *s = priv;

	spin_lock(&s->lock);
	pl031_write(s, info->gpa - s->base, r, 4);
	spin_unlock(&s->lock);

	return 1;
}

static const struct mmio_handler_ops vrtc_mmio_handler = {
    .read = vrtc_mmio_read,
    .write = vrtc_mmio_write,
};

s_time_t mktimegm(struct tm *tm)
{
    s_time_t t;
    int y = tm->tm_year + 1900, m = tm->tm_mon + 1, d = tm->tm_mday;
    if (m < 3) {
        m += 12;
        y--;
    }
    t = 86400ULL * (d + (153 * m - 457) / 5 + 365 * y + y / 4 - y / 100 + 
                 y / 400 - 719469);
    t += 3600 * tm->tm_hour + 60 * tm->tm_min + tm->tm_sec;
    return t;
}

void domain_vpl031_deinit(struct domain *d)
{
    PL031State *s = &d->arch.vrtc;

    if (s->irq == GUEST_VPL031_SPI)
    {
        spin_barrier(&s->lock);

        kill_timer(&s->timer);
    }
}

int domain_vpl031_init(struct domain *d)
{
    PL031State *s = &d->arch.vrtc;
    struct tm current_tm;

    spin_lock_init(&s->lock);

    spin_lock(&s->lock);
    register_mmio_handler(d, &vrtc_mmio_handler, GUEST_PL031_BASE, 0x1000, s);
    s->base = GUEST_PL031_BASE;
    s->d = d;
    s->irq = GUEST_VPL031_SPI;

    current_tm = gmtime(get_localtime(d));

    s->tick_offset = mktimegm(&current_tm) -
        get_localtime_us(d) / USEC_PER_SEC;

    init_timer(&s->timer, pl031_interrupt, s, smp_processor_id());
    spin_unlock(&s->lock);

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
