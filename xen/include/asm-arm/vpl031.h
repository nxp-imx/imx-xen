/*
 * Virtual PL031 RTC
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _VPL031_H_
#define _VPL031_H_

#include <public/domctl.h>
#include <asm/vreg.h>
#include <xen/mm.h>

typedef struct PL031State {
    struct timer timer;
    uint32_t irq;

    /* Needed to preserve the tick_count across migration, even if the
     * absolute value of the rtc_clock is different on the source and
     * destination.
     */
    uint32_t tick_offset;

    uint32_t mr;
    uint32_t lr;
    uint32_t cr;
    uint32_t im;
    uint32_t is;
    uint32_t base;
    struct domain *d;
    spinlock_t lock;
} PL031State;

#ifdef CONFIG_VRTC_PL031
int domain_vpl031_init(struct domain *d);
void domain_vpl031_deinit(struct domain *d);
#else
static inline int domain_vpl031_init(struct domain *d)
{
    return -ENOSYS;
}

static inline void domain_vpl031_deinit(struct domain *d) { }
#endif
#endif  /* _VPL031_H_ */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
