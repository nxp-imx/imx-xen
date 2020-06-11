/*
 * xen/arch/arm/tee/trusty.c
 *
 * Trusty mediator
 *
 * Peng Fan <peng.fan@nxp.com>
 * Copyright 2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <xen/device_tree.h>
#include <xen/domain_page.h>
#include <xen/sched.h>
#include <asm/smccc.h>
#include <asm/tee/tee.h>

#include <asm/tee/trusty/smcall.h>

#include "trusty-log.h"

void __flush_dcache_area(const void *vaddr, unsigned long size);
struct trusty_ipc_cmd_hdr {
    uint16_t opcode;
    uint16_t flags;
    uint32_t status;
    uint32_t handle;
    uint32_t payload_len;
    uint8_t  payload[0];
};

struct trusty_ipc_cmd_hdr *hdr;

#define MAX_STD_CALLS   16

/* Errors from the secure monitor */

/* Unknown SMC (defined by ARM DEN 0028A(0.9.0) */
#define SM_ERR_UNDEFINED_SMC 0xFFFFFFFF
#define SM_ERR_INVALID_PARAMETERS -2
/* Got interrupted. Call back with restart SMC */
#define SM_ERR_INTERRUPTED -3
/* Got an restart SMC when we didn't expect it */
#define SM_ERR_UNEXPECTED_RESTART -4
/* Temporarily busy. Call back with original args */
#define SM_ERR_BUSY -5
/* Got a trusted_service SMC when a restart SMC is required */
#define SM_ERR_INTERLEAVED_SMC -6
/* Unknown error */
#define SM_ERR_INTERNAL_FAILURE -7
#define SM_ERR_NOT_SUPPORTED -8
/* SMC call not allowed */
#define SM_ERR_NOT_ALLOWED -9
#define SM_ERR_END_OF_INPUT -10
/* Secure OS crashed */
#define SM_ERR_PANIC -11
/* Got interrupted by FIQ. Call back with SMC_SC_RESTART_FIQ on same CPU */
#define SM_ERR_FIQ_INTERRUPTED -12
/* SMC call waiting for another CPU */
#define SM_ERR_CPU_IDLE -13
/* Got interrupted. Call back with new SMC_SC_NOP */
#define SM_ERR_NOP_INTERRUPTED -14
/* Cpu idle after SMC_SC_NOP (not an error) */
#define SM_ERR_NOP_DONE -15

struct fw_rsc_hdr {
	u32 type;
	u8 data[0];
} __packed;

struct fw_rsc_vdev_vring {
	u32 da;
	u32 align;
	u32 num;
	u32 notifyid;
	u32 reserved;
} __packed;

struct fw_rsc_vdev {
	u32 id;
	u32 notifyid;
	u32 dfeatures;
	u32 gfeatures;
	u32 config_len;
	u8 status;
	u8 num_of_vrings;
	u8 reserved[2];
	struct fw_rsc_vdev_vring vring[0];
} __packed;

#define MAX_DEV_NAME_LEN 32
struct tipc_dev_config {
	u32 msg_buf_max_size;
	u32 msg_buf_alignment;
	char dev_name[MAX_DEV_NAME_LEN];
} __packed;

#define TIPC_VQ_NUM 2

struct resource_table {
	u32 ver;
	u32 num;
	u32 reserved[2];
	u32 offset[0];
} __packed;

struct tipc_vdev_descr {
    struct fw_rsc_hdr hdr;
    struct fw_rsc_vdev vdev;
    struct fw_rsc_vdev_vring vrings[TIPC_VQ_NUM];
    struct tipc_dev_config config;
} __packed;

typedef uint16_t __virtio16;
typedef uint32_t __virtio32;
typedef uint64_t __virtio64;

struct vring_desc {
	/* Address (guest-physical). */
	__virtio64 addr;
	/* Length. */
	__virtio32 len;
	/* The flags as indicated above. */
	__virtio16 flags;
	/* We chain unused descriptors via this, too */
	__virtio16 next;
};

struct vring_used_elem {
	/* Index of start of used descriptor chain. */
	__virtio32 id;
	/* Total length of the descriptor chain which was used (written to) */
	__virtio32 len;
};

static inline unsigned vring_size(unsigned int num, unsigned long align)
{
	return ((sizeof(struct vring_desc) * num + sizeof(__virtio16) * (3 + num)
		 + align - 1) & ~(align - 1))
		+ sizeof(__virtio16) * 3 + sizeof(struct vring_used_elem) * num;
}
/*
 * Call context. OP-TEE can issue multiple RPC returns during one call.
 * We need to preserve context during them.
 */
struct std_call_ctx {
    struct list_head list;
    void *guest_arg;
    void *xen_arg;
    void *descr;
    uint32_t size;
    mfn_t guest_arg_mfn;
    paddr_t xen_addr;
    int thread_id;
    int rpc_op;
    bool valid;
    bool uboot_stage;
    int num_of_vrings;
    /* Only support 2 now */
    //struct resource_table *descr;
    u64 guest_virtio_paddr[2];
    mfn_t guest_virtio_mfn[2];
    void *xen_virtio_addr[2];
    bool interrupted;
    bool need_copy_virtio;
    paddr_t gpaddr_orig[2][64];
};

struct domain_ctx {
    struct list_head list;
    struct list_head call_ctx_list;
    struct domain *domain;
    struct std_call_ctx *call;
    atomic_t call_ctx_count;
    spinlock_t lock;
};

static LIST_HEAD(domain_ctx_list);
static DEFINE_SPINLOCK(domain_ctx_list_lock);

static mfn_t lookup_and_pin_guest_ram_addr(paddr_t gaddr,
                                           struct page_info **pg)
{
    mfn_t mfn;
    gfn_t gfn;
    p2m_type_t t;
    struct page_info *page;
    struct domain *d = current->domain;

    gfn = gaddr_to_gfn(gaddr);
    mfn = p2m_lookup(d, gfn, &t);

    if ( t != p2m_ram_rw || mfn_eq(mfn, INVALID_MFN) )
        return INVALID_MFN;

    page = mfn_to_page(mfn);
    if ( !page )
        return INVALID_MFN;

    if ( !get_page(page, d) )
        return INVALID_MFN;

    if ( pg )
        *pg = page;

    return mfn;
}

static mfn_t lookup_guest_ram_addr(paddr_t gaddr, struct page_info **pg)
{
    mfn_t mfn;
    gfn_t gfn;
    p2m_type_t t;
    struct page_info *page;
    struct domain *d = current->domain;

    gfn = gaddr_to_gfn(gaddr);
    mfn = p2m_lookup(d, gfn, &t);

    if ( t != p2m_ram_rw || mfn_eq(mfn, INVALID_MFN) )
        return INVALID_MFN;

    page = mfn_to_page(mfn);
    if ( !page )
        return INVALID_MFN;

    if ( pg )
        *pg = page;

    return mfn;
}


static void unpin_guest_ram_addr(mfn_t mfn)
{
    struct page_info *page;
    page = mfn_to_page(mfn);
    if ( !page )
        return;

    put_page(page);
}

static struct domain_ctx *find_domain_ctx(struct domain* d)
{
    struct domain_ctx *ctx;

    spin_lock(&domain_ctx_list_lock);

    list_for_each_entry( ctx, &domain_ctx_list, list )
    {
        if ( ctx->domain == d )
        {
                spin_unlock(&domain_ctx_list_lock);
                return ctx;
        }
    }

    spin_unlock(&domain_ctx_list_lock);
    return NULL;
}

static bool trusty_probe(void)
{
    printk("%s\n", __func__);

    return true;
}

static int trusty_enabled;

static struct std_call_ctx *allocate_std_call_ctx(struct domain_ctx *ctx)
{
    struct std_call_ctx *call;
    int count;

    count = atomic_add_unless(&ctx->call_ctx_count, 1, MAX_STD_CALLS);
    if ( count == MAX_STD_CALLS )
        return NULL;

    call = xzalloc(struct std_call_ctx);
    if ( !call ) {
        atomic_dec(&ctx->call_ctx_count);
        return NULL;
    }

    call->thread_id = -1;
    call->interrupted = false;
    call->valid = false;

    spin_lock(&ctx->lock);
    list_add_tail(&call->list, &ctx->call_ctx_list);
    spin_unlock(&ctx->lock);

    ctx->call = call;

    return call;
}

static int trusty_domain_init(struct domain *d)
{
    struct domain_ctx *ctx;

    /* Only supports one domain use trusty */

    ctx = xzalloc(struct domain_ctx);
    if ( !ctx )
        return -ENOMEM;

    ctx->domain = d;
    INIT_LIST_HEAD(&ctx->call_ctx_list);

    atomic_set(&ctx->call_ctx_count, 0);
    spin_lock_init(&ctx->lock);

    ctx->call = allocate_std_call_ctx(ctx);
    if (!ctx->call)
    {
        xfree(ctx);
	return -ENOMEM;
    }

    spin_lock(&domain_ctx_list_lock);
    list_add_tail(&ctx->list, &domain_ctx_list);
    spin_unlock(&domain_ctx_list_lock);

#ifdef TRUSTY_DEBUG
    printk("%s %p %d %p\n", __func__, ctx, (int)d->domain_id, ctx->call);
#endif
    return 0;
}

static void forward_call(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    struct arm_smccc_res resp;
    unsigned long flags;
    register_t trusty_ret;
    register_t a0;
    register_t a1;
    register_t a2;
    register_t a3;

    a0 = get_user_reg(regs, 0);
    a1 = get_user_reg(regs, 1);
    a2 = get_user_reg(regs, 2);
    a3 = get_user_reg(regs, 3);

    set_user_reg(regs, 7, 0x5F);
#ifdef TRUSTY_DEBUG
    printk("%s %x\n", __func__, (int)get_user_reg(regs, 0));
#endif
    spin_lock_irqsave(&ctx->lock, flags);
    arm_smccc_smc(get_user_reg(regs, 0),
                  get_user_reg(regs, 1),
                  get_user_reg(regs, 2),
                  get_user_reg(regs, 3),
                  get_user_reg(regs, 0),
                  get_user_reg(regs, 1),
                  get_user_reg(regs, 2),
                  get_user_reg(regs, 7),
                  &resp);
    spin_unlock_irqrestore(&ctx->lock, flags);

    set_user_reg(regs, 0, resp.a0);
    set_user_reg(regs, 1, resp.a1);
    set_user_reg(regs, 2, resp.a2);
    set_user_reg(regs, 3, resp.a3);
    set_user_reg(regs, 4, 0);
    set_user_reg(regs, 5, 0);
    set_user_reg(regs, 6, 0);
    set_user_reg(regs, 7, 0);

    trusty_ret = get_user_reg(regs, 0);

#ifdef TRUSTY_DEBUG
    printk("%s %d\n", __func__, (int)resp.a0);
#endif
}

static void free_std_call_ctx(struct domain_ctx *ctx, struct std_call_ctx *call)
{
    atomic_dec(&ctx->call_ctx_count);

    spin_lock(&ctx->lock);
    list_del(&call->list);
    spin_unlock(&ctx->lock);

    if ( call->xen_arg )
    {
        free_xenheap_page(call->xen_arg);
        call->xen_arg = NULL;
    }

    if ( call->guest_arg ) {
        unmap_domain_page_global(call->guest_arg);
        unpin_guest_ram_addr(call->guest_arg_mfn);
        call->guest_arg = NULL;
    }

    xfree(call);
}

static bool copy_std_request_back(struct domain_ctx *ctx,
                                  struct cpu_user_regs *regs,
                                  struct std_call_ctx *call)
{
    memcpy(call->guest_arg, call->xen_arg, 4096);

    return true;
}

static bool execute_std_call(struct domain_ctx *ctx,
                             struct cpu_user_regs *regs,
                             struct std_call_ctx *call)
{
    register_t trusty_ret;

    memcpy(call->xen_arg, call->guest_arg, 4096);

    set_user_reg(regs, 1, call->xen_addr & 0xFFFFFFFF);
    set_user_reg(regs, 2, call->xen_addr >> 32);

#ifdef TRUSTY_DEBUG
    printk("%s %lx %x\n", __func__, get_user_reg(regs, 0), *(uint16_t *)call->xen_arg);

    {
        int i = 128;
        for (i = 0; i < 128; i += 4) {
	    printk("x %08x %08x\n", *(uint32_t *)((void *)call->xen_arg + i), *(uint32_t *)((void *)call->guest_arg + i));
	}
    }
#endif

    forward_call(ctx, regs);

    trusty_ret = get_user_reg(regs, 0);
    if ((trusty_ret == SM_ERR_FIQ_INTERRUPTED) ||
        (trusty_ret == SM_ERR_INTERRUPTED) ||
        (trusty_ret == SM_ERR_CPU_IDLE))
    {
#ifdef TRUSTY_DEBUG
        printk("%s: interrupt or cpuidle %ld\n", __func__, trusty_ret);
#endif
	call->interrupted = true;
        return true;
    }

    call->interrupted = false;

    copy_std_request_back(ctx, regs, call);

    return true;
}

#define RSC_DESCR_VER 1


__maybe_unused static int virtio_pre(struct domain_ctx *ctx, struct cpu_user_regs *regs, struct std_call_ctx *call, paddr_t (*gpaddr_orig)[64])
{
    struct resource_table *descr;
    int i, j, k;
    struct fw_rsc_hdr *hdr;
    struct fw_rsc_vdev *vd;
    struct fw_rsc_vdev_vring *vr;
    /* TODO: 64 should be enough for trusty, use more elegant code */

    descr = call->descr;

    if (!descr)
	    return 0;

    for (i = 0; i < descr->num; i++)
    {
        u32 offset = descr->offset[i];
        hdr = (struct fw_rsc_hdr *)((u8 *)descr + offset);
        offset += sizeof(struct fw_rsc_hdr);
        vd = (struct fw_rsc_vdev *)((u8 *)descr + offset);
#ifdef TRUSTY_DEBUG
	printk("desc num %d\n", vr->num);
#endif

	if (vd->num_of_vrings > 2)
		WARN_ON(1);

	for (j = 0; j < vd->num_of_vrings; j++)
        {
            paddr_t maddr;
            vr = &vd->vring[j];
	    if (vr->num > 64)
		    WARN_ON(1);

	    for (k = 0; k < vr->num; k++)
            {
                paddr_t gpaddr;
		__maybe_unused paddr_t pg_offset;
		mfn_t tmp;
		struct vring_desc *ab;
		pg_offset = 0;
                gpaddr = *(paddr_t *)((void *)call->xen_virtio_addr[j] + sizeof(struct vring_desc) * k);
		ab = (struct vring_desc *)((void *)call->xen_virtio_addr[j] + sizeof(struct vring_desc) * k);
		if (gpaddr & 0xfff)
			pg_offset = gpaddr & 0xfff;
		if (pg_offset) {
			printk("Wrong addr that might be not usable by Trusty, not aligned to 4KB, %lx %p\n", gpaddr, ab);
		}
		gpaddr_orig[j][k] = gpaddr;
		tmp = lookup_guest_ram_addr(gpaddr, NULL);
		if (mfn_eq(tmp, INVALID_MFN))
                {
#ifdef TRUSTY_DEBUG
		    printk("Not valide\n");
#endif
		    continue;
		}
                maddr = mfn_to_maddr(tmp) + pg_offset;
                *(paddr_t *)((void *)call->xen_virtio_addr[j] + sizeof(struct vring_desc) * k) = maddr;
#ifdef TRUSTY_DEBUG
		if (pg_offset)
			printk("pre gpaddr %d %lx %lx 0x%x %lx\n", k, gpaddr, maddr, ab->len, ab->addr);
#endif
	    }

	    __flush_dcache_area(call->xen_virtio_addr[j], sizeof(struct vring_desc) * vr->num);
	}
    }

    return 0;
}

__maybe_unused static int virtio_post(struct domain_ctx *ctx, struct cpu_user_regs *regs, struct std_call_ctx *call, paddr_t (*gpaddr_orig)[64])
{
    return 0;
}

static bool execute_virtio_kick_call(struct domain_ctx *ctx,
                                   struct cpu_user_regs *regs, struct std_call_ctx *call)
{
    register_t trusty_ret;

    virtio_pre(ctx, regs, call, call->gpaddr_orig);

    forward_call(ctx, regs);

    trusty_ret = get_user_reg(regs, 0);
    if ((trusty_ret == SM_ERR_FIQ_INTERRUPTED) ||
        (trusty_ret == SM_ERR_INTERRUPTED) ||
        (trusty_ret == SM_ERR_CPU_IDLE))
    {
        printk("%s: interrupt or cpuidle %ld\n", __func__, trusty_ret);
        call->interrupted = true;
        call->need_copy_virtio = true;
        return true;
    }

    virtio_post(ctx, regs, call, call->gpaddr_orig);

    call->interrupted = false;
    call->need_copy_virtio = false;

    return true;
}

static bool execute_vdev_kick_vq(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    register_t trusty_ret;
    struct std_call_ctx *call = ctx->call;

    forward_call(ctx, regs);

    trusty_ret = get_user_reg(regs, 0);
    if ((trusty_ret == SM_ERR_FIQ_INTERRUPTED) ||
        (trusty_ret == SM_ERR_INTERRUPTED) ||
        (trusty_ret == SM_ERR_CPU_IDLE))
    {
	call->interrupted = true;
        printk("%s: interrupt or cpuidle %ld\n", __func__, trusty_ret);
        return true;
    }
    call->interrupted = false;

    return true;
}

static bool execute_std_call_nomem_ignore_interrupt(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    struct std_call_ctx *call = ctx->call;

    forward_call(ctx, regs);

    call->interrupted = false;

    return true;
}

static bool smc_sc_restart_last(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    register_t trusty_ret;
    struct std_call_ctx *call = ctx->call;

#ifdef TRUSTY_DEBUG
    printk("================================ %d\n", call->need_copy_virtio);
#endif
    forward_call(ctx, regs);

    trusty_ret = get_user_reg(regs, 0);
    if ((trusty_ret == SM_ERR_FIQ_INTERRUPTED) ||
        (trusty_ret == SM_ERR_INTERRUPTED) ||
        (trusty_ret == SM_ERR_CPU_IDLE))
    {
#ifdef TRUSTY_DEBUG
        printk("%s: interrupt or cpuidle %ld\n", __func__, trusty_ret);
#endif
        return true;
    }

    if (call->interrupted)
    {
	if (call->need_copy_virtio)
            virtio_post(ctx, regs, call, call->gpaddr_orig);
	else if (call->uboot_stage)
	   copy_std_request_back(ctx, regs, call);
	/* TODO */
    }

    call->interrupted = false;
    call->need_copy_virtio = false;

    return true;
}

static int trusty_relinquish_resources(struct domain *d)
{
    struct domain_ctx *ctx;
    bool found = false;
    struct std_call_ctx *call;
    struct arm_smccc_res resp;

    /* Remove context from the list */
    spin_lock(&domain_ctx_list_lock);
    list_for_each_entry( ctx, &domain_ctx_list, list )
    {
        if ( ctx->domain == d )
        {
            found = true;
            list_del(&ctx->list);
            break;
        }
    }
    spin_unlock(&domain_ctx_list_lock);

    if ( !found )
        return 0;

    ASSERT(!spin_is_locked(&ctx->lock));

restart:
    call = ctx->call;
    if (call->valid)
    {
        if (call->uboot_stage)
        {
            arm_smccc_smc(SMC_SC_TRUSTY_IPC_SHUTDOWN_QL_DEV,
                          call->xen_addr & 0xFFFFFFFF, call->xen_addr >> 32,
                          call->size, 0, 0, 0, 0, &resp);
        } else {
                //printkk("Kernel stage destroy not supported\n");
        }

        if ((resp.a0 == SM_ERR_FIQ_INTERRUPTED) ||
            (resp.a0 == SM_ERR_INTERRUPTED) ||
            (resp.a0 == SM_ERR_CPU_IDLE))
        {
	    call->interrupted = true;
#ifdef TRUSTY_DEBUG
	    printk("%s: interrupt or cpuidle %ld\n", __func__, resp.a0);
#endif
            goto restart;
        }
    call->interrupted = false;
    }

    free_std_call_ctx(ctx, ctx->call);

    ASSERT(!atomic_read(&ctx->call_ctx_count));

    xfree(ctx);

    trusty_enabled = 0;

    return 0;
}

/*
 * Copy command buffer into xen memory to:
 * 1) Hide translated addresses from guest
 * 2) Make sure that guest wouldn't change data in command buffer during call
 */
#define NS_PTE_PHYSADDR(pte)       ((pte) & 0xFFFFFFFFF000ULL)
#define NS_PTE_MAIR_SHIFT           48
#define NS_PTE_SHAREABLE_SHIFT      8
#define NS_PTE_AP_SHIFT             6

static uint64_t par2attr(uint64_t par)
{
    uint64_t attr;

    /* set phys address */
    attr = NS_PTE_PHYSADDR(par);

    /* cache attributes */
    attr |= ((par >> 56) & 0xFF) << NS_PTE_MAIR_SHIFT;

    /* shareable attributes */
    attr |= ((par >> 7) & 0x03) << NS_PTE_SHAREABLE_SHIFT;

    /* the memory is writable and accessible so leave AP field 0 */
    attr |= 0x0 << NS_PTE_AP_SHIFT;

    return attr;
}

static bool create_ql_dev_request(struct cpu_user_regs *regs,
                                  struct std_call_ctx *call)
{
    paddr_t cmd_gaddr, xen_addr;
    uint64_t par;
    uint32_t size = get_user_reg(regs, 3);

    if (size > 4096)
    {
        printk("size too big: 0x%x\n", size);
        return false;
    }

    cmd_gaddr = (paddr_t)get_user_reg(regs, 1) |
        get_user_reg(regs, 2) << 32;

    cmd_gaddr = NS_PTE_PHYSADDR(cmd_gaddr);

    call->guest_arg_mfn = lookup_and_pin_guest_ram_addr(cmd_gaddr, NULL);
    if ( mfn_eq(call->guest_arg_mfn, INVALID_MFN) )
        return false;

    call->guest_arg = map_domain_page_global(call->guest_arg_mfn);
    if ( !call->guest_arg ) {
        unpin_guest_ram_addr(call->guest_arg_mfn);
        return false;
    }

    call->xen_arg = alloc_xenheap_page();
    if ( !call->xen_arg ) {
        unpin_guest_ram_addr(call->guest_arg_mfn);
        return false;
    }
    hdr = (void *)call->xen_arg;

    call->size = size;

    par = virt_to_maddr((vaddr_t)call->xen_arg);
    if (call->uboot_stage)
       xen_addr = par2attr(par);
    else
       xen_addr = 0xff000000000f13 | par;

    call->xen_addr = xen_addr;

    set_user_reg(regs, 1, xen_addr & 0xFFFFFFFF);
    set_user_reg(regs, 2, xen_addr >> 32);

    return true;
}

static bool handle_create_call(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    struct std_call_ctx *call;
    register_t trusty_ret;
    bool ret;

    call = ctx->call;

    if (call->uboot_stage)
    {
        ret = create_ql_dev_request(regs, call);
        if ( !ret )
            goto out;
    }
    else
    {
        ret = 0;
        goto out;
    }

    forward_call(ctx, regs);

    call->valid = true;

    trusty_ret = get_user_reg(regs, 0);
    if ((trusty_ret == SM_ERR_FIQ_INTERRUPTED) ||
        (trusty_ret == SM_ERR_INTERRUPTED) ||
        (trusty_ret == SM_ERR_CPU_IDLE))
    {
#ifdef TRUSTY_DEBUG
        printk("%s: interrupt or cpuidle %ld\n", __func__, trusty_ret);
#endif
	call->interrupted = true;
	return true;
    }
    call->interrupted = false;

out:
    return ret;
}

static bool handle_std_call(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    struct std_call_ctx *call;

    call = ctx->call;

    if (!call || !call->valid) {
	    printk("%s: %p %d\n", __func__, call, call->valid);
	    return false;
    }

    return execute_std_call(ctx, regs, call);
}

static bool handle_restart(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    struct std_call_ctx *call;

    call = ctx->call;

    if (!call || !call->valid)
	    return false;

    return execute_std_call(ctx, regs, call);
}

static bool handle_shutdown_call(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    register_t trusty_ret;
    struct std_call_ctx *call;

    call = ctx->call;

    execute_std_call(ctx, regs, call);

    trusty_ret = get_user_reg(regs, 0);
    if ((trusty_ret == SM_ERR_FIQ_INTERRUPTED) ||
        (trusty_ret == SM_ERR_INTERRUPTED) ||
        (trusty_ret == SM_ERR_CPU_IDLE))
    {
#ifdef TRUSTY_DEBUG
        printk("%s: interrupt or cpuidle %ld\n", __func__, trusty_ret);
#endif
	call->interrupted = true;
        return true;
    }

    call->interrupted = false;
    if ( call->xen_arg )
    {
        free_xenheap_page(call->xen_arg);
        call->xen_arg = NULL;
    }

    if ( call->descr)
    {
	unmap_domain_page_global(call->descr);
        call->descr = NULL;
    }

    if ( call->guest_arg ) {
        unmap_domain_page_global(call->guest_arg);
        unpin_guest_ram_addr(call->guest_arg_mfn);
        call->guest_arg = NULL;
    }

    call->valid = false;

    call->need_copy_virtio = false;

    return true;
}

static bool smc_sc_shared_log_add_rm(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    register_t a1 = get_user_reg(regs, 1);
    register_t a2 = get_user_reg(regs, 2);
    mfn_t mfn;
    paddr_t maddr;
    register_t trusty_ret;

    struct std_call_ctx *call = ctx->call;

    mfn = gfn_to_mfn(ctx->domain, gaddr_to_gfn(a2 << 32 | a1));
    if ( mfn_eq(mfn, INVALID_MFN) )
    {
	set_user_reg(regs, 0, -1);
	printk("%s: not valid mfn\n", __func__);
	return false;
    }

    maddr = mfn_to_maddr(mfn);

    set_user_reg(regs, 1, (u32)maddr);
    set_user_reg(regs, 2, (u32)(maddr >> 32));

    forward_call(ctx, regs);

    trusty_ret = get_user_reg(regs, 0);
    if ((trusty_ret == SM_ERR_FIQ_INTERRUPTED) ||
        (trusty_ret == SM_ERR_INTERRUPTED) ||
        (trusty_ret == SM_ERR_CPU_IDLE))
    {
        return true;
    }

    call->interrupted = false;

    return true;
}

static bool trusty_load_device_descr(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    paddr_t addr;
    u64 attr;
    mfn_t g_mfn;
    struct std_call_ctx *call = ctx->call;
    register_t trusty_ret;

    attr = (get_user_reg(regs, 2) << 32) | (get_user_reg(regs, 1));

    addr = NS_PTE_PHYSADDR(attr);

    g_mfn = lookup_and_pin_guest_ram_addr(addr, NULL);
    if ( mfn_eq(g_mfn, INVALID_MFN) )
        return false;

    addr = mfn_to_maddr(g_mfn);

    attr &= ~0xFFFFFFFFF000ULL;
    attr |= addr;

    set_user_reg(regs, 1, (u32)attr);
    set_user_reg(regs, 2, (u32)(attr >> 32));

    call->valid = true;
    forward_call(ctx, regs);

    trusty_ret = get_user_reg(regs, 0);
    if ((trusty_ret == SM_ERR_FIQ_INTERRUPTED) ||
        (trusty_ret == SM_ERR_INTERRUPTED) ||
        (trusty_ret == SM_ERR_CPU_IDLE))
    {
#ifdef TRUSTY_DEBUG
        printk("%s interrupted: ret %d\n", __func__, (int)trusty_ret);
#endif
	call->interrupted = true;
	call->need_copy_virtio = false;
        return true;
    }

    call->interrupted = false;
    call->need_copy_virtio = false;

    unpin_guest_ram_addr(g_mfn);
#ifdef TRUSTY_DEBUG
    printk("%s %lx %lx %lx\n", __func__, g_mfn, addr, attr);
#endif

    return true;
}

static bool trusty_virtio_start_call(struct domain_ctx *ctx, struct cpu_user_regs *regs, struct std_call_ctx *call)
{
    paddr_t addr;
    u64 attr;
    mfn_t g_mfn;
    struct resource_table *descr;
    int i, j;
    u64 p1;
    struct fw_rsc_hdr *hdr;
    struct fw_rsc_vdev *vd;
    struct fw_rsc_vdev_vring *vr;
    register_t trusty_ret;

    attr = (get_user_reg(regs, 2) << 32) | (get_user_reg(regs, 1));
    addr = NS_PTE_PHYSADDR(attr);

    g_mfn = lookup_guest_ram_addr(addr, NULL);
    if ( mfn_eq(g_mfn, INVALID_MFN) )
    {
        return false;
    }

    addr = mfn_to_maddr(g_mfn);

    attr &= ~0xFFFFFFFFF000ULL;
    attr |= addr;

    set_user_reg(regs, 1, (u32)attr);
    set_user_reg(regs, 2, (u32)(attr >> 32));

    descr = map_domain_page_global(g_mfn);
    if ( !descr )
    {
        return false;
    }

    call->descr = descr;

    if (descr->ver != RSC_DESCR_VER)
    {
        printk("Error DESCR VER %d\n", descr->ver);
	return false;
    }

    if (descr->num > 1)
    {
       printk("Not support num > 1\n");
       return false;
    }

    for (i = 0; i < descr->num; i++)
    {
        u32 offset = descr->offset[i];
	u32 cfg_len;
	struct tipc_dev_config *cfg;

        hdr = (struct fw_rsc_hdr *)((u8 *)descr + offset);
        offset += sizeof(struct fw_rsc_hdr);
        vd = (struct fw_rsc_vdev *)((u8 *)descr + offset);
        vr = vd->vring;
	cfg = (void *)(vr + vd->num_of_vrings);
	cfg_len = vd->config_len;

	for (j = 0; j < vd->num_of_vrings; j++)
        {
            paddr_t maddr;
	    paddr_t offset;
	    offset = 0;
            vr = &vd->vring[j];
            p1 = vr->da | ((u64)vr->reserved << 32);
	    /* TODO: size is 8KB, not 4KB,  if linux not physcal continus? */
	    call->guest_virtio_paddr[j] = vr->da | ((u64)vr->reserved << 32);
	    if (call->guest_virtio_paddr[j] & 0xfff)
		    offset = call->guest_virtio_paddr[j] & 0xfff;
	    if (offset)
		    continue;
	    call->guest_virtio_mfn[j] = lookup_guest_ram_addr(call->guest_virtio_paddr[j], NULL);
	    if ( mfn_eq(call->guest_arg_mfn, INVALID_MFN) )
            {
                printk("Not a valid guest mfn");
                return false;
	    }
	    call->xen_virtio_addr[j] = map_domain_page_global(call->guest_virtio_mfn[j]);
	    /* Make sure to unmap */
	    maddr = mfn_to_maddr(call->guest_virtio_mfn[j]) + offset;
	    vr->da = maddr & 0xffffffff;
	    vr->reserved = maddr >> 32;
	    //printk("--- %lx %lx\n", call->guest_virtio_paddr[j], maddr);
	}
    }

    if (get_user_reg(regs, 0) == 0x32000015) {
    }

    virtio_pre(ctx, regs, call, call->gpaddr_orig);
    forward_call(ctx, regs);

    trusty_ret = get_user_reg(regs, 0);
    if ((trusty_ret == SM_ERR_FIQ_INTERRUPTED) ||
        (trusty_ret == SM_ERR_INTERRUPTED) ||
        (trusty_ret == SM_ERR_CPU_IDLE))
    {
	call->interrupted = true;
	call->need_copy_virtio = true;
        return true;
    }

    virtio_post(ctx, regs, call, call->gpaddr_orig);
    call->interrupted = false;
    call->need_copy_virtio = false;

    //printk("gggggggggggggggggg\n");
    return true;
}

static bool handle_virtio_start_call(struct domain_ctx *ctx, struct cpu_user_regs *regs)
{
    struct std_call_ctx *call;

    call = ctx->call;

    if (!call || !call->valid) {
	    printk("%s: %p %d\n", __func__, call, call->valid);
	    return false;
    }

    return trusty_virtio_start_call(ctx, regs, call);
}

static bool trusty_handle_call(struct cpu_user_regs *regs)
{
    struct domain_ctx *ctx;

    ctx = find_domain_ctx(current->domain);
    if ( !ctx ) {
	printk("%s %lx: no domain\n", __func__, get_user_reg(regs, 0));
        return false;
    }
#ifdef TRUSTY_DEBUG
    printk("%s: %p, %d\n", __func__, ctx, (int)current->domain->domain_id);
    printk("%s %lx\n", __func__, get_user_reg(regs, 0));
#endif
    switch ( get_user_reg(regs, 0) )
    {
    case SMC_SC_RESTART_FIQ:
        return handle_restart(ctx, regs);
    case SMC_SC_TRUSTY_IPC_CREATE_QL_DEV:
	ctx->call->uboot_stage = true;
	return handle_create_call(ctx, regs);
    case SMC_SC_TRUSTY_IPC_HANDLE_QL_DEV_CMD:
        return handle_std_call(ctx, regs);
    case SMC_SC_TRUSTY_IPC_SHUTDOWN_QL_DEV:
	return handle_shutdown_call(ctx, regs);

    case SMC_SC_VIRTIO_GET_DESCR:
	ctx->call->uboot_stage = false;
	return trusty_load_device_descr(ctx, regs);
    case SMC_SC_VIRTIO_START:
        return handle_virtio_start_call(ctx, regs);
    case SMC_SC_VIRTIO_STOP:
	return execute_virtio_kick_call(ctx, regs, ctx->call);
    case SMC_SC_VDEV_RESET:
	return execute_std_call_nomem_ignore_interrupt(ctx, regs);
    case SMC_NC_VDEV_KICK_VQ:
        return execute_vdev_kick_vq(ctx, regs);
    case SMC_SC_VDEV_KICK_VQ:
	return execute_std_call_nomem_ignore_interrupt(ctx, regs);

    case SMC_SC_SHARED_LOG_VERSION:
	return execute_std_call_nomem_ignore_interrupt(ctx, regs);
    case SMC_SC_SHARED_LOG_RM:
    case SMC_SC_SHARED_LOG_ADD:
	return smc_sc_shared_log_add_rm(ctx, regs);
    case SMC_SC_SHARED_CONSOLE_CTL:
	return execute_std_call_nomem_ignore_interrupt(ctx, regs);

    case SMC_SC_NOP:
	//if (get_user_reg(regs, 1) == SMC_NC_VDEV_KICK_VQ)
	return execute_virtio_kick_call(ctx, regs, ctx->call);
    case (uint32_t)SMC_FC_API_VERSION:
    case (uint32_t)SMC_FC_GET_VERSION_STR:
    case (uint32_t)SMC_SC_LOCKED_NOP:
    case (uint32_t)SMC_FC_GET_NEXT_IRQ:
	return execute_std_call_nomem_ignore_interrupt(ctx, regs);
    case (uint32_t)SMC_SC_RESTART_LAST:
	return smc_sc_restart_last(ctx, regs);
    default:
	printk("unknown smc %s %lx\n", __func__, get_user_reg(regs, 0));
    }

    return false;
}

static const struct tee_mediator_ops trusty_ops =
{
    .probe = trusty_probe,
    .domain_init = trusty_domain_init,
    .relinquish_resources = trusty_relinquish_resources,
    .handle_call = trusty_handle_call,
};

REGISTER_TEE_MEDIATOR(trusty, "TRUSTY", XEN_DOMCTL_CONFIG_TEE_TRUSTY, &trusty_ops);

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
