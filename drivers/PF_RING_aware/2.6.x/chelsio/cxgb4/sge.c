/*
 * This file is part of the Chelsio T4 Ethernet driver.
 *
 * Copyright (C) 2005-2009 Chelsio Communications.  All rights reserved.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the LICENSE file included in this
 * release for licensing terms and conditions.
 */

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <linux/dma-mapping.h>
#include <net/ipv6.h>
#include <net/tcp.h>
#include "common.h"
#include "t4_regs.h"
#include "t4_regs_values.h"
#include "t4_msg.h"
#include "t4fw_interface.h"

#define HAVE_PF_RING

#ifdef HAVE_PF_RING
#include "../../../../kernel/linux/pf_ring.h"
#endif

/*
 * Rx buffer size for "packed" pages Free List buffers (multiple ingress
 * packets packed per page buffer).  We use largish buffers if possible but
 * settle for single pages under memory shortage.
 */
#if PAGE_SHIFT >= 16
# define FL_PG_ORDER 0
#else
# define FL_PG_ORDER (16 - PAGE_SHIFT)
#endif

/* RX_PULL_LEN should be <= RX_COPY_THRES */
#define RX_COPY_THRES    256
#define RX_PULL_LEN      128

/*
 * Main body length for sk_buffs used for Rx Ethernet packets with fragments.
 * Should be >= RX_PULL_LEN but possibly bigger to give pskb_may_pull some room.
 */
#define RX_PKT_SKB_LEN   512

/* Ethernet header padding prepended to RX_PKTs */
#define RX_PKT_PAD 2

/*
 * Max number of Tx descriptors we clean up at a time.  Should be modest as
 * freeing skbs isn't cheap and it happens while holding locks.  We just need
 * to free packets faster than they arrive, we eventually catch up and keep
 * the amortized cost reasonable.  Must be >= 2 * TXQ_STOP_THRES.
 */
#define MAX_TX_RECLAIM 16

/*
 * Max number of Rx buffers we replenish at a time.  Again keep this modest,
 * allocating buffers isn't cheap either.
 */
#define MAX_RX_REFILL 16U

/*
 * Period of the Rx queue check timer.  This timer is infrequent as it has
 * something to do only when the system experiences severe memory shortage.
 */
#define RX_QCHECK_PERIOD (HZ / 2)

/*
 * Period of the Tx queue check timer.
 */
#define TX_QCHECK_PERIOD (HZ / 2)

/*
 * Max number of Tx descriptors to be reclaimed by the Tx timer.
 */
#define MAX_TIMER_TX_RECLAIM 100

/*
 * Timer index used when Rx queues encounter severe memory shortage.
 */
#define NOMEM_TMR_IDX (SGE_NTIMERS - 1)

/*
 * Suspend an Ethernet Tx queue with fewer available descriptors than this.
 * This is the same as calc_tx_descs() for a TSO packet with
 * nr_frags == MAX_SKB_FRAGS.
 */
#define ETHTXQ_STOP_THRES \
	(1 + DIV_ROUND_UP((3 * MAX_SKB_FRAGS) / 2 + (MAX_SKB_FRAGS & 1), 8))

/*
 * Suspension threshold for non-Ethernet Tx queues.  We require enough room
 * for a full sized WR.
 */
#define TXQ_STOP_THRES (SGE_MAX_WR_LEN / sizeof(struct tx_desc))

/*
 * Max Tx descriptor space we allow for an Ethernet packet to be inlined
 * into a WR.
 */
#define MAX_IMM_TX_PKT_LEN 128

/*
 * Max size of a WR sent through a control Tx queue.
 */
#define MAX_CTRL_WR_LEN SGE_MAX_WR_LEN

enum {
	/* packet alignment in FL buffers */
	FL_ALIGN = L1_CACHE_BYTES < 32 ? 32 : L1_CACHE_BYTES,

	/* payload data is DMA'ed offset this far from the buffer start */
	FL_PKTSHIFT = 2,

	/* egress status entry size */
	STAT_LEN = L1_CACHE_BYTES > 64 ? 128 : 64
};

/*
 * Currently there are two types of coalesce WR. Type 0 needs 48 bytes per
 * packet (if one sgl is present) and type 1 needs 32 bytes. This means
 * that type 0 can fit a maximum of 10 packets per WR and type 1 can fit
 * 15 packets. We need to keep track of the skb pointers in a coalesce WR
 * to be able to free those skbs when we get completions back from the FW.
 * Allocating the maximum number of pointers in every tx desc is a waste
 * of memory resources so we only store 2 pointers per tx desc which should
 * be enough since a tx desc can only fit 2 packets in the best case
 * scenario where a packet needs 32 bytes.
 */
#define ETH_COALESCE_PKT_NUM 15
#define ETH_COALESCE_PKT_PER_DESC 2

struct tx_eth_coal_desc {
	struct sk_buff *skb[ETH_COALESCE_PKT_PER_DESC];
	struct ulptx_sgl *sgl[ETH_COALESCE_PKT_PER_DESC];
	int idx; 
};

struct tx_sw_desc {                /* SW state per Tx descriptor */
	struct sk_buff *skb;
	struct ulptx_sgl *sgl;
	struct tx_eth_coal_desc coalesce;
};

struct rx_sw_desc {                /* SW state per Rx descriptor */
	void *buf;                 /* struct page or sk_buff */
	dma_addr_t dma_addr;
};

/*
 * Rx buffer sizes for "useskbs" Free List buffers (one ingress packet per skb
 * buffer).  We currently only support two sizes for 1500- and 9000-byte MTUs.
 * We could easily support more but there doesn't seem to be much need for
 * that ...
 */
#define FL_MTU_OVERHEAD (FL_PKTSHIFT + ETH_HLEN + VLAN_HLEN)
#define FL_MTU_SMALL 1500
#define FL_MTU_LARGE 9000
#define FL_MTU_SMALL_BUFSIZE ALIGN(FL_MTU_SMALL + FL_MTU_OVERHEAD, FL_ALIGN)
#define FL_MTU_LARGE_BUFSIZE ALIGN(FL_MTU_LARGE + FL_MTU_OVERHEAD, FL_ALIGN)

/*
 * The low bits of rx_sw_desc.dma_addr have special meaning.  The hardware
 * uses these to specify the buffer size as an index into the SGE Free List
 * Buffer Size register array.  We also use one bit, when the buffer isn't
 * mapped to the hardware to indicate that the buffer isn't mapped.  (If we
 * ever need to support more than 8 different buffer sizes, then we'll need to
 * move that "Buffer Unmapped" flag into a separate "flags" word in the Rx
 * Software Descriptor.)
 */
enum {
	RX_BUF_FLAGS     = 0xf,   /* bottom four bits are special */
	RX_BUF_SIZE      = 0x7,   /* bottom three bits are for buf sizes */
	RX_UNMAPPED_BUF  = 0x8,   /* buffer is not mapped */

	RX_SMALL_PG_BUF  = 0x0,   /* small (PAGE_SIZE) page buffer */
	RX_LARGE_PG_BUF  = 0x1,   /* buffer large (FL_PG_ORDER) page buffer */

	RX_SMALL_MTU_BUF = 0x2,   /* small MTU buffer */
	RX_LARGE_MTU_BUF = 0x3,   /* large MTU buffer */
};

static inline dma_addr_t get_buf_addr(const struct rx_sw_desc *d)
{
	return d->dma_addr & ~(dma_addr_t)RX_BUF_FLAGS;
}

static inline bool is_buf_mapped(const struct rx_sw_desc *d)
{
	return !(d->dma_addr & RX_UNMAPPED_BUF);
}

/**
 *	txq_avail - return the number of available slots in a Tx queue
 *	@q: the Tx queue
 *
 *	Returns the number of descriptors in a Tx queue available to write new
 *	packets.
 */
static inline unsigned int txq_avail(const struct sge_txq *q)
{
	return q->size - 1 - q->in_use;
}

/**
 *	fl_cap - return the capacity of a free-buffer list
 *	@fl: the FL
 *
 *	Returns the capacity of a free-buffer list.  The capacity is less than
 *	the size because one descriptor needs to be left unpopulated, otherwise
 *	HW will think the FL is empty.
 */
static inline unsigned int fl_cap(const struct sge_fl *fl)
{
	return fl->size - 8;   /* 1 descriptor = 8 buffers */
}

/**
 *	fl_starving - return whether a Free List is starving.
 *	@adapter: pointer to the adapter
 *	@fl: the Free List
 *
 *	Tests specified Free List to see whether the number of buffers
 *	available to the hardware has falled below our "starvation"
 *	threshhold.
 */
static inline bool fl_starving(const struct adapter *adapter,
			       const struct sge_fl *fl)
{
	const struct sge *s = &adapter->sge;

	return fl->avail - fl->pend_cred <= s->fl_starve_thres;
}

static int map_skb(struct device *dev, const struct sk_buff *skb,
		   dma_addr_t *addr)
{
	const skb_frag_t *fp, *end;
	const struct skb_shared_info *si;

	*addr = dma_map_single(dev, skb->data, skb_headlen(skb), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *addr))
		goto out_err;

	si = skb_shinfo(skb);
	end = &si->frags[si->nr_frags];

	for (fp = si->frags; fp < end; fp++) {
		*++addr = dma_map_page(dev, fp->page, fp->page_offset, fp->size,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(dev, *addr))
			goto unwind;
	}
	return 0;

unwind:
	while (fp-- > si->frags)
		dma_unmap_page(dev, *--addr, fp->size, DMA_TO_DEVICE);

	dma_unmap_single(dev, addr[-1], skb_headlen(skb), DMA_TO_DEVICE);
out_err:
	return -ENOMEM;
}


static void unmap_skb(struct device *dev, const struct sk_buff *skb,
		      const dma_addr_t *addr)
{
	const skb_frag_t *fp, *end;
	const struct skb_shared_info *si;

	dma_unmap_single(dev, *addr++, skb_headlen(skb), DMA_TO_DEVICE);

	si = skb_shinfo(skb);
	end = &si->frags[si->nr_frags];
	for (fp = si->frags; fp < end; fp++)
		dma_unmap_page(dev, *addr++, fp->size, DMA_TO_DEVICE);
}

/**
 *	deferred_unmap_destructor - unmap a packet when it is freed
 *	@skb: the packet
 *
 *	This is the packet destructor used for Tx packets that need to remain
 *	mapped until they are freed rather than until their Tx descriptors are
 *	freed.
 */
static void deferred_unmap_destructor(struct sk_buff *skb)
{
	unmap_skb(skb->dev->dev.parent, skb, (dma_addr_t *)skb->head);
}

static void unmap_sgl(struct device *dev, const struct sk_buff *skb,
		      const struct ulptx_sgl *sgl, const struct sge_txq *q)
{
	const struct ulptx_sge_pair *p;
	unsigned int nfrags = skb_shinfo(skb)->nr_frags;

	if (likely(skb_headlen(skb)))
		dma_unmap_single(dev, be64_to_cpu(sgl->addr0), ntohl(sgl->len0),
				 DMA_TO_DEVICE);
	else {
		dma_unmap_page(dev, be64_to_cpu(sgl->addr0), ntohl(sgl->len0),
			       DMA_TO_DEVICE);
		nfrags--;
	}

	/*
	 * the complexity below is because of the possibility of a wrap-around
	 * in the middle of an SGL
	 */
	for (p = sgl->sge; nfrags >= 2; nfrags -= 2) {
		if (likely((u8 *)(p + 1) <= (u8 *)q->stat)) {
unmap:			dma_unmap_page(dev, be64_to_cpu(p->addr[0]),
				       ntohl(p->len[0]), DMA_TO_DEVICE);
			dma_unmap_page(dev, be64_to_cpu(p->addr[1]),
				       ntohl(p->len[1]), DMA_TO_DEVICE);
			p++;
		} else if ((u8 *)p == (u8 *)q->stat) {
			p = (const struct ulptx_sge_pair *)q->desc;
			goto unmap;
		} else if ((u8 *)p + 8 == (u8 *)q->stat) {
			const __be64 *addr = (const __be64 *)q->desc;

			dma_unmap_page(dev, be64_to_cpu(addr[0]),
				       ntohl(p->len[0]), DMA_TO_DEVICE);
			dma_unmap_page(dev, be64_to_cpu(addr[1]),
				       ntohl(p->len[1]), DMA_TO_DEVICE);
			p = (const struct ulptx_sge_pair *)&addr[2];
		} else {
			const __be64 *addr = (const __be64 *)q->desc;

			dma_unmap_page(dev, be64_to_cpu(p->addr[0]),
				       ntohl(p->len[0]), DMA_TO_DEVICE);
			dma_unmap_page(dev, be64_to_cpu(addr[0]),
				       ntohl(p->len[1]), DMA_TO_DEVICE);
			p = (const struct ulptx_sge_pair *)&addr[1];
		}
	}
	if (nfrags) {
		__be64 addr;

		if ((u8 *)p == (u8 *)q->stat)
			p = (const struct ulptx_sge_pair *)q->desc;
		addr = (u8 *)p + 16 <= (u8 *)q->stat ? p->addr[0] :
						       *(const __be64 *)q->desc;
		dma_unmap_page(dev, be64_to_cpu(addr), ntohl(p->len[0]),
			       DMA_TO_DEVICE);
	}
}

/**
 *	 need_skb_unmap - does the platform need unmapping of sk_buffs?
 *
 *	Returns true if the platfrom needs sk_buff unmapping.  The compiler
 *	optimizes away unecessary code if this returns true.
 */
static inline int need_skb_unmap(void)
{
	/*
	 * This structure is used to tell if the platfrom needs buffer
	 * unmapping by checking if DECLARE_PCI_UNMAP_ADDR defines anything.
	 */
	struct dummy {
		DECLARE_PCI_UNMAP_ADDR(addr);
	};

	return sizeof(struct dummy) != 0;
}

/**
 *	free_tx_desc - reclaims Tx descriptors and their buffers
 *	@adapter: the adapter
 *	@q: the Tx queue to reclaim descriptors from
 *	@n: the number of descriptors to reclaim
 *	@unmap: whether the buffers should be unmapped for DMA
 *
 *	Reclaims Tx descriptors from an SGE Tx queue and frees the associated
 *	Tx buffers.  Called with the Tx queue lock held.
 */
static void free_tx_desc(struct adapter *adap, struct sge_txq *q,
			 unsigned int n, bool unmap)
{
	struct tx_sw_desc *d;
	unsigned int cidx = q->cidx;
	struct device *dev = adap->pdev_dev;
	int i;

	const int need_unmap = need_skb_unmap() && unmap;

#ifdef T4_TRACE
	T4_TRACE2(adap->tb[q->cntxt_id & 7],
		  "reclaiming %u Tx descriptors at cidx %u", n, cidx);
#endif
	d = &q->sdesc[cidx];
	while (n--) {
		if (d->skb) {                       /* an SGL is present */
			if (need_unmap)
				unmap_sgl(dev, d->skb, d->sgl, q);
			kfree_skb(d->skb);
			d->skb = NULL;
		}
		if (d->coalesce.idx) {
			for (i = 0; i < d->coalesce.idx; i++) {
				if (need_unmap)
					unmap_sgl(dev, d->coalesce.skb[i],
						  d->coalesce.sgl[i], q);
				kfree_skb(d->coalesce.skb[i]);
				d->coalesce.skb[i] = NULL;
			}
			d->coalesce.idx = 0;
		}
		++d;
		if (++cidx == q->size) {
			cidx = 0;
			d = q->sdesc;
		}
	}
	q->cidx = cidx;
}

/*
 * Return the number of reclaimable descriptors in a Tx queue.
 */
static inline int reclaimable(const struct sge_txq *q)
{
	int hw_cidx = ntohs(q->stat->cidx);
	hw_cidx -= q->cidx;
	return hw_cidx < 0 ? hw_cidx + q->size : hw_cidx;
}

/**
 *	reclaim_completed_tx - reclaims completed Tx descriptors
 *	@adap: the adapter
 *	@q: the Tx queue to reclaim completed descriptors from
 *	@unmap: whether the buffers should be unmapped for DMA
 *
 *	Reclaims Tx descriptors that the SGE has indicated it has processed,
 *	and frees the associated buffers if possible.  Called with the Tx
 *	queue locked.
 */
static inline void reclaim_completed_tx(struct adapter *adap, struct sge_txq *q,
					bool unmap)
{
	int avail = reclaimable(q);

	if (avail) {
		/*
		 * Limit the amount of clean up work we do at a time to keep
		 * the Tx lock hold time O(1).
		 */
		if (avail > MAX_TX_RECLAIM)
			avail = MAX_TX_RECLAIM;

		free_tx_desc(adap, q, avail, unmap);
		q->in_use -= avail;
	}
}

static inline int get_buf_size(const struct rx_sw_desc *d)
{
	unsigned int rx_buf_size_idx = d->dma_addr & RX_BUF_SIZE;
	static int rx_buf_size[] = {
		[RX_SMALL_PG_BUF] = PAGE_SIZE,
		[RX_LARGE_PG_BUF] = PAGE_SIZE << FL_PG_ORDER,
		[RX_SMALL_MTU_BUF] = FL_MTU_SMALL_BUFSIZE,
		[RX_LARGE_MTU_BUF] = FL_MTU_LARGE_BUFSIZE,
	};

	BUG_ON(rx_buf_size_idx >= ARRAY_SIZE(rx_buf_size));
	return rx_buf_size[rx_buf_size_idx];
}

/**
 *	free_rx_bufs - free the Rx buffers on an SGE free list
 *	@adap: the adapter
 *	@q: the SGE free list to free buffers from
 *	@n: how many buffers to free
 *
 *	Release the next @n buffers on an SGE free-buffer Rx queue.   The
 *	buffers must be made inaccessible to HW before calling this function.
 */
static void free_rx_bufs(struct adapter *adap, struct sge_fl *q, int n)
{
	struct sge_eth_rxq *rxq = container_of(q, struct sge_eth_rxq, fl);

	while (n--) {
		struct rx_sw_desc *d = &q->sdesc[q->cidx];

		if (unlikely(rxq->useskbs)) {
			if (is_buf_mapped(d))
				dma_unmap_single(adap->pdev_dev,
						 get_buf_addr(d),
						 get_buf_size(d),
						 PCI_DMA_FROMDEVICE);
			kfree_skb(d->buf);
		} else {
			if (is_buf_mapped(d))
				dma_unmap_page(adap->pdev_dev, get_buf_addr(d),
					       get_buf_size(d),
					       PCI_DMA_FROMDEVICE);
			put_page(d->buf);
		}

		d->buf = NULL;
		if (++q->cidx == q->size)
			q->cidx = 0;
		q->avail--;
	}
}

/**
 *	unmap_rx_buf - unmap the current Rx buffer on an SGE free list
 *	@adap: the adapter
 *	@q: the SGE free list
 *
 *	Unmap the current buffer on an SGE free-buffer Rx queue.   The
 *	buffer must be made inaccessible to HW before calling this function.
 *
 *	This is similar to @free_rx_bufs above but does not free the buffer.
 *	Do note that the FL still loses any further access to the buffer.
 */
static void unmap_rx_buf(struct adapter *adap, struct sge_fl *q)
{
	struct sge_eth_rxq *rxq = container_of(q, struct sge_eth_rxq, fl);
	struct rx_sw_desc *d = &q->sdesc[q->cidx];

	if (unlikely(rxq->useskbs)) {
		if (is_buf_mapped(d))
			dma_unmap_page(adap->pdev_dev, get_buf_addr(d),
				       get_buf_size(d), PCI_DMA_FROMDEVICE);
	} else {
		if (is_buf_mapped(d))
			dma_unmap_page(adap->pdev_dev, get_buf_addr(d),
				       get_buf_size(d), PCI_DMA_FROMDEVICE);
	}

	if (++q->cidx == q->size)
		q->cidx = 0;
	q->avail--;
}

static inline void ring_fl_db(struct adapter *adap, struct sge_fl *q)
{
	if (q->pend_cred >= 8) {
		wmb();
		t4_write_reg(adap, MYPF_REG(A_SGE_PF_KDOORBELL), F_DBPRIO |
			     V_QID(q->cntxt_id) | V_PIDX(q->pend_cred / 8));
		q->pend_cred &= 7;
	}
}

static inline void set_rx_sw_desc(struct rx_sw_desc *sd, void *buf,
				  dma_addr_t mapping)
{
	sd->buf = buf;
	sd->dma_addr = mapping;      /* includes size low bits */
}

#define POISON_BUF_VAL -1

static inline void poison_buf(struct page *pg, size_t sz)
{
#if POISON_BUF_VAL >= 0
	memset(page_address(pg), POISON_BUF_VAL, sz);
#endif
}

/**
 *	refill_fl_usepages - refill an SGE Rx buffer ring with pages
 *	@adap: the adapter
 *	@q: the ring to refill
 *	@n: the number of new buffers to allocate
 *	@gfp: the gfp flags for the allocations
 *
 *	(Re)populate an SGE free-buffer queue with up to @n new packet buffers,
 *	allocated with the supplied gfp flags.  The caller must assure that
 *	@n does not exceed the queue's capacity.  If afterwards the queue is
 *	found critically low mark it as starving in the bitmap of starving FLs.
 *
 *	Returns the number of buffers allocated.
 */
static unsigned int refill_fl_usepages(struct adapter *adap, struct sge_fl *q,
				       int n, gfp_t gfp)
{
	struct page *pg;
	dma_addr_t mapping;
	unsigned int cred = q->avail;
	__be64 *d = &q->desc[q->pidx];
	struct rx_sw_desc *sd = &q->sdesc[q->pidx];

	if (test_bit(q->cntxt_id - adap->sge.egr_start, adap->sge.blocked_fl))
		goto out;

	gfp |= __GFP_NOWARN;         /* failures are expected */

#if FL_PG_ORDER > 0
	/*
	 * Prefer large buffers
	 */
	while (n) {
		pg = alloc_pages(gfp | __GFP_COMP, FL_PG_ORDER);
		if (unlikely(!pg)) {
			q->large_alloc_failed++;
			break;       /* fall back to single pages */
		}

		poison_buf(pg, PAGE_SIZE << FL_PG_ORDER);

		mapping = dma_map_page(adap->pdev_dev, pg, 0,
				       PAGE_SIZE << FL_PG_ORDER,
				       PCI_DMA_FROMDEVICE);
		if (unlikely(dma_mapping_error(adap->pdev_dev, mapping))) {
			__free_pages(pg, FL_PG_ORDER);
			goto out;   /* do not try small pages for this error */
		}
		mapping |= RX_LARGE_PG_BUF;
		*d++ = cpu_to_be64(mapping);

		set_rx_sw_desc(sd, pg, mapping);
		sd++;

		q->avail++;
		if (++q->pidx == q->size) {
			q->pidx = 0;
			sd = q->sdesc;
			d = q->desc;
		}
		n--;
	}
#endif

	while (n--) {
		pg = __netdev_alloc_page(adap->port[0], gfp);
		if (unlikely(!pg)) {
			q->alloc_failed++;
			break;
		}

		poison_buf(pg, PAGE_SIZE);

		mapping = dma_map_page(adap->pdev_dev, pg, 0, PAGE_SIZE,
				       PCI_DMA_FROMDEVICE);
		if (unlikely(dma_mapping_error(adap->pdev_dev, mapping))) {
			netdev_free_page(adap->port[0], pg);
			break;
		}
		mapping |= RX_SMALL_PG_BUF;
		*d++ = cpu_to_be64(mapping);

		set_rx_sw_desc(sd, pg, mapping);
		sd++;

		q->avail++;
		if (++q->pidx == q->size) {
			q->pidx = 0;
			sd = q->sdesc;
			d = q->desc;
		}
	}

out:	cred = q->avail - cred;
	q->pend_cred += cred;
	ring_fl_db(adap, q);

	if (unlikely(fl_starving(adap, q))) {
		smp_wmb();
		set_bit(q->cntxt_id - adap->sge.egr_start,
			adap->sge.starving_fl);
	}

	return cred;
}

/**
 *	refill_fl_useskbs - refill an SGE Rx buffer ring with skbs
 *	@adap: the adapter
 *	@q: the ring to refill
 *	@n: the number of new buffers to allocate
 *	@gfp: the gfp flags for the allocations
 *
 *	(Re)populate an SGE free-buffer queue with up to @n new packet buffers,
 *	allocated with the supplied gfp flags.  The caller must assure that
 *	@n does not exceed the queue's capacity.  If afterwards the queue is
 *	found critically low mark it as starving in the bitmap of starving FLs.
 *
 *	Returns the number of buffers allocated.
 */
static unsigned int refill_fl_useskbs(struct adapter *adap, struct sge_fl *q,
				      int n, gfp_t gfp)
{
	struct sge_eth_rxq *rxq = container_of(q, struct sge_eth_rxq, fl);
	struct net_device *dev = rxq->rspq.netdev;
	unsigned int mtu = dev->mtu;
	unsigned int buf_size, buf_size_idx, skb_size;
	unsigned int cred = q->avail;
	__be64 *d = &q->desc[q->pidx];
	struct rx_sw_desc *sd = &q->sdesc[q->pidx];

	gfp |= __GFP_NOWARN;         /* failures are expected */

	/*
	 * Figure out what skb buffer size and corresponding T4 SGE FL Buffer
	 * Size index we'll be using based on the device's current MTU.  We
	 * need to allocate an extra (FL_ALIGN-1) bytes in order to be able to
	 * create an aligned address (and leave the bottom bits available for
	 * our flags and buffer size indices).
	 */
	if (mtu <= FL_MTU_SMALL) {
		buf_size = FL_MTU_SMALL_BUFSIZE;
		buf_size_idx = RX_SMALL_MTU_BUF;
	} else {
		buf_size = FL_MTU_LARGE_BUFSIZE;
		buf_size_idx = RX_LARGE_MTU_BUF;
	}
	skb_size = buf_size + FL_ALIGN - 1;

	while (n--) {
		struct sk_buff *skb = alloc_skb(skb_size, gfp);
		void *buf_start;
		dma_addr_t mapping;

		if (unlikely(!skb))
			goto out;

		buf_start = PTR_ALIGN(skb->data, FL_ALIGN);
		if (buf_start != skb->data)
			skb_reserve(skb,
				   (__kernel_ptrdiff_t)buf_start -
				   (__kernel_ptrdiff_t)skb->data);

		mapping = dma_map_single(adap->pdev_dev, buf_start, buf_size,
					 PCI_DMA_FROMDEVICE);
		if (unlikely(dma_mapping_error(adap->pdev_dev, mapping))) {
			kfree_skb(skb);
			goto out;
		}

		mapping |= buf_size_idx;
		*d++ = cpu_to_be64(mapping);

		set_rx_sw_desc(sd, skb, mapping);
		sd++;

		q->avail++;
		if (++q->pidx == q->size) {
			q->pidx = 0;
			sd = q->sdesc;
			d = q->desc;
		}
	}

out:	cred = q->avail - cred;
	q->pend_cred += cred;
	ring_fl_db(adap, q);

	if (unlikely(fl_starving(adap, q))) {
		smp_wmb();
		set_bit(q->cntxt_id - adap->sge.egr_start,
			adap->sge.starving_fl);
	}

	return cred;
}

/**
 *	refill_fl - refill an SGE Rx buffer ring with skbs
 *	@adap: the adapter
 *	@q: the ring to refill
 *	@n: the number of new buffers to allocate
 *	@gfp: the gfp flags for the allocations
 *
 *	(Re)populate an SGE free-buffer queue with up to @n new packet buffers,
 *	allocated with the supplied gfp flags.  The caller must assure that
 *	@n does not exceed the queue's capacity.  Returns the number of buffers
 *	allocated.
 */
static unsigned int refill_fl(struct adapter *adap, struct sge_fl *q, int n,
			      gfp_t gfp)
{
	struct sge_eth_rxq *rxq = container_of(q, struct sge_eth_rxq, fl);

	return (unlikely(rxq->useskbs)
		? refill_fl_useskbs(adap, q, n, gfp)
		: refill_fl_usepages(adap, q, n, gfp));
}

static inline void __refill_fl(struct adapter *adap, struct sge_fl *fl)
{
	refill_fl(adap, fl, min(MAX_RX_REFILL, fl_cap(fl) - fl->avail),
		  GFP_ATOMIC);
}

/**
 *	alloc_ring - allocate resources for an SGE descriptor ring
 *	@dev: the PCI device's core device
 *	@nelem: the number of descriptors
 *	@elem_size: the size of each descriptor
 *	@sw_size: the size of the SW state associated with each ring element
 *	@phys: the physical address of the allocated ring
 *	@metadata: address of the array holding the SW state for the ring
 *	@stat_size: extra space in HW ring for status information
 *
 *	Allocates resources for an SGE descriptor ring, such as Tx queues,
 *	free buffer lists, or response queues.  Each SGE ring requires
 *	space for its HW descriptors plus, optionally, space for the SW state
 *	associated with each HW entry (the metadata).  The function returns
 *	three values: the virtual address for the HW ring (the return value
 *	of the function), the bus address of the HW ring, and the address
 *	of the SW ring.
 */
static void *alloc_ring(struct device *dev, size_t nelem, size_t elem_size,
			size_t sw_size, dma_addr_t *phys, void *metadata,
			size_t stat_size)
{
	size_t len = nelem * elem_size + stat_size;
	void *s = NULL;
	void *p = dma_alloc_coherent(dev, len, phys, GFP_KERNEL);

	if (!p)
		return NULL;
	if (sw_size) {
		s = kcalloc(nelem, sw_size, GFP_KERNEL);

		if (!s) {
			dma_free_coherent(dev, len, p, *phys);
			return NULL;
		}
	}
	if (metadata)
		*(void **)metadata = s;
	memset(p, 0, len);
	return p;
}

/**
 *	sgl_len - calculates the size of an SGL of the given capacity
 *	@n: the number of SGL entries
 *
 *	Calculates the number of flits needed for a scatter/gather list that
 *	can hold the given number of entries.
 */
static inline unsigned int sgl_len(unsigned int n)
{
	/*
	 * A Direct Scatter Gather List uses 32-bit lengths and 64-bit PCI DMA
	 * addresses.  The DSGL Work Request starts off with a 32-bit DSGL
	 * ULPTX header, then Length0, then Address0, then, for 1 <= i <= N,
	 * repeated sequences of { Length[i], Length[i+1], Address[i],
	 * Address[i+1] } (this ensures that all addresses are on 64-bit
	 * boundaries).  If N is even, then Length[N+1] should be set to 0 and
	 * Address[N+1] is omitted.
	 *
	 * The following calculation incorporates all of the above.  It's
	 * somewhat hard to follow but, briefly: the "+2" accounts for the
	 * first two flits which include the DSGL header, Length0 and
	 * Address0; the "(3*(n-1))/2" covers the main body of list entries (3
	 * flits for every pair of the remaining N) +1 if (n-1) is odd; and
	 * finally the "+((n-1)&1)" adds the one remaining flit needed if
	 * (n-1) is odd ...
	 */
	n--;
	return (3 * n) / 2 + (n & 1) + 2;
}

/**
 *	flits_to_desc - returns the num of Tx descriptors for the given flits
 *	@n: the number of flits
 *
 *	Returns the number of Tx descriptors needed for the supplied number
 *	of flits.
 */
static inline unsigned int flits_to_desc(unsigned int n)
{
	BUG_ON(n > SGE_MAX_WR_LEN / 8);
	return DIV_ROUND_UP(n, 8);
}

/**
 *	is_eth_imm - can an Ethernet packet be sent as immediate data?
 *	@skb: the packet
 *
 *	Returns whether an Ethernet packet is small enough to fit as
 *	immediate data.
 */
static inline int is_eth_imm(const struct sk_buff *skb)
{
	return skb->len <= MAX_IMM_TX_PKT_LEN - sizeof(struct cpl_tx_pkt);
}

/**
 *	calc_tx_flits - calculate the number of flits for a packet Tx WR
 *	@skb: the packet
 *
 * 	Returns the number of flits needed for a Tx WR for the given Ethernet
 * 	packet, including the needed WR and CPL headers.
 */
static inline unsigned int calc_tx_flits(const struct sk_buff *skb)
{
	unsigned int flits;

	/*
	 * If the skb is small enough, we can pump it out as a work request
	 * with only immediate data.  In that case we just have to have the
	 * TX Packet header plus the skb data in the Work Request.
	 */
	if (is_eth_imm(skb))
		return DIV_ROUND_UP(skb->len + sizeof(struct cpl_tx_pkt), 8);

	/*
	 * Otherwise, we're going to have to construct a Scatter gather list
	 * of the skb body and fragments.  We also include the flits necessary
	 * for the TX Packet Work Request and CPL.  We always have a firmware
	 * Write Header (incorporated as part of the cpl_tx_pkt_lso and
	 * cpl_tx_pkt structures), followed by either a TX Packet Write CPL
	 * message or, if we're doing a Large Send Offload, an LSO CPL message
	 * with an embeded TX Packet Write CPL message.
	 */
	flits = sgl_len(skb_shinfo(skb)->nr_frags + 1);
	if (skb_shinfo(skb)->gso_size)
		flits += (sizeof(struct fw_eth_tx_pkt_wr) +
			  sizeof(struct cpl_tx_pkt_lso_core) +
			  sizeof(struct cpl_tx_pkt_core)) / sizeof(__be64);
	else
		flits += (sizeof(struct fw_eth_tx_pkt_wr) +
			  sizeof(struct cpl_tx_pkt_core)) / sizeof(__be64);
	return flits;
}

/**
 *	calc_tx_descs - calculate the number of Tx descriptors for a packet
 *	@skb: the packet
 *
 * 	Returns the number of Tx descriptors needed for the given Ethernet
 * 	packet, including the needed WR and CPL headers.
 */
static inline unsigned int calc_tx_descs(const struct sk_buff *skb)
{
	return flits_to_desc(calc_tx_flits(skb));
}

/**
 *	write_sgl - populate a scatter/gather list for a packet
 *	@skb: the packet
 *	@q: the Tx queue we are writing into
 *	@sgl: starting location for writing the SGL
 *	@end: points right after the end of the SGL
 *	@start: start offset into skb main-body data to include in the SGL
 *
 *	Generates a scatter/gather list for the buffers that make up a packet.
 *	The caller must provide adequate space for the SGL that will be written.
 *	The SGL includes all of the packet's page fragments and the data in its
 *	main body except for the first @start bytes.  @sgl must be 16-byte
 *	aligned and within a Tx descriptor with available space.  @end points
 *	write after the end of the SGL but does not account for any potential
 *	wrap around, i.e., @end > @sgl.
 */
static void write_sgl(const struct sk_buff *skb, struct sge_txq *q,
		      struct ulptx_sgl *sgl, u64 *end, unsigned int start,
		      const dma_addr_t *addr)
{
	unsigned int i, len;
	struct ulptx_sge_pair *to;
	const struct skb_shared_info *si = skb_shinfo(skb);
	unsigned int nfrags = si->nr_frags;
	struct ulptx_sge_pair buf[MAX_SKB_FRAGS / 2 + 1];

	len = skb_headlen(skb) - start;
	if (likely(len)) {
		sgl->len0 = htonl(len);
		sgl->addr0 = cpu_to_be64(addr[0] + start);
		nfrags++;
	} else {
		sgl->len0 = htonl(si->frags[0].size);
		sgl->addr0 = cpu_to_be64(addr[1]);
	}

	sgl->cmd_nsge = htonl(V_ULPTX_CMD(ULP_TX_SC_DSGL) |
			      V_ULPTX_NSGE(nfrags));
	if (likely(--nfrags == 0))
		return;
	/*
	 * Most of the complexity below deals with the possibility we hit the
	 * end of the queue in the middle of writing the SGL.  For this case
	 * only we create the SGL in a temporary buffer and then copy it.
	 */
	to = (u8 *)end > (u8 *)q->stat ? buf : sgl->sge;

	for (i = (nfrags != si->nr_frags); nfrags >= 2; nfrags -= 2, to++) {
		to->len[0] = cpu_to_be32(si->frags[i].size);
		to->len[1] = cpu_to_be32(si->frags[++i].size);
		to->addr[0] = cpu_to_be64(addr[i]);
		to->addr[1] = cpu_to_be64(addr[++i]);
	}
	if (nfrags) {
		to->len[0] = cpu_to_be32(si->frags[i].size);
		to->len[1] = cpu_to_be32(0);
		to->addr[0] = cpu_to_be64(addr[i + 1]);
	}
	if (unlikely((u8 *)end > (u8 *)q->stat)) {
		unsigned int part0 = (u8 *)q->stat - (u8 *)sgl->sge, part1;

		if (likely(part0))
			memcpy(sgl->sge, buf, part0);
		part1 = (u8 *)end - (u8 *)q->stat;
		memcpy(q->desc, (u8 *)buf + part0, part1);
		end = (void *)q->desc + part1;
	}
	if ((uintptr_t)end & 8)           /* 0-pad to multiple of 16 */
		*(u64 *)end = 0;
}

/**
 *	ring_tx_db - check and potentially ring a Tx queue's doorbell
 *	@adap: the adapter
 *	@q: the Tx queue
 *	@n: number of new descriptors to give to HW
 *
 *	Ring the doorbel for a Tx queue.
 */
static inline void ring_tx_db(struct adapter *adap, struct sge_txq *q, int n)
{
	WARN_ON((V_QID(q->cntxt_id) | V_PIDX(n)) & F_DBPRIO);
	wmb();            /* write descriptors before telling HW */
	t4_write_reg(adap, MYPF_REG(A_SGE_PF_KDOORBELL),
		     V_QID(q->cntxt_id) | V_PIDX(n));
}

/**
 * 	inline_tx_skb - inline a packet's data into Tx descriptors
 * 	@skb: the packet
 * 	@q: the Tx queue where the packet will be inlined
 * 	@pos: starting position in the Tx queue where to inline the packet
 *
 *	Inline a packet's contents directly into Tx descriptors, starting at
 *	the given position within the Tx DMA ring.
 *	Most of the complexity of this operation is dealing with wrap arounds
 *	in the middle of the packet we want to inline.
 */
static void inline_tx_skb(const struct sk_buff *skb, const struct sge_txq *q,
			  void *pos)
{
	u64 *p;
	int left = (void *)q->stat - pos;

	if (likely(skb->len <= left)) {
		if (likely(!skb->data_len))
			skb_copy_from_linear_data(skb, pos, skb->len);
		else
			skb_copy_bits(skb, 0, pos, skb->len);
		pos += skb->len;
	} else {
		skb_copy_bits(skb, 0, pos, left);
		skb_copy_bits(skb, left, q->desc, skb->len - left);
		pos = (void *)q->desc + (skb->len - left);
	}

	/* 0-pad to multiple of 16 */
	p = PTR_ALIGN(pos, 8);
	if ((uintptr_t)p & 8)
		*p = 0;
}

/*
 * Figure out what HW csum a packet wants and return the appropriate control
 * bits.
 */
static u64 hwcsum(const struct sk_buff *skb)
{
	int csum_type;
	const struct iphdr *iph = ip_hdr(skb);

	if (iph->version == 4) {
		if (iph->protocol == IPPROTO_TCP)
			csum_type = TX_CSUM_TCPIP;
		else if (iph->protocol == IPPROTO_UDP)
			csum_type = TX_CSUM_UDPIP;
		else {
nocsum:			/*
			 * unknown protocol, disable HW csum
			 * and hope a bad packet is detected
			 */
			return F_TXPKT_L4CSUM_DIS;
		}
	} else {
		/*
		 * this doesn't work with extension headers
		 */
		const struct ipv6hdr *ip6h = (const struct ipv6hdr *)iph;

		if (ip6h->nexthdr == IPPROTO_TCP)
			csum_type = TX_CSUM_TCPIP6;
		else if (ip6h->nexthdr == IPPROTO_UDP)
			csum_type = TX_CSUM_UDPIP6;
		else
			goto nocsum;
	}

	if (likely(csum_type >= TX_CSUM_TCPIP))
		return V_TXPKT_CSUM_TYPE(csum_type) |
			V_TXPKT_IPHDR_LEN(skb_network_header_len(skb)) |
			V_TXPKT_ETHHDR_LEN(skb_network_offset(skb) - ETH_HLEN);
	else {
		int start = skb_transport_offset(skb);

		return V_TXPKT_CSUM_TYPE(csum_type) |
			V_TXPKT_CSUM_START(start) |
			V_TXPKT_CSUM_LOC(start + skb->csum_offset);
	}
}

#if 0
/*
 * Returns a pointer to the Tx descriptor at the start of the 8th most recent
 * packet.
 */
static struct tx_desc *wakeup_desc(const struct sge_txq *q)
{
	unsigned int n;

	n = (q->recent & 0x0f0f0f0f) + ((q->recent >> 4) & 0x0f0f0f0f);
	n = (n & 0x00ff00ff) + ((n >> 8) & 0x00ff00ff);
	n = (n & 0xffff) + (n >> 16);
	n = q->pidx >= n ? q->pidx - n : q->pidx - n + q->size;
	return &q->desc[n];
}
#endif

static void eth_txq_stop(struct sge_eth_txq *q)
{
	netif_tx_stop_queue(q->txq);
	q->q.stops++;
}

static inline void txq_advance(struct sge_txq *q, unsigned int n)
{
	q->in_use += n;
	q->pidx += n;
	if (q->pidx >= q->size)
		q->pidx -= q->size;
}

#define MAX_COALESCE_LEN 64000

static inline int wraps_around(struct sge_txq *q, int ndesc)
{
	return (q->pidx + ndesc) > q->size ? 1 : 0;
}

/**
 * 	ship_tx_pkt_coalesce_wr - finalizes and ships a coalesce WR
 * 	@ adap: adapter structure
 * 	@txq: tx queue
 *
 * 	writes the different fields of the pkts WR and sends it.
 */
static inline int ship_tx_pkt_coalesce_wr(struct adapter *adap, struct sge_eth_txq *txq)
{
	u32 wr_mid;
	struct sge_txq *q = &txq->q;
	struct fw_eth_tx_pkts_wr *wr;
	unsigned int ndesc;

	/* fill the pkts WR header */
	wr = (void *)&q->desc[q->pidx];
	wr->op_immdlen = htonl(V_FW_WR_OP(FW_ETH_TX_PKTS_WR) |
                               V_FW_WR_IMMDLEN(sizeof(u64) * (q->coalesce.flits - 2)));

	wr_mid = V_FW_WR_LEN16(DIV_ROUND_UP(q->coalesce.flits, 2));
	ndesc = flits_to_desc(q->coalesce.flits);
	
	if (q->coalesce.intr) {
		wr_mid |= F_FW_WR_EQUEQ | F_FW_WR_EQUIQ;
		q->coalesce.intr = false;
	}

	wr->equiq_to_len16 = htonl(wr_mid);
	wr->plen = cpu_to_be16(q->coalesce.len);
	wr->npkt = q->coalesce.idx;
	wr->r3 = 0;
	wr->type = q->coalesce.type;

	/* zero out coalesce structure members */
	q->coalesce.idx = 0;
	q->coalesce.flits = 0;
	q->coalesce.len = 0;

	txq_advance(q, ndesc);
	txq->coal_wr++;
	txq->coal_pkts += wr->npkt;
	ring_tx_db(adap, q, ndesc);

	return 1;
}

int t4_sge_coalesce_handler(struct adapter *adap, struct sge_eth_txq *eq)
{
	struct sge_txq *q = &eq->q;
	int hw_cidx = ntohs(q->stat->cidx);
	int in_use = q->pidx - hw_cidx + flits_to_desc(q->coalesce.flits);

	/* in_use is what the hardware hasn't processed yet and not
	 * the tx descriptors not yet freed */
	if (in_use < 0)
		in_use += q->size;

	/* if the queue is stopped and half the descritors were consumed
	 * by the hw, restart the queue */
	if (netif_tx_queue_stopped(eq->txq) && in_use < (eq->q.size >> 1)) {
		netif_tx_wake_queue(eq->txq);
		eq->q.restarts++;
	} else if (!netif_tx_queue_stopped(eq->txq) && in_use >= (eq->q.size >> 1))
		eq->q.coalesce.intr = true;

	if (eq->q.coalesce.idx && __netif_tx_trylock(eq->txq)){
		ship_tx_pkt_coalesce_wr(adap, eq);
		__netif_tx_unlock(eq->txq);
	}
	return 1; 
}

/**
 * 	should_tx_packet_coalesce - decides wether to coalesce an skb or not
 * 	@txq: tx queue where the skb is sent
 * 	@skb: skb to be sent
 * 	@nflits: return value for number of flits needed
 * 	@adap: adapter structure
 *
 *	This function decides if a packet should be coalesced or not. We start
 *	coalescing if half of the descriptors in a tx queue are used and stop
 *	when the number of used descriptors falls down to one fourth of the txq.
 */

static inline int should_tx_packet_coalesce(struct sge_eth_txq *txq, struct sk_buff *skb,
					    int *nflits, struct adapter *adap)
{
	struct skb_shared_info *si = skb_shinfo(skb);
	struct sge_txq *q = &txq->q;
	unsigned int flits, ndesc;
	unsigned char type = 0;
	int credits, hw_cidx = ntohs(q->stat->cidx);
	int in_use = q->pidx - hw_cidx + flits_to_desc(q->coalesce.flits);

	if (in_use < 0)
		in_use += q->size;

	if (unlikely(type != q->coalesce.type && q->coalesce.idx))
		ship_tx_pkt_coalesce_wr(adap, txq);

	/* calculate the number of flits required for coalescing this packet
	 * without the 2 flits of the WR header. These are added further down
	 * if we are just starting in new PKTS WR. sgl_len doesn't account for
	 * the possible 16 bytes alignment ULP TX commands so we do it here.
	 */
	flits = (sgl_len(si->nr_frags + 1) + 1) & ~1U;
	if (type == 0)
		flits += (sizeof(struct ulp_txpkt) +
			  sizeof(struct ulptx_idata)) / sizeof(__be64);
	flits += sizeof(struct cpl_tx_pkt_core) / sizeof(__be64);
	*nflits = flits;

	/* if we're using less than 64 descriptors and the tx_coal module parameter
	 * is not equal to 2 stop coalescing and ship any pending WR */
	if ((adap->tx_coal != 2) && in_use < 64) {
		if (q->coalesce.idx)
			ship_tx_pkt_coalesce_wr(adap, txq);
		q->coalesce.ison = false;

		return 0;
	}

	/* we don't bother coalescing gso packets */
	if (si->gso_size) {
		if (q->coalesce.idx)
			ship_tx_pkt_coalesce_wr(adap, txq);
		return 0;
	}

	/* if coalescing is on, the skb is added to a pkts WR. Otherwise,
	 * if the queue is half full we turn coalescing on but send this
	 * skb through the normal path to request a completion interrupt.
	 * if the queue is not half full we just send the skb through the
	 * normal path. */
	if (q->coalesce.ison) {
		if (q->coalesce.idx) {
			ndesc = DIV_ROUND_UP(q->coalesce.flits + flits, 8);
			credits = txq_avail(q) - ndesc;
			/* If credits are not available for this skb, send the
			 * already coalesced skbs and let the non-coalesce pass
			 * handle stopping the queue.
			 */
			if (unlikely(credits < ETHTXQ_STOP_THRES ||
				     wraps_around(q, ndesc))) {
				ship_tx_pkt_coalesce_wr(adap, txq);
				return 0;
			}
			/* If the max coalesce len or the max WR len is reached
			 * ship the WR and keep coalescing on.
			 */
			if (unlikely((q->coalesce.len + skb->len > 
				      MAX_COALESCE_LEN) ||
				     (q->coalesce.flits + flits >
				      q->coalesce.max))) {
				ship_tx_pkt_coalesce_wr(adap, txq);
				goto new;
			}
			return 1;
		} else
			goto new;
			
	} else if (in_use > (q->size >> 1)) {
		/* start coalescing and arm completion interrupt */
		q->coalesce.ison = true;
		q->coalesce.intr = true;
		return 0;
	} else
		return 0;

new:
	/* start a new pkts WR, the WR header is not filled below */
	flits += sizeof(struct fw_eth_tx_pkts_wr) /
			sizeof(__be64);
	ndesc = flits_to_desc(q->coalesce.flits + flits);
	credits = txq_avail(q) - ndesc;
	if (unlikely((credits < ETHTXQ_STOP_THRES) || wraps_around(q, ndesc)))
		return 0;
	q->coalesce.flits += 2;
	q->coalesce.type = type;
	q->coalesce.ptr = (unsigned char *) &q->desc[q->pidx] +
			  2 * sizeof(__be64);
	return 1;
}

/**
 * 	tx_do_packet_coalesce - add an skb to a coalesce WR
 *	@txq: sge_eth_txq used send the skb
 *	@skb: skb to be sent
 *	@flits: flits needed for this skb
 *	@adap: adapter structure
 *	@pi: port_info structure
 *	@addr: mapped address of the skb
 *
 *	Adds an skb to be sent as part of a coalesce WR by filling a
 *	ulp_tx_pkt command, ulp_tx_sc_imm command, cpl message and
 *	ulp_tx_sc_dsgl command.
 */
static inline int tx_do_packet_coalesce(struct sge_eth_txq *txq,
					struct sk_buff *skb,
					int flits, struct adapter *adap,
					const struct port_info *pi,
					dma_addr_t *addr)
{
	u64 cntrl, *end;
	struct sge_txq *q = &txq->q;
	struct ulp_txpkt *mc;
	struct ulptx_idata *sc_imm;
	struct cpl_tx_pkt_core *cpl;
	struct tx_sw_desc *sd;
	unsigned int idx = q->coalesce.idx, len = skb->len;

	if (q->coalesce.type == 0) {
		mc = (struct ulp_txpkt *) q->coalesce.ptr;
		mc->cmd_dest = htonl(V_ULPTX_CMD(4) | V_ULP_TXPKT_DEST(0) |
				V_ULP_TXPKT_FID(adap->sge.fw_evtq.cntxt_id) |
				F_ULP_TXPKT_RO);
		mc->len = htonl(DIV_ROUND_UP(flits, 2));

		sc_imm = (struct ulptx_idata *) (mc + 1);
		sc_imm->cmd_more = htonl(V_ULPTX_CMD(ULP_TX_SC_IMM) | F_ULP_TX_SC_MORE);
		sc_imm->len = htonl(sizeof(*cpl));
		end = (u64 *) mc + flits;
		cpl = (struct cpl_tx_pkt_core *) (sc_imm + 1);
	} else {
		end = (u64 *) q->coalesce.ptr + flits;
		cpl = (struct cpl_tx_pkt_core *) q->coalesce.ptr;
	}

	/* update coalesce structure for this txq */
	q->coalesce.flits += flits;
	q->coalesce.ptr += flits * sizeof(__be64);
	q->coalesce.len += skb->len;

	/* fill the cpl message, same as in t4_eth_xmit, this should be kept
	 * similar to t4_eth_xmit
	 */
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		cntrl = hwcsum(skb) | F_TXPKT_IPCSUM_DIS;
		txq->tx_cso++;
	} else
		cntrl = F_TXPKT_L4CSUM_DIS | F_TXPKT_IPCSUM_DIS;

	if (vlan_tx_tag_present(skb)) {
		txq->vlan_ins++;
		cntrl |= F_TXPKT_VLAN_VLD | V_TXPKT_VLAN(vlan_tx_tag_get(skb));
	}

	cpl->ctrl0 = htonl(V_TXPKT_OPCODE(CPL_TX_PKT_XT) |
			   V_TXPKT_INTF(pi->tx_chan) |
			   V_TXPKT_PF(adap->pf));
	cpl->pack = htons(0);
	cpl->len = htons(len);
	cpl->ctrl1 = cpu_to_be64(cntrl);

	write_sgl(skb, q, (struct ulptx_sgl *)(cpl + 1), end, 0,
		  addr);
	skb_orphan(skb);

	/* store pointers to the skb and the sgl used in free_tx_desc.
	 * each tx desc can hold two pointers corresponding to the value
	 * of ETH_COALESCE_PKT_PER_DESC */
	sd = &q->sdesc[q->pidx + (idx >> 1)];
	sd->coalesce.skb[idx & 1] = skb;
	sd->coalesce.sgl[idx & 1] = (struct ulptx_sgl *)(cpl + 1);
	sd->coalesce.idx = (idx & 1) + 1;

	/* send the coaelsced work request if max reached */
	if (++q->coalesce.idx == ETH_COALESCE_PKT_NUM)
		ship_tx_pkt_coalesce_wr(adap, txq);

	return NETDEV_TX_OK;
}

/**
 *	t4_eth_xmit - add a packet to an Ethernet Tx queue
 *	@skb: the packet
 *	@dev: the egress net device
 *
 *	Add a packet to an SGE Ethernet Tx queue.  Runs with softirqs disabled.
 */
int t4_eth_xmit(struct sk_buff *skb, struct net_device *dev)
{
	u32 wr_mid;
	u64 cntrl, *end;
	int qidx, credits;
	unsigned int flits, ndesc, cflits;
	struct adapter *adap;
	struct sge_eth_txq *q;
	const struct port_info *pi;
	struct fw_eth_tx_pkt_wr *wr;
	struct cpl_tx_pkt_core *cpl;
	const struct skb_shared_info *ssi;
	dma_addr_t addr[MAX_SKB_FRAGS + 1];

	/*
	 * The chip min packet length is 10 octets but play safe and reject
	 * anything shorter than an Ethernet header.
	 */
	if (unlikely(skb->len < ETH_HLEN)) {
out_free:	dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	pi = netdev_priv(dev);
	adap = pi->adapter;
	qidx = skb_get_queue_mapping(skb);
	q = &adap->sge.ethtxq[qidx + pi->first_qset];

	reclaim_completed_tx(adap, &q->q, true);

	/* align the end fo coalesce WR to a 512 byte boundary */
	q->q.coalesce.max = (8 - (q->q.pidx & 7)) * 8;

	/* check if we can do packet coalescing */
	if (adap->tx_coal && should_tx_packet_coalesce(q, skb, &cflits, adap)) {
		if (unlikely(map_skb(adap->pdev_dev, skb, addr) < 0)) {
			q->mapping_err++;
			goto out_free;
		}
		return tx_do_packet_coalesce(q, skb, cflits, adap, pi, addr);
	}

	flits = calc_tx_flits(skb);
	ndesc = flits_to_desc(flits);
	credits = txq_avail(&q->q) - ndesc;

	if (unlikely(credits < 0)) {
		eth_txq_stop(q);
		dev_err(adap->pdev_dev,
			"%s: Tx ring %u full while queue awake!\n",
			dev->name, qidx);
		return NETDEV_TX_BUSY;
	}

	if (!is_eth_imm(skb) &&
	    unlikely(map_skb(adap->pdev_dev, skb, addr) < 0)) {
		q->mapping_err++;
		goto out_free;
	}

	wr_mid = V_FW_WR_LEN16(DIV_ROUND_UP(flits, 2));
	if (unlikely(credits < ETHTXQ_STOP_THRES)) {
		eth_txq_stop(q);
		wr_mid |= F_FW_WR_EQUEQ | F_FW_WR_EQUIQ;
	}

	/* request tx completion if needed for tx coalescing */
	if (adap->tx_coal && q->q.coalesce.intr) {
		wr_mid |= F_FW_WR_EQUEQ | F_FW_WR_EQUIQ;
		q->q.coalesce.intr = false;
	}

	wr = (void *)&q->q.desc[q->q.pidx];
	wr->equiq_to_len16 = htonl(wr_mid);
	wr->r3 = cpu_to_be64(0);
	end = (u64 *)wr + flits;

	ssi = skb_shinfo(skb);
	if (ssi->gso_size) {
		struct cpl_tx_pkt_lso_core *lso = (void *)(wr + 1);
		bool v6 = (ssi->gso_type & SKB_GSO_TCPV6) != 0;
		int l3hdr_len = skb_network_header_len(skb);
		int eth_xtra_len = skb_network_offset(skb) - ETH_HLEN;

		wr->op_immdlen = htonl(V_FW_WR_OP(FW_ETH_TX_PKT_WR) |
				     V_FW_WR_IMMDLEN(sizeof(*lso) +
								sizeof(*cpl)));
		lso->lso_ctrl = htonl(V_LSO_OPCODE(CPL_TX_PKT_LSO) |
					F_LSO_FIRST_SLICE | F_LSO_LAST_SLICE |
					V_LSO_IPV6(v6) |
					V_LSO_ETHHDR_LEN(eth_xtra_len / 4) |
					V_LSO_IPHDR_LEN(l3hdr_len / 4) |
					V_LSO_TCPHDR_LEN(tcp_hdr(skb)->doff));
		lso->ipid_ofst = htons(0);
		lso->mss = htons(ssi->gso_size);
		lso->seqno_offset = htonl(0);
		lso->len = htonl(skb->len);
		cpl = (void *)(lso + 1);
		cntrl = V_TXPKT_CSUM_TYPE(v6 ? TX_CSUM_TCPIP6 : TX_CSUM_TCPIP) |
			V_TXPKT_IPHDR_LEN(l3hdr_len) |
			V_TXPKT_ETHHDR_LEN(eth_xtra_len);
		q->tso++;
		q->tx_cso += ssi->gso_segs;
	} else {
		int len;

		len = is_eth_imm(skb) ? skb->len + sizeof(*cpl) : sizeof(*cpl);
		wr->op_immdlen = htonl(V_FW_WR_OP(FW_ETH_TX_PKT_WR) |
				       V_FW_WR_IMMDLEN(len));
		cpl = (void *)(wr + 1);
		if (skb->ip_summed == CHECKSUM_PARTIAL) {
			cntrl = hwcsum(skb) | F_TXPKT_IPCSUM_DIS;
			q->tx_cso++;
		} else
			cntrl = F_TXPKT_L4CSUM_DIS | F_TXPKT_IPCSUM_DIS;
	}

	if (vlan_tx_tag_present(skb)) {
		q->vlan_ins++;
		cntrl |= F_TXPKT_VLAN_VLD | V_TXPKT_VLAN(vlan_tx_tag_get(skb));
	}

	cpl->ctrl0 = htonl(V_TXPKT_OPCODE(CPL_TX_PKT_XT) |
			   V_TXPKT_INTF(pi->tx_chan) |
			   V_TXPKT_PF(adap->pf));
	cpl->pack = htons(0);
	cpl->len = htons(skb->len);
	cpl->ctrl1 = cpu_to_be64(cntrl);

#ifdef T4_TRACE
	T4_TRACE5(adap->tb[q->q.cntxt_id & 7],
		  "eth_xmit: ndesc %u, credits %u, pidx %u, len %u, frags %u",
		  ndesc, credits, q->q.pidx, skb->len, ssi->nr_frags);
#endif

	if (is_eth_imm(skb)) {
		inline_tx_skb(skb, &q->q, cpl + 1);
		dev_kfree_skb(skb);
	} else {
		int last_desc;

		write_sgl(skb, &q->q, (struct ulptx_sgl *)(cpl + 1), end, 0,
			  addr);
		skb_orphan(skb);

		last_desc = q->q.pidx + ndesc - 1;
		if (last_desc >= q->q.size)
			last_desc -= q->q.size;
		q->q.sdesc[last_desc].skb = skb;
		q->q.sdesc[last_desc].sgl = (struct ulptx_sgl *)(cpl + 1);
	}

	txq_advance(&q->q, ndesc);

	dev->trans_start = jiffies;    // XXX removed in newer kernels
	ring_tx_db(adap, &q->q, ndesc);
	return NETDEV_TX_OK;
}

/**
 *	reclaim_completed_tx_imm - reclaim completed control-queue Tx descs
 *	@q: the SGE control Tx queue
 *
 *	This is a variant of reclaim_completed_tx() that is used for Tx queues
 *	that send only immediate data (presently just the control queues) and
 *	thus do not have any sk_buffs to release.
 */
static inline void reclaim_completed_tx_imm(struct sge_txq *q)
{
	int hw_cidx = ntohs(q->stat->cidx);
	int reclaim = hw_cidx - q->cidx;

	if (reclaim < 0)
		reclaim += q->size;

	q->in_use -= reclaim;
	q->cidx = hw_cidx;
}

/**
 *	is_imm - check whether a packet can be sent as immediate data
 *	@skb: the packet
 *
 *	Returns true if a packet can be sent as a WR with immediate data.
 */
static inline int is_imm(const struct sk_buff *skb)
{
	return skb->len <= MAX_CTRL_WR_LEN;
}

/**
 *	ctrlq_check_stop - check if a control queue is full and should stop
 *	@q: the queue
 *	@wr: most recent WR written to the queue
 *
 *	Check if a control queue has become full and should be stopped.
 *	We clean up control queue descriptors very lazily, only when we are out.
 *	If the queue is still full after reclaiming any completed descriptors
 *	we suspend it and have the last WR wake it up.
 */
static void ctrlq_check_stop(struct sge_ctrl_txq *q, struct fw_wr_hdr *wr)
{
	reclaim_completed_tx_imm(&q->q);
	if (unlikely(txq_avail(&q->q) < TXQ_STOP_THRES)) {
		wr->lo |= htonl(F_FW_WR_EQUEQ | F_FW_WR_EQUIQ);
		q->q.stops++;
		q->full = 1;
	}
}

/**
 *	ctrl_xmit - send a packet through an SGE control Tx queue
 *	@q: the control queue
 *	@skb: the packet
 *
 *	Send a packet through an SGE control Tx queue.  Packets sent through
 *	a control queue must fit entirely as immediate data.
 */
static int ctrl_xmit(struct sge_ctrl_txq *q, struct sk_buff *skb)
{
	unsigned int ndesc;
	struct fw_wr_hdr *wr;

	if (unlikely(!is_imm(skb))) {
		WARN_ON(1);
		dev_kfree_skb(skb);
		return NET_XMIT_DROP;
	}

	ndesc = DIV_ROUND_UP(skb->len, sizeof(struct tx_desc));
	spin_lock(&q->sendq.lock);

	if (unlikely(q->full)) {
		skb->priority = ndesc;                  /* save for restart */
		__skb_queue_tail(&q->sendq, skb);
		spin_unlock(&q->sendq.lock);
		return NET_XMIT_CN;
	}

	wr = (struct fw_wr_hdr *)&q->q.desc[q->q.pidx];
	inline_tx_skb(skb, &q->q, wr);

	txq_advance(&q->q, ndesc);
	if (unlikely(txq_avail(&q->q) < TXQ_STOP_THRES))
		ctrlq_check_stop(q, wr);

	ring_tx_db(q->adap, &q->q, ndesc);
	spin_unlock(&q->sendq.lock);

	kfree_skb(skb);
	return NET_XMIT_SUCCESS;
}


/**
 *	restart_ctrlq - restart a suspended control queue
 *	@data: the control queue to restart
 *
 *	Resumes transmission on a suspended Tx control queue.
 */
static void restart_ctrlq(unsigned long data)
{
	struct sk_buff *skb;
	unsigned int written = 0;
	struct sge_ctrl_txq *q = (struct sge_ctrl_txq *)data;

	spin_lock(&q->sendq.lock);
	reclaim_completed_tx_imm(&q->q);
	BUG_ON(txq_avail(&q->q) < TXQ_STOP_THRES);  /* q should be empty */

	while ((skb = __skb_dequeue(&q->sendq)) != NULL) {
		struct fw_wr_hdr *wr;
		unsigned int ndesc = skb->priority;     /* previously saved */

		/*
		 * Write descriptors and free skbs outside the lock to limit
		 * wait times.  q->full is still set so new skbs will be queued.
		 */
		spin_unlock(&q->sendq.lock);

		wr = (struct fw_wr_hdr *)&q->q.desc[q->q.pidx];
		inline_tx_skb(skb, &q->q, wr);
		kfree_skb(skb);

		written += ndesc;
		txq_advance(&q->q, ndesc);
		if (unlikely(txq_avail(&q->q) < TXQ_STOP_THRES)) {
			unsigned long old = q->q.stops;

			ctrlq_check_stop(q, wr);
			if (q->q.stops != old) {          /* suspended anew */
				spin_lock(&q->sendq.lock);
				goto ringdb;
			}
		}
		if (written > 16) {
			ring_tx_db(q->adap, &q->q, written);
			written = 0;
		}
		spin_lock(&q->sendq.lock);
	}
	q->full = 0;
ringdb: if (written)
		ring_tx_db(q->adap, &q->q, written);
	spin_unlock(&q->sendq.lock);
}

/**
 *	t4_mgmt_tx - send a management message
 *	@adap: the adapter
 *	@skb: the packet containing the management message
 *
 *	Send a management message through control queue 0.
 */
int t4_mgmt_tx(struct adapter *adap, struct sk_buff *skb)
{
	int ret;

	local_bh_disable();
	ret = ctrl_xmit(&adap->sge.ctrlq[0], skb);
	local_bh_enable();
	return ret;
}

/**
 *	is_ofld_imm - check whether a packet can be sent as immediate data
 *	@skb: the packet
 *
 *	Returns true if a packet can be sent as an offload WR with immediate
 *	data.  We currently use the same limit as for Ethernet packets.
 */
static inline int is_ofld_imm(const struct sk_buff *skb)
{
	return skb->len <= MAX_IMM_TX_PKT_LEN;
}

/**
 *	calc_tx_flits_ofld - calculate # of flits for an offload packet
 *	@skb: the packet
 *
 * 	Returns the number of flits needed for the given offload packet.
 * 	These packets are already fully constructed and no additional headers
 * 	will be added.
 */
static inline unsigned int calc_tx_flits_ofld(const struct sk_buff *skb)
{
	unsigned int flits, cnt;

	if (is_ofld_imm(skb))
		return DIV_ROUND_UP(skb->len, 8);

	flits = skb_transport_offset(skb) / 8U;   /* headers */
	cnt = skb_shinfo(skb)->nr_frags;
	if (skb->tail != skb->transport_header)
		cnt++;
	return flits + sgl_len(cnt);
}

/**
 *	txq_stop_maperr - stop a Tx queue due to I/O MMU exhaustion
 *	@adap: the adapter
 *	@q: the queue to stop
 *
 *	Mark a Tx queue stopped due to I/O MMU exhaustion and resulting
 *	inability to map packets.  A periodic timer attempts to restart
 *	queues so marked.
 */
static void txq_stop_maperr(struct sge_ofld_txq *q)
{
	q->mapping_err++;
	q->q.stops++;
	set_bit(q->q.cntxt_id - q->adap->sge.egr_start,
		q->adap->sge.txq_maperr);
}

/**
 *	ofldtxq_stop - stop an offload Tx queue that has become full
 *	@q: the queue to stop
 *	@skb: the packet causing the queue to become full
 *
 *	Stops an offload Tx queue that has become full and modifies the packet
 *	being written to request a wakeup.
 */
static void ofldtxq_stop(struct sge_ofld_txq *q, struct sk_buff *skb)
{
	struct fw_wr_hdr *wr = (struct fw_wr_hdr *)skb->data;

	wr->lo |= htonl(F_FW_WR_EQUEQ | F_FW_WR_EQUIQ);
	q->q.stops++;
	q->full = 1;
}

/**
 *	service_ofldq - restart a suspended offload queue
 *	@q: the offload queue
 *
 *	Services an offload Tx queue by moving packets from its packet queue
 *	to the HW Tx ring.  The function starts and ends with the queue locked.
 */
static void service_ofldq(struct sge_ofld_txq *q)
{
	u64 *pos;
	int credits;
	struct sk_buff *skb;
	unsigned int written = 0;
	unsigned int flits, ndesc;

	while ((skb = skb_peek(&q->sendq)) != NULL && !q->full) {
		/*
		 * We drop the lock but leave skb on sendq, thus retaining
		 * exclusive access to the state of the queue.
		 */
		spin_unlock(&q->sendq.lock);

		reclaim_completed_tx(q->adap, &q->q, false);

		flits = skb->priority;                /* previously saved */
		ndesc = flits_to_desc(flits);
		credits = txq_avail(&q->q) - ndesc;
		BUG_ON(credits < 0);
		if (unlikely(credits < TXQ_STOP_THRES))
			ofldtxq_stop(q, skb);
#ifdef T4_TRACE
		T4_TRACE5(q->adap->tb[q->q.cntxt_id & 7],
			  "ofld_xmit: ndesc %u, pidx %u, len %u, main %u, "
			  "frags %u", ndesc, q->q.pidx, skb->len,
			  skb->len - skb->data_len, skb_shinfo(skb)->nr_frags);
#endif
		pos = (u64 *)&q->q.desc[q->q.pidx];
		if (is_ofld_imm(skb))
			inline_tx_skb(skb, &q->q, pos);
		else if (map_skb(q->adap->pdev_dev, skb,
				 (dma_addr_t *)skb->head)) {
			txq_stop_maperr(q);
			spin_lock(&q->sendq.lock);
			break;
		} else {
			int last_desc, hdr_len = skb_transport_offset(skb);

			/*
			 * This assumes the WR headers fit within one descriptor
			 * with room to spare.  Otherwise we need to deal with
			 * wrap-around here.
			 */
			memcpy(pos, skb->data, hdr_len);
			write_sgl(skb, &q->q, (void *)pos + hdr_len,
				  pos + flits, hdr_len,
				  (dma_addr_t *)skb->head);

			if (need_skb_unmap()) {
				skb->dev = q->adap->port[0];
				skb->destructor = deferred_unmap_destructor;
			}

			last_desc = q->q.pidx + ndesc - 1;
			if (last_desc >= q->q.size)
				last_desc -= q->q.size;
			q->q.sdesc[last_desc].skb = skb;
		}

		txq_advance(&q->q, ndesc);
		written += ndesc;
		if (unlikely(written > 32)) {
			ring_tx_db(q->adap, &q->q, written);
			written = 0;
		}

		spin_lock(&q->sendq.lock);
		__skb_unlink(skb, &q->sendq);
		if (is_ofld_imm(skb))
			kfree_skb(skb);
	}
	if (likely(written))
		ring_tx_db(q->adap, &q->q, written);
}

/**
 *	ofld_xmit - send a packet through an offload queue
 *	@q: the Tx offload queue
 *	@skb: the packet
 *
 *	Send an offload packet through an SGE offload queue.
 */
static int ofld_xmit(struct sge_ofld_txq *q, struct sk_buff *skb)
{
	skb->priority = calc_tx_flits_ofld(skb);       /* save for restart */
	spin_lock(&q->sendq.lock);
	__skb_queue_tail(&q->sendq, skb);
	if (q->sendq.qlen == 1)
		service_ofldq(q);
	spin_unlock(&q->sendq.lock);
	return NET_XMIT_SUCCESS;
}

/**
 *	restart_ofldq - restart a suspended offload queue
 *	@data: the offload queue to restart
 *
 *	Resumes transmission on a suspended Tx offload queue.
 */
static void restart_ofldq(unsigned long data)
{
	struct sge_ofld_txq *q = (struct sge_ofld_txq *)data;

	spin_lock(&q->sendq.lock);
	q->full = 0;            /* the queue actually is completely empty now */
	service_ofldq(q);
	spin_unlock(&q->sendq.lock);
}

/**
 *	skb_txq - return the Tx queue an offload packet should use
 *	@skb: the packet
 *
 *	Returns the Tx queue an offload packet should use as indicated by bits
 *	1-15 in the packet's queue_mapping.
 */
static inline unsigned int skb_txq(const struct sk_buff *skb)
{
	return skb->queue_mapping >> 1;
}

/**
 *	is_ctrl_pkt - return whether an offload packet is a control packet
 *	@skb: the packet
 *
 *	Returns whether an offload packet should use an OFLD or a CTRL
 *	Tx queue as indicated by bit 0 in the packet's queue_mapping.
 */
static inline unsigned int is_ctrl_pkt(const struct sk_buff *skb)
{
	return skb->queue_mapping & 1;
}

static inline int ofld_send(struct adapter *adap, struct sk_buff *skb)
{
	unsigned int idx = skb_txq(skb);

	if (unlikely(is_ctrl_pkt(skb)))
		return ctrl_xmit(&adap->sge.ctrlq[idx], skb);
	return ofld_xmit(&adap->sge.ofldtxq[idx], skb);
}

/**
 *	t4_ofld_send - send an offload packet
 *	@adap: the adapter
 *	@skb: the packet
 *
 *	Sends an offload packet.  We use the packet queue_mapping to select the
 *	appropriate Tx queue as follows: bit 0 indicates whether the packet
 *	should be sent as regular or control, bits 1-15 select the queue.
 */
int t4_ofld_send(struct adapter *adap, struct sk_buff *skb)
{
	int ret;

	local_bh_disable();
	ret = ofld_send(adap, skb);
	local_bh_enable();
	return ret;
}

/**
 *	cxgb4_ofld_send - send an offload packet
 *	@dev: the net device
 *	@skb: the packet
 *
 *	Sends an offload packet.  This is an exported version of @t4_ofld_send,
 *	intended for ULDs.
 */
int cxgb4_ofld_send(struct net_device *dev, struct sk_buff *skb)
{
	return t4_ofld_send(netdev2adap(dev), skb);
}
EXPORT_SYMBOL(cxgb4_ofld_send);

/**
 *	pktgl_to_skb_usepages - build an sk_buff from a packet gather list
 *	@gl: the gather list
 *	@skb_len: size of sk_buff main body if it carries fragments
 *	@pull_len: amount of data to move to the sk_buff's main body
 *
 *	Builds an sk_buff from the given packet gather list.  Returns the
 *	sk_buff or %NULL if sk_buff allocation failed.
 */
struct sk_buff *t4_pktgl_to_skb_usepages(const struct pkt_gl *gl,
					 unsigned int skb_len,
					 unsigned int pull_len)
{
	struct sk_buff *skb;
	struct skb_shared_info *ssi;

	/*
	 * Below we rely on RX_COPY_THRES being less than the smallest Rx buffer
	 * size, which is expected since buffers are at least PAGE_SIZEd.
	 * In this case packets up to RX_COPY_THRES have only one fragment.
	 */
	if (gl->tot_len <= RX_COPY_THRES) {
		skb = alloc_skb(gl->tot_len, GFP_ATOMIC);
		if (unlikely(!skb))
			goto out;
		__skb_put(skb, gl->tot_len);
		skb_copy_to_linear_data(skb, gl->va, gl->tot_len);
	} else {
		skb = alloc_skb(skb_len, GFP_ATOMIC);
		if (unlikely(!skb))
			goto out;
		__skb_put(skb, pull_len);
		skb_copy_to_linear_data(skb, gl->va, pull_len);

		ssi = skb_shinfo(skb);
		ssi->frags[0].page = gl->frags[0].page;
		ssi->frags[0].page_offset = gl->frags[0].page_offset + pull_len;
		ssi->frags[0].size = gl->frags[0].size - pull_len;
		if (gl->nfrags > 1)
			memcpy(&ssi->frags[1], &gl->frags[1],
			       (gl->nfrags - 1) * sizeof(skb_frag_t));
		ssi->nr_frags = gl->nfrags;

		skb->len = gl->tot_len;
		skb->data_len = skb->len - pull_len;
		skb->truesize += skb->data_len;

		/* Get a reference for the last page, we don't own it */
		get_page(gl->frags[gl->nfrags - 1].page);
	}
out:	return skb;
}


/**
 *	pktgl_to_skb_useskbs - build an sk_buff from a packet gather list
 *	@gl: the gather list
 *	@skb_len: size of sk_buff main body if it carries fragments
 *	@pull_len: amount of data to move to the sk_buff's main body
 *
 *	Builds an sk_buff from the given packet gather list.  Returns the
 *	sk_buff or %NULL if sk_buff allocation failed.
 */
struct sk_buff *t4_pktgl_to_skb_useskbs(const struct pkt_gl *gl,
					unsigned int skb_len,
					unsigned int pull_len)
{
	struct sk_buff *skb;
	unsigned char *dp;
	int frag;


	/*
	 * If there's only one skb fragment, just return that.
	 */
	if (likely(gl->nfrags == 1))
		return gl->skbs[0];

	/*
	 * There are multiple skb fragments so we need to create a single new
	 * skb which contains all the data.  This can happen when the MTU on
	 * an interface is increased and the ingress packet is received into
	 * the old smaller MTU buffers which were on the receive ring at the
	 * time of the MTU change.
	 */
	skb = alloc_skb(gl->tot_len + FL_PKTSHIFT, GFP_ATOMIC);
	if (unlikely(!skb))
		goto out;

	skb_put(skb, gl->tot_len + FL_PKTSHIFT);
	dp = skb->data;
	for (frag = 0; frag < gl->nfrags; frag++) {
		struct sk_buff *sskb = gl->skbs[frag];
		memcpy(dp, sskb->data, sskb->len);
		dp += sskb->len;
		kfree_skb(sskb);
	}

out:	return skb;
}

/**
 *	pktgl_to_skb - build an sk_buff from a packet gather list
 *	@gl: the gather list
 *	@skb_len: size of sk_buff main body if it carries fragments
 *	@pull_len: amount of data to move to the sk_buff's main body
 *
 *	Builds an sk_buff from the given packet gather list.  Returns the
 *	sk_buff or %NULL if sk_buff allocation failed.
 */
struct sk_buff *t4_pktgl_to_skb(const struct pkt_gl *gl,
					unsigned int skb_len,
					unsigned int pull_len)
{
	return (unlikely(gl->useskbs)
		? t4_pktgl_to_skb_useskbs(gl, skb_len, pull_len)
		: t4_pktgl_to_skb_usepages(gl, skb_len, pull_len));
}
EXPORT_SYMBOL(t4_pktgl_to_skb);

/**
 *	t4_pktgl_free - free a packet gather list
 *	@gl: the gather list
 *
 *	Releases the buffers of a packet gather list.
 */
void t4_pktgl_free(const struct pkt_gl *gl)
{
	int n;

	if (unlikely(gl->useskbs)) {
		for (n = 0; n < gl->nfrags; n++)
			kfree_skb(gl->skbs[n]);
	} else {
		const skb_frag_t *p;

		/*
		 * We do not own the last page on the list and do not free
		 * it.
		 */
		for (p = gl->frags, n = gl->nfrags - 1; n--; p++)
			put_page(p->page);
	}
}

#ifdef CONFIG_CXGB4_GRO
/**
 *	copy_frags - copy fragments from gather list into skb_shared_info
 *	@si: destination skb shared info structure
 *	@gl: source internal packet gather list
 *	@offset: packet start offset in first page
 *
 *	Copy an internal packet gather list into a Linux skb_shared_info
 *	structure.
 */
static inline void copy_frags(struct skb_shared_info *si,
			      const struct pkt_gl *gl,
			      unsigned int offset)
{
	unsigned int n;

	/* usually there's just one frag */
	si->frags[0].page = gl->frags[0].page;
	si->frags[0].page_offset = gl->frags[0].page_offset + offset;
	si->frags[0].size = gl->frags[0].size - offset;
	si->nr_frags = gl->nfrags;

	n = gl->nfrags - 1;
	if (n)
		memcpy(&si->frags[1], &gl->frags[1], n * sizeof(skb_frag_t));

	/* get a reference to the last page, we don't own it */
	get_page(gl->frags[n].page);
}

/**
 *	do_gro - perform Generic Receive Offload ingress packet processing
 *	@rxq: ingress RX Ethernet Queue
 *	@gl: gather list for ingress packet
 *	@pkt: CPL header for last packet fragment
 *
 *	Perform Generic Receive Offload (GRO) ingress packet processing.
 *	We use the standard Linux GRO interfaces for this.
 */
static void do_gro(struct sge_eth_rxq *rxq, const struct pkt_gl *gl,
		   const struct cpl_rx_pkt *pkt)
{
	int ret;
	struct sk_buff *skb;

	skb = napi_get_frags(&rxq->rspq.napi);
	if (unlikely(!skb)) {
		t4_pktgl_free(gl);
		rxq->stats.rx_drops++;
		return;
	}

	copy_frags(skb_shinfo(skb), gl, FL_PKTSHIFT);
	skb->len = gl->tot_len - FL_PKTSHIFT;
	skb->data_len = skb->len;
	skb->truesize += skb->data_len;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb_record_rx_queue(skb, rxq->rspq.idx);

	if (unlikely(pkt->vlan_ex)) {
		struct port_info *pi = netdev_priv(rxq->rspq.netdev);
		struct vlan_group *grp = pi->vlan_grp;

		rxq->stats.vlan_ex++;
		if (likely(grp)) {
			ret = vlan_gro_frags(&rxq->rspq.napi, grp,
					     be16_to_cpu(pkt->vlan));
			goto stats;
		}
	}
	ret = napi_gro_frags(&rxq->rspq.napi);

stats:
	if (ret == GRO_HELD)
		rxq->stats.lro_pkts++;
	else if (ret == GRO_MERGED || ret == GRO_MERGED_FREE)
		rxq->stats.lro_merged++;
	rxq->stats.pkts++;
	rxq->stats.rx_cso++;
}
#endif

/*
 * Process an MPS trace packet.  Give it an unused protocol number so it won't
 * be delivered to anyone and send it to the stack for capture.
 */
static noinline int handle_trace_pkt(struct adapter *adap,
				     const struct pkt_gl *gl)
{
	struct sk_buff *skb;
	struct cpl_trace_pkt *p;

	skb = t4_pktgl_to_skb(gl, RX_PULL_LEN, RX_PULL_LEN);
	if (unlikely(!skb)) {
		t4_pktgl_free(gl);
		return 0;
	}

	p = (struct cpl_trace_pkt *)skb->data;
	__skb_pull(skb, sizeof(*p));
	skb_reset_mac_header(skb);
	skb->protocol = htons(0xffff);
	skb->dev = adap->port[0];
	netif_receive_skb(skb);
	return 0;
}

/**
 *	t4_ethrx_handler - process an ingress ethernet packet
 *	@q: the response queue that received the packet
 *	@rsp: the response queue descriptor holding the RX_PKT message
 *	@si: the gather list of packet fragments
 *
 *	Process an ingress ethernet packet and deliver it to the stack.
 */
int t4_ethrx_handler(struct sge_rspq *q, const __be64 *rsp,
		     const struct pkt_gl *si)
{
	struct sk_buff *skb;
	struct port_info *pi;
	const struct cpl_rx_pkt *pkt;
	bool csum_ok;
	struct sge_eth_rxq *rxq = container_of(q, struct sge_eth_rxq, rspq);

	if (unlikely(*(u8 *)rsp == CPL_TRACE_PKT))
		return handle_trace_pkt(q->adapter, si);

	pkt = (void *)&rsp[1];
	csum_ok = pkt->csum_calc && !pkt->err_vec;

#ifdef CONFIG_CXGB4_GRO
	/*
	 * If this is a good TCP packet and we have Generic Receive Offload
	 * enabled, handle the packet in the GRO path.
	 */
	if ((pkt->l2info & cpu_to_be32(F_RXF_TCP)) &&
	    (q->netdev->features & NETIF_F_GRO) &&
	    likely(si->useskbs == 0) &&
	    csum_ok && !pkt->ip_frag) {
		do_gro(rxq, si, pkt);
		return 0;
	}
#endif

	skb = t4_pktgl_to_skb(si, RX_PKT_SKB_LEN, RX_PULL_LEN);
	if (unlikely(!skb)) {
		t4_pktgl_free(si);
		rxq->stats.rx_drops++;
		return 0;
	}

	__skb_pull(skb, RX_PKT_PAD);      /* remove ethernet header padding */
	skb->protocol = eth_type_trans(skb, q->netdev);
	skb_record_rx_queue(skb, q->idx);
	pi = netdev_priv(skb->dev);
	rxq->stats.pkts++;

	if (csum_ok && (pi->rx_offload & RX_CSO) &&
	    (pkt->l2info & htonl(F_RXF_UDP | F_RXF_TCP))) {
		if (!pkt->ip_frag) {
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			rxq->stats.rx_cso++;
		} else if (pkt->l2info & htonl(F_RXF_IP)) {
			__sum16 c = (__force __sum16)pkt->csum;
			skb->csum = csum_unfold(c);
			skb->ip_summed = CHECKSUM_COMPLETE;
			rxq->stats.rx_cso++;
		}
	} else
		skb->ip_summed = CHECKSUM_NONE;

	if (unlikely(pkt->vlan_ex)) {
		struct vlan_group *grp = pi->vlan_grp;

		rxq->stats.vlan_ex++;
		if (likely(grp))
			vlan_hwaccel_receive_skb(skb, grp, ntohs(pkt->vlan));
		else
			dev_kfree_skb_any(skb);
	} else {
#ifdef HAVE_PF_RING
                {
                    int debug = 0;
                    struct net_device *dev = skb->dev;
                    struct pfring_hooks *hook = (struct pfring_hooks *)
                        dev->pfring_ptr;

                    if (hook && (hook->magic == PF_RING)) {
                        /* Wow: PF_RING is alive & kickin' ! */
                        int rc;

                    if (debug)
                        printk(KERN_INFO "[PF_RING] alive [%s][len=%d]\n",
                        dev->name, skb->len);

                    if (*hook->transparent_mode != standard_linux_path) {
                        rc = hook->ring_handler(skb, 1, 1, -1, 1);

                        if (rc == 1 /* Packet handled by PF_RING */) {
                            if (*hook->transparent_mode == driver2pf_ring_non_transparent) {
                                /* PF_RING has already freed the memory */
                                goto rcd_done;
                            }
                        }
                    }
                    else {
                        if (debug) printk(KERN_INFO "[PF_RING] not present on %s\n",
                               dev->name);
                    }
                    }
                }
#endif
		netif_receive_skb(skb);
	}
#ifdef HAVE_PF_RING
rcd_done:
#endif
	return 0;
}

/**
 *	restore_rx_bufs - put back a packet's Rx buffers
 *	@q: the SGE free list
 *	@frags: number of FL buffers to restore
 *
 *	Puts back on an FL the Rx buffers.  The buffers have already been
 *	unmapped and are left unmapped, we mark them so to prevent further
 *	unmapping attempts. 
 *
 *	This function undoes a series of @unmap_rx_buf calls when we find out
 *	that the current packet can't be processed right away afterall and we
 *	need to come back to it later.  This is a very rare event and there's
 *	no effort to make this particularly efficient.
 */
static void restore_rx_bufs(struct sge_fl *q, int frags)
{
	struct rx_sw_desc *d;

	while (frags--) {
		if (q->cidx == 0)
			q->cidx = q->size - 1;
		else
			q->cidx--;
		d = &q->sdesc[q->cidx];
		d->dma_addr |= RX_UNMAPPED_BUF;
		q->avail++;
	}
}

/**
 *	is_new_response - check if a response is newly written
 *	@r: the response descriptor
 *	@q: the response queue
 *
 *	Returns true if a response descriptor contains a yet unprocessed
 *	response.
 */
static inline bool is_new_response(const struct rsp_ctrl *r,
				   const struct sge_rspq *q)
{
	return (r->u.type_gen >> S_RSPD_GEN) == q->gen;
}

/**
 *	rspq_next - advance to the next entry in a response queue
 *	@q: the queue
 *
 *	Updates the state of a response queue to advance it to the next entry.
 */
static inline void rspq_next(struct sge_rspq *q)
{
	q->cur_desc = (void *)q->cur_desc + q->iqe_len;
	if (unlikely(++q->cidx == q->size)) {
		q->cidx = 0;
		q->gen ^= 1;
		q->cur_desc = q->desc;
	}
}

/**
 *	process_responses - process responses from an SGE response queue
 *	@q: the ingress queue to process
 *	@budget: how many responses can be processed in this round
 *
 *	Process responses from an SGE response queue up to the supplied budget.
 *	Responses include received packets as well as control messages from FW
 *	or HW.
 *
 *	Additionally choose the interrupt holdoff time for the next interrupt
 *	on this queue.  If the system is under memory shortage use a fairly
 *	long delay to help recovery.
 */
int process_responses(struct sge_rspq *q, int budget)
{
	int ret, rsp_type;
	int budget_left = budget;
	const struct rsp_ctrl *rc;
	struct sge_eth_rxq *rxq = container_of(q, struct sge_eth_rxq, rspq);

	while (likely(budget_left)) {
		rc = (void *)q->cur_desc + (q->iqe_len - sizeof(*rc));
		if (!is_new_response(rc, q))
			break;

		rmb();
		rsp_type = G_RSPD_TYPE(rc->u.type_gen);
		if (likely(rsp_type == X_RSPD_TYPE_FLBUF)) {
			const struct rx_sw_desc *rsd;
			u32 len = ntohl(rc->pldbuflen_qid), bufsz, frags;
			struct pkt_gl si;

			si.useskbs = rxq->useskbs;
			if (unlikely(si.useskbs)) {
				struct sk_buff *skb;

				/*
				 * In "use skbs" mode, we don't pack multiple
				 * ingress packets per buffer (skb) so we
				 * should _always_ get a "New Buffer" flags
				 * from the SGE.  Also, since we hand the
				 * skb's up to the host stack for it to
				 * eventually free, we don't release the skb's
				 * in the driver (in contrast to the "packed
				 * page" mode where the driver needs to
				 * release its reference on the page buffers).
				 */
				BUG_ON(!(len & F_RSPD_NEWBUF));
				len = G_RSPD_LEN(len);
				si.tot_len = len;

				/* gather packet fragments */
				for (frags = 0; len; frags++) {
					rsd = &rxq->fl.sdesc[rxq->fl.cidx];
					bufsz = min(get_buf_size(rsd),
						    (int)len);
					skb = rsd->buf;
					skb_put(skb, bufsz);
					si.skbs[frags] = skb;
					len -= bufsz;
					unmap_rx_buf(q->adapter, &rxq->fl);
				}

				si.va = si.skbs[0]->data;
				prefetch(si.va);
	
				si.nfrags = frags;
				ret = q->handler(q, q->cur_desc, &si);
				if (unlikely(ret != 0))
					restore_rx_bufs(&rxq->fl, frags);
			} else {
				skb_frag_t *fp;

				if (len & F_RSPD_NEWBUF) {
					if (likely(q->offset > 0)) {
						free_rx_bufs(q->adapter,
							     &rxq->fl, 1);
						q->offset = 0;
					}
					len = G_RSPD_LEN(len);
				}
				si.tot_len = len;
	
				/* gather packet fragments */
				for (frags = 0, fp = si.frags; ; frags++, fp++) {
					rsd = &rxq->fl.sdesc[rxq->fl.cidx];
					bufsz = min(get_buf_size(rsd), (int)len);
					fp->page = rsd->buf;
					fp->page_offset = q->offset;
					fp->size = bufsz;
					len -= bufsz;
					if (!len)
						break;
					unmap_rx_buf(q->adapter, &rxq->fl);
				}
	
				/*
				 * Last buffer remains mapped so explicitly
				 * make it coherent for CPU access.
				 */
				dma_sync_single_for_cpu(q->adapter->pdev_dev,
							get_buf_addr(rsd),
							fp->size,
							DMA_FROM_DEVICE);
	
					si.va = page_address(si.frags[0].page) +
						si.frags[0].page_offset;
	
				prefetch(si.va);
	
				si.nfrags = frags + 1;
				ret = q->handler(q, q->cur_desc, &si);
				if (likely(ret == 0))
					q->offset += ALIGN(fp->size, FL_ALIGN);
				else
					restore_rx_bufs(&rxq->fl, frags);
			}
		} else if (likely(rsp_type == X_RSPD_TYPE_CPL)) {
			ret = q->handler(q, q->cur_desc, NULL);
		} else {
			ret = q->handler(q, (const __be64 *)rc, CXGB4_MSG_AN);
		}

		if (unlikely(ret)) {
			/* couldn't process descriptor, back off for recovery */
			q->next_intr_params = V_QINTR_TIMER_IDX(NOMEM_TMR_IDX);
			break;
		}

		rspq_next(q);
		budget_left--;
	}

	if (q->offset >= 0 && rxq->fl.size - rxq->fl.avail >= 16)
		__refill_fl(q->adapter, &rxq->fl);
	return budget - budget_left;
}

/**
 *	napi_rx_handler - the NAPI handler for Rx processing
 *	@napi: the napi instance
 *	@budget: how many packets we can process in this round
 *
 *	Handler for new data events when using NAPI.  This does not need any
 *	locking or protection from interrupts as data interrupts are off at
 *	this point and other adapter interrupts do not interfere (the latter
 *	in not a concern at all with MSI-X as non-data interrupts then have
 *	a separate handler).
 */
static int napi_rx_handler(struct napi_struct *napi, int budget)
{
	unsigned int params;
	struct sge_rspq *q = container_of(napi, struct sge_rspq, napi);
	int work_done = process_responses(q, budget);

	if (likely(work_done < budget)) {
		napi_complete(napi);
		params = q->next_intr_params;
		q->next_intr_params = q->intr_params;
	} else
		params = V_QINTR_TIMER_IDX(X_TIMERREG_UPDATE_CIDX);

	t4_write_reg(q->adapter, MYPF_REG(A_SGE_PF_GTS), V_CIDXINC(work_done) |
		     V_INGRESSQID((u32)q->cntxt_id) | V_SEINTARM(params));
	return work_done;
}

/*
 * The MSI-X interrupt handler for an SGE response queue.
 */
irqreturn_t t4_sge_intr_msix(int irq, void *cookie)
{
	struct sge_rspq *q = cookie;

	napi_schedule(&q->napi);
	return IRQ_HANDLED;
}

/*
 * Process the indirect interrupt entries in the interrupt queue and kick off
 * NAPI for each queue that has generated an entry.
 */
static unsigned int process_intrq(struct adapter *adap)
{
	unsigned int credits;
	const struct rsp_ctrl *rc;
	struct sge_rspq *q = &adap->sge.intrq;

	spin_lock(&adap->sge.intrq_lock);
	for (credits = 0; ; credits++) {
		rc = (void *)q->cur_desc + (q->iqe_len - sizeof(*rc));
		if (!is_new_response(rc, q))
			break;

		rmb();
		if (G_RSPD_TYPE(rc->u.type_gen) == X_RSPD_TYPE_INTR) {
			unsigned int qid = ntohl(rc->pldbuflen_qid);

			qid -= adap->sge.ingr_start;
			napi_schedule(&adap->sge.ingr_map[qid]->napi);
		}

		rspq_next(q);
	}

	t4_write_reg(adap, MYPF_REG(A_SGE_PF_GTS), V_CIDXINC(credits) |
		     V_INGRESSQID(q->cntxt_id) | V_SEINTARM(q->intr_params));
	spin_unlock(&adap->sge.intrq_lock);
	return credits;
}

/*
 * The MSI interrupt handler, which handles data events from SGE response queues
 * as well as error and other async events as they all use the same MSI vector.
 */
static irqreturn_t t4_intr_msi(int irq, void *cookie)
{
	struct adapter *adap = cookie;

	t4_slow_intr_handler(adap);
	process_intrq(adap);
	return IRQ_HANDLED;
}

/*
 * Interrupt handler for legacy INTx interrupts for T4-based cards.
 * Handles data events from SGE response queues as well as error and other
 * async events as they all use the same interrupt line.
 */
static irqreturn_t t4_intr_intx(int irq, void *cookie)
{
	struct adapter *adap = cookie;

	t4_write_reg(adap, MYPF_REG(A_PCIE_PF_CLI), 0);
	if (t4_slow_intr_handler(adap) | process_intrq(adap))
		return IRQ_HANDLED;
	return IRQ_NONE;             /* probably shared interrupt */
}

/**
 *	t4_intr_handler - select the top-level interrupt handler
 *	@adap: the adapter
 *
 *	Selects the top-level interrupt handler based on the type of interrupts
 *	(MSI-X, MSI, or INTx).
 */
irq_handler_t t4_intr_handler(adapter_t *adap)
{
	if (adap->flags & USING_MSIX)
		return t4_sge_intr_msix;
	if (adap->flags & USING_MSI)
		return t4_intr_msi;
	return t4_intr_intx;
}

/**
 *	sge_rx_timer_cb - perform periodic maintenance of SGE Rx queues
 *	@data: the adapter
 *
 *	Runs periodically from a timer to perform maintenance of SGE Rx queues.
 *	It performs two tasks:
 *
 *	a) Replenishes Rx queues that have run out due to memory shortage.
 *	Normally new Rx buffers are added as existing ones are consumed but
 *	when out of memory a queue can become empty.  We schedule NAPI to do
 *	the actual refill.
 *
 *	b) Checks that the SGE is not stuck trying to deliver packets.  This
 *	typically indicates a programming error that has caused an Rx queue to
 *	be exhausted.
 */
static void sge_rx_timer_cb(unsigned long data)
{
	unsigned long m;
	unsigned int i, state, cnt[2];
	struct adapter *adap = (struct adapter *)data;
	struct sge *s = &adap->sge;

	for (i = 0; i < ARRAY_SIZE(s->starving_fl); i++)
		for (m = s->starving_fl[i]; m; m &= m - 1) {
			struct sge_eth_rxq *rxq;
			unsigned int id = __ffs(m) + i * BITS_PER_LONG;
			struct sge_fl *fl = s->egr_map[id];

			clear_bit(id, s->starving_fl);
			smp_mb__after_clear_bit();

			/*
			 * Since we are accessing fl without a lock there's a
			 * small probability of a false positive where we
			 * schedule napi but the FL is no longer starving.
			 * No biggie.
			 */
			if (fl_starving(adap, fl)) {
				rxq = container_of(fl, struct sge_eth_rxq, fl);
				if (napi_reschedule(&rxq->rspq.napi))
					fl->starving++;
				else
					set_bit(id, s->starving_fl);
			}
		}

	t4_write_reg(adap, A_SGE_DEBUG_INDEX, 13);
	cnt[0] = t4_read_reg(adap, A_SGE_DEBUG_DATA_HIGH);
	cnt[1] = t4_read_reg(adap, A_SGE_DEBUG_DATA_LOW);

	for (i = 0; i < 2; i++)
		if (cnt[i] >= s->starve_thres) {
			if (s->idma_state[i] || cnt[i] == 0xffffffff)
				continue;
			s->idma_state[i] = 1;
			t4_write_reg(adap, A_SGE_DEBUG_INDEX, 0);
			m = t4_read_reg(adap, A_SGE_DEBUG_DATA_LOW) >> (i * 9);
			state = m & 0x3f;
			t4_write_reg(adap, A_SGE_DEBUG_INDEX, 11);
			m = t4_read_reg(adap, A_SGE_DEBUG_DATA_LOW) >> (i * 16);
			CH_WARN(adap, "SGE idma%u stuck in state %u, "
				"queue %lu\n", i, state, m & 0xffff);
		} else if (s->idma_state[i])
			s->idma_state[i] = 0;

	mod_timer(&s->rx_timer, jiffies + RX_QCHECK_PERIOD);
}

/**
 *	send_flush_wr - send a Flush Work Request on a TX Queue
 *	@adapter: the adapter
 *	@txq: TX Queue to flush
 *
 *	Send a Flush Work Request on the indicated TX Queue with a request to
 *	updated the Status Page of the TX Queue when the Flush Work Request
 *	is processed.  This will allow us to determine when all of the
 *	preceeding TX Requests have been processed.
 */
static void send_flush_wr(struct adapter *adapter, struct sge_eth_txq *txq)
{
	int  credits;
	unsigned int ndesc;
	struct fw_eq_flush_wr *fwr;
	struct sk_buff *skb;
	unsigned int len;

	/*
	 * See if there's space in the TX Queue to fit the Flush Work Request.
	 * If not, we simply return.
	 */
	len = sizeof *fwr;
	ndesc = DIV_ROUND_UP(len, sizeof(struct tx_desc));
	credits = txq_avail(&txq->q) - ndesc;
	if (unlikely(credits < 0))
		return;

	/*
	 * Allocate an skb to hold the Flush Work Request and initialize it 
	 * with the flush request.
	 */
	skb = alloc_skb(len, GFP_ATOMIC);
	if (unlikely(!skb))
		return;
	fwr = (struct fw_eq_flush_wr *)__skb_put(skb, len);
	memset(fwr, 0, sizeof(*fwr));

	fwr->opcode = htonl(V_FW_WR_OP(FW_EQ_FLUSH_WR));
	fwr->equiq_to_len16 = cpu_to_be32(F_FW_WR_EQUEQ |
					  V_FW_WR_LEN16(len / 16));

	/*
	 * If the Flush Work Request fills up the TX Queue to the point where
	 * we don't have enough room for a maximum sized TX Request, then
	 * we need to stop further TX Requests and request that the firmware
	 * notify us with an interrupt when it processes this request.
	 */
	if (unlikely(credits < ETHTXQ_STOP_THRES)) {
		eth_txq_stop(txq);
		fwr->equiq_to_len16 |= cpu_to_be32(F_FW_WR_EQUIQ);
	}

	/*
	 * Copy the Flush Work Request into the TX Queue and notify the
	 * hardware that we've given it some more to do ...
	 */
	inline_tx_skb(skb, &txq->q, &txq->q.desc[txq->q.pidx]);
	txq_advance(&txq->q, ndesc);
	ring_tx_db(adapter, &txq->q, ndesc);

	/*
	 * Free up the skb and return ...
	 */
	kfree_skb(skb);
	return;
}

/**
 *	sge_tx_timer_cb - perform periodic maintenance of SGE Tx queues
 *	@data: the adapter
 *
 *	Runs periodically from a timer to perform maintenance of SGE Tx queues.
 *	It performs two tasks:
 *
 *	a) Restarts offload Tx queues stopped due to I/O MMU mapping errors.
 *
 *	b) Reclaims completed Tx packets for the Ethernet queues.  Normally
 *	packets are cleaned up by new Tx packets, this timer cleans up packets
 *	when no new packets are being submitted.  This is essential for pktgen,
 *	at least.
 */
static void sge_tx_timer_cb(unsigned long data)
{
	unsigned long m, period;
	unsigned int i, budget;
	struct adapter *adap = (struct adapter *)data;
	struct sge *s = &adap->sge;

	for (i = 0; i < ARRAY_SIZE(s->txq_maperr); i++)
		for (m = s->txq_maperr[i]; m; m &= m - 1) {
			unsigned long id = __ffs(m) + i * BITS_PER_LONG;
			struct sge_ofld_txq *txq = s->egr_map[id];

			clear_bit(id, s->txq_maperr);
			tasklet_schedule(&txq->qresume_tsk);
		}

	budget = MAX_TIMER_TX_RECLAIM;
	i = s->ethtxq_rover;
	do {
		struct sge_eth_txq *q = &s->ethtxq[i];

		if (__netif_tx_trylock(q->txq)) {

			if (reclaimable(&q->q)) {
				int avail = reclaimable(&q->q);
				if (avail > budget)
					avail = budget;

				free_tx_desc(adap, &q->q, avail, true);
				q->q.in_use -= avail;

				budget -= avail;
				if (!budget){
					__netif_tx_unlock(q->txq);
					break;
				}
			}

			/* if coalescing is on, ship the coal WR */
			if (q->q.coalesce.idx) {
				ship_tx_pkt_coalesce_wr(adap, q);
				if (adap->tx_coal == 2)
					q->q.coalesce.ison = false;
			}

			/*
			 * If the TX Queue has unreclaimed TX Descriptors and
			 * the last time anything was sent on the associated
			 * net device was more than 5 seconds in the past,
			 * issue a flush request on the TX Queue in order to
			 * get any stranded skb's off the TX Queue.
			 */
			if (q->q.in_use > 0 &&
			    time_after(jiffies,
				       q->txq->dev->trans_start + HZ * 5)) {
				local_bh_disable();
				send_flush_wr(adap, q);
				local_bh_enable();
			}
			__netif_tx_unlock(q->txq);
		}

		i++;
		if (i >= s->ethqsets)
			i = 0;
	} while (i != s->ethtxq_rover);
	s->ethtxq_rover = i;
	/* if we coalesce all the time, we need to run the timer more often */
	period = (adap->tx_coal == 2) ? (TX_QCHECK_PERIOD / 10) :
					TX_QCHECK_PERIOD;
	
	/*
	 * If we found too many reclaimable packets schedule a timer in the
	 * near future to continue where we left off.  Otherwise the next timer
	 * will be at its normal interval.
	 */
	mod_timer(&s->tx_timer, jiffies + (budget ? period : 2));
}

/*
 * @intr_idx: MSI/MSI-X vector if >=0, -(absolute qid + 1) if < 0
 * @cong: < 0 -> no congestion feedback, >= 0 -> congestion channel map
 */
int t4_sge_alloc_rxq(struct adapter *adap, struct sge_rspq *iq, bool fwevtq,
		     struct net_device *dev, int intr_idx,
		     struct sge_fl *fl, rspq_handler_t hnd, int cong)
{
	int ret, flsz = 0;
	struct fw_iq_cmd c;
	struct port_info *pi = netdev_priv(dev);

	/* Size needs to be multiple of 16, including status entry. */
	iq->size = roundup(iq->size, 16);

	iq->desc = alloc_ring(adap->pdev_dev, iq->size, iq->iqe_len, 0,
			      &iq->phys_addr, NULL, 0);
	if (!iq->desc)
		return -ENOMEM;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(V_FW_CMD_OP(FW_IQ_CMD) | F_FW_CMD_REQUEST |
			    F_FW_CMD_WRITE | F_FW_CMD_EXEC |
			    V_FW_IQ_CMD_PFN(adap->pf) | V_FW_IQ_CMD_VFN(0));
	c.alloc_to_len16 = htonl(F_FW_IQ_CMD_ALLOC | F_FW_IQ_CMD_IQSTART |
				 (sizeof(c) / 16));
	c.type_to_iqandstindex = htonl(V_FW_IQ_CMD_TYPE(FW_IQ_TYPE_FL_INT_CAP) |
		V_FW_IQ_CMD_IQASYNCH(fwevtq) | V_FW_IQ_CMD_VIID(pi->viid) |
		V_FW_IQ_CMD_IQANDST(intr_idx < 0) |
		V_FW_IQ_CMD_IQANUD(X_UPDATEDELIVERY_INTERRUPT) |
		V_FW_IQ_CMD_IQANDSTINDEX(intr_idx >= 0 ? intr_idx :
							-intr_idx - 1));
	c.iqdroprss_to_iqesize = htons(V_FW_IQ_CMD_IQPCIECH(pi->tx_chan) |
		F_FW_IQ_CMD_IQGTSMODE |
		V_FW_IQ_CMD_IQINTCNTTHRESH(iq->pktcnt_idx) |
		V_FW_IQ_CMD_IQESIZE(ilog2(iq->iqe_len) - 4));
	c.iqsize = htons(iq->size);
	c.iqaddr = cpu_to_be64(iq->phys_addr);
	if (cong >= 0)
		c.iqns_to_fl0congen = htonl(F_FW_IQ_CMD_IQFLINTCONGEN);

	if (fl) {
		struct sge_eth_rxq *rxq = container_of(fl, struct sge_eth_rxq,
						       fl);

		fl->size = roundup(fl->size, 8);
		fl->desc = alloc_ring(adap->pdev_dev, fl->size, sizeof(__be64),
				      sizeof(struct rx_sw_desc), &fl->addr,
				      &fl->sdesc, STAT_LEN);
		if (!fl->desc)
			goto fl_nomem;

		flsz = fl->size / 8 + STAT_LEN / sizeof(struct tx_desc);
		c.iqns_to_fl0congen |=
			htonl(V_FW_IQ_CMD_FL0HOSTFCMODE(X_HOSTFCMODE_NONE) |
			      (unlikely(rxq->useskbs)
			       ? 0
			       : F_FW_IQ_CMD_FL0PACKEN) |
			      F_FW_IQ_CMD_FL0FETCHRO | F_FW_IQ_CMD_FL0DATARO |
			      F_FW_IQ_CMD_FL0PADEN);
		if (cong >= 0)
			c.iqns_to_fl0congen |=
				htonl(V_FW_IQ_CMD_FL0CNGCHMAP(cong) |
				      F_FW_IQ_CMD_FL0CONGCIF |
				      F_FW_IQ_CMD_FL0CONGEN);
		c.fl0dcaen_to_fl0cidxfthresh =
			htons(V_FW_IQ_CMD_FL0FBMIN(X_FETCHBURSTMIN_64B) |
			      V_FW_IQ_CMD_FL0FBMAX(X_FETCHBURSTMAX_512B));
		c.fl0size = htons(flsz);
		c.fl0addr = cpu_to_be64(fl->addr);
	}

	ret = t4_wr_mbox(adap, adap->mbox, &c, sizeof(c), &c);
	if (ret)
		goto err;

	netif_napi_add(dev, &iq->napi, napi_rx_handler, 64);
	iq->cur_desc = iq->desc;
	iq->cidx = 0;
	iq->gen = 1;
	iq->next_intr_params = iq->intr_params;
	iq->cntxt_id = ntohs(c.iqid);
	iq->abs_id = ntohs(c.physiqid);
	iq->size--;                           /* subtract status entry */
	iq->netdev = dev;   // XXX use napi.dev in newer kernels
	iq->handler = hnd;

	/* set offset to -1 to distinguish ingress queues without FL */
	iq->offset = fl ? 0 : -1;

	adap->sge.ingr_map[iq->cntxt_id - adap->sge.ingr_start] = iq;

	if (fl) {
		fl->cntxt_id = ntohs(c.fl0id);
		fl->avail = fl->pend_cred = 0;
		fl->pidx = fl->cidx = 0;
		fl->alloc_failed = fl->large_alloc_failed = fl->starving = 0;
		adap->sge.egr_map[fl->cntxt_id - adap->sge.egr_start] = fl;
		refill_fl(adap, fl, fl_cap(fl), GFP_KERNEL);
	}
	return 0;

fl_nomem:
	ret = -ENOMEM;
err:
	if (iq->desc) {
		dma_free_coherent(adap->pdev_dev, iq->size * iq->iqe_len,
				  iq->desc, iq->phys_addr);
		iq->desc = NULL;
	}
	if (fl && fl->desc) {
		kfree(fl->sdesc);
		fl->sdesc = NULL;
		dma_free_coherent(adap->pdev_dev, flsz * sizeof(struct tx_desc),
				  fl->desc, fl->addr);
		fl->desc = NULL;
	}
	return ret;
}

static void init_txq(struct adapter *adap, struct sge_txq *q, unsigned int id)
{
	q->in_use = 0;
	q->cidx = q->pidx = 0;
	q->stops = q->restarts = 0;
	q->coalesce.idx = q->coalesce.flits = 0;
	q->coalesce.ison = q->coalesce.intr = false;
	q->stat = (void *)&q->desc[q->size];
	q->cntxt_id = id;
	adap->sge.egr_map[id - adap->sge.egr_start] = q;
}

int t4_sge_alloc_eth_txq(struct adapter *adap, struct sge_eth_txq *txq,
			 struct net_device *dev, struct netdev_queue *netdevq,
			 unsigned int iqid)
{
	int ret, nentries;
	struct fw_eq_eth_cmd c;
	struct port_info *pi = netdev_priv(dev);

	/* Add status entries */
	nentries = txq->q.size + STAT_LEN / sizeof(struct tx_desc);

	txq->q.desc = alloc_ring(adap->pdev_dev, txq->q.size,
			sizeof(struct tx_desc), sizeof(struct tx_sw_desc),
			&txq->q.phys_addr, &txq->q.sdesc, STAT_LEN);
	if (!txq->q.desc)
		return -ENOMEM;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(V_FW_CMD_OP(FW_EQ_ETH_CMD) | F_FW_CMD_REQUEST |
			    F_FW_CMD_WRITE | F_FW_CMD_EXEC |
			    V_FW_EQ_ETH_CMD_PFN(adap->pf) |
			    V_FW_EQ_ETH_CMD_VFN(0));
	c.alloc_to_len16 = htonl(F_FW_EQ_ETH_CMD_ALLOC |
				 F_FW_EQ_ETH_CMD_EQSTART | (sizeof(c) / 16));
	c.viid_pkd = htonl(V_FW_EQ_ETH_CMD_VIID(pi->viid));
	c.fetchszm_to_iqid =
		htonl(V_FW_EQ_ETH_CMD_HOSTFCMODE(X_HOSTFCMODE_STATUS_PAGE) |
		      V_FW_EQ_ETH_CMD_PCIECHN(pi->tx_chan) |
		      F_FW_EQ_ETH_CMD_FETCHRO | V_FW_EQ_ETH_CMD_IQID(iqid));
	c.dcaen_to_eqsize =
		htonl(V_FW_EQ_ETH_CMD_FBMIN(X_FETCHBURSTMIN_64B) |
		      V_FW_EQ_ETH_CMD_FBMAX(X_FETCHBURSTMAX_512B) |
		      V_FW_EQ_ETH_CMD_CIDXFTHRESH(X_CIDXFLUSHTHRESH_32) |
		      V_FW_EQ_ETH_CMD_EQSIZE(nentries));
	c.eqaddr = cpu_to_be64(txq->q.phys_addr);

	ret = t4_wr_mbox(adap, adap->mbox, &c, sizeof(c), &c);
	if (ret) {
		kfree(txq->q.sdesc);
		txq->q.sdesc = NULL;
		dma_free_coherent(adap->pdev_dev,
				  nentries * sizeof(struct tx_desc),
				  txq->q.desc, txq->q.phys_addr);
		txq->q.desc = NULL;
		return ret;
	}

	init_txq(adap, &txq->q, G_FW_EQ_ETH_CMD_EQID(ntohl(c.eqid_pkd)));
	txq->txq = netdevq;
	txq->tso = txq->tx_cso = txq->vlan_ins = 0;
	txq->mapping_err = 0;
	return 0;
}

int t4_sge_alloc_ctrl_txq(struct adapter *adap, struct sge_ctrl_txq *txq,
			  struct net_device *dev, unsigned int iqid,
			  unsigned int cmplqid)
{
	int ret, nentries;
	struct fw_eq_ctrl_cmd c;
	struct port_info *pi = netdev_priv(dev);

	/* Add status entries */
	nentries = txq->q.size + STAT_LEN / sizeof(struct tx_desc);

	txq->q.desc = alloc_ring(adap->pdev_dev, nentries,
				 sizeof(struct tx_desc), 0, &txq->q.phys_addr,
				 NULL, 0);
	if (!txq->q.desc)
		return -ENOMEM;

	c.op_to_vfn = htonl(V_FW_CMD_OP(FW_EQ_CTRL_CMD) | F_FW_CMD_REQUEST |
			    F_FW_CMD_WRITE | F_FW_CMD_EXEC |
			    V_FW_EQ_CTRL_CMD_PFN(adap->pf) |
			    V_FW_EQ_CTRL_CMD_VFN(0));
	c.alloc_to_len16 = htonl(F_FW_EQ_CTRL_CMD_ALLOC |
				 F_FW_EQ_CTRL_CMD_EQSTART | (sizeof(c) / 16));
	c.cmpliqid_eqid = htonl(V_FW_EQ_CTRL_CMD_CMPLIQID(cmplqid));
	c.physeqid_pkd = htonl(0);
	c.fetchszm_to_iqid =
		htonl(V_FW_EQ_CTRL_CMD_HOSTFCMODE(X_HOSTFCMODE_STATUS_PAGE) |
		      V_FW_EQ_CTRL_CMD_PCIECHN(pi->tx_chan) |
		      F_FW_EQ_CTRL_CMD_FETCHRO | V_FW_EQ_CTRL_CMD_IQID(iqid));
	c.dcaen_to_eqsize =
		htonl(V_FW_EQ_CTRL_CMD_FBMIN(X_FETCHBURSTMIN_64B) |
		      V_FW_EQ_CTRL_CMD_FBMAX(X_FETCHBURSTMAX_512B) |
		      V_FW_EQ_CTRL_CMD_CIDXFTHRESH(X_CIDXFLUSHTHRESH_32) |
		      V_FW_EQ_CTRL_CMD_EQSIZE(nentries));
	c.eqaddr = cpu_to_be64(txq->q.phys_addr);

	ret = t4_wr_mbox(adap, adap->mbox, &c, sizeof(c), &c);
	if (ret) {
		dma_free_coherent(adap->pdev_dev,
				  nentries * sizeof(struct tx_desc),
				  txq->q.desc, txq->q.phys_addr);
		txq->q.desc = NULL;
		return ret;
	}

	init_txq(adap, &txq->q, G_FW_EQ_CTRL_CMD_EQID(ntohl(c.cmpliqid_eqid)));
	txq->adap = adap;
	skb_queue_head_init(&txq->sendq);
	tasklet_init(&txq->qresume_tsk, restart_ctrlq, (unsigned long)txq);
	txq->full = 0;
	return 0;
}

int t4_sge_alloc_ofld_txq(struct adapter *adap, struct sge_ofld_txq *txq,
			  struct net_device *dev, unsigned int iqid)
{
	int ret, nentries;
	struct fw_eq_ofld_cmd c;
	struct port_info *pi = netdev_priv(dev);

	/* Add status entries */
	nentries = txq->q.size + STAT_LEN / sizeof(struct tx_desc);

	txq->q.desc = alloc_ring(adap->pdev_dev, txq->q.size,
			sizeof(struct tx_desc), sizeof(struct tx_sw_desc),
			&txq->q.phys_addr, &txq->q.sdesc, STAT_LEN);
	if (!txq->q.desc)
		return -ENOMEM;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(V_FW_CMD_OP(FW_EQ_OFLD_CMD) | F_FW_CMD_REQUEST |
			    F_FW_CMD_WRITE | F_FW_CMD_EXEC |
			    V_FW_EQ_OFLD_CMD_PFN(adap->pf) |
			    V_FW_EQ_OFLD_CMD_VFN(0));
	c.alloc_to_len16 = htonl(F_FW_EQ_OFLD_CMD_ALLOC |
				 F_FW_EQ_OFLD_CMD_EQSTART | (sizeof(c) / 16));
	c.fetchszm_to_iqid =
		htonl(V_FW_EQ_OFLD_CMD_HOSTFCMODE(X_HOSTFCMODE_STATUS_PAGE) |
		      V_FW_EQ_OFLD_CMD_PCIECHN(pi->tx_chan) |
		      F_FW_EQ_OFLD_CMD_FETCHRO | V_FW_EQ_OFLD_CMD_IQID(iqid));
	c.dcaen_to_eqsize =
		htonl(V_FW_EQ_OFLD_CMD_FBMIN(X_FETCHBURSTMIN_64B) |
		      V_FW_EQ_OFLD_CMD_FBMAX(X_FETCHBURSTMAX_512B) |
		      V_FW_EQ_OFLD_CMD_CIDXFTHRESH(X_CIDXFLUSHTHRESH_32) |
		      V_FW_EQ_OFLD_CMD_EQSIZE(nentries));
	c.eqaddr = cpu_to_be64(txq->q.phys_addr);

	ret = t4_wr_mbox(adap, adap->mbox, &c, sizeof(c), &c);
	if (ret) {
		kfree(txq->q.sdesc);
		txq->q.sdesc = NULL;
		dma_free_coherent(adap->pdev_dev,
				  nentries * sizeof(struct tx_desc),
				  txq->q.desc, txq->q.phys_addr);
		txq->q.desc = NULL;
		return ret;
	}

	init_txq(adap, &txq->q, G_FW_EQ_OFLD_CMD_EQID(ntohl(c.eqid_pkd)));
	txq->adap = adap;
	skb_queue_head_init(&txq->sendq);
	tasklet_init(&txq->qresume_tsk, restart_ofldq, (unsigned long)txq);
	txq->full = 0;
	txq->mapping_err = 0;

	return 0;
}

static void free_txq(struct adapter *adap, struct sge_txq *q)
{
	dma_free_coherent(adap->pdev_dev,
			  q->size * sizeof(struct tx_desc) + STAT_LEN,
			  q->desc, q->phys_addr);
	q->cntxt_id = 0;
	q->sdesc = NULL;
	q->desc = NULL;
}

static void free_rspq_fl(struct adapter *adap, struct sge_rspq *rq,
			 struct sge_fl *fl)
{
	unsigned int fl_id = fl ? fl->cntxt_id : 0xffff;

	adap->sge.ingr_map[rq->cntxt_id - adap->sge.ingr_start] = NULL;
	t4_iq_free(adap, adap->mbox, adap->pf, 0, FW_IQ_TYPE_FL_INT_CAP,
		   rq->cntxt_id, fl_id, 0xffff);
	dma_free_coherent(adap->pdev_dev, (rq->size + 1) * rq->iqe_len,
			  rq->desc, rq->phys_addr);
	netif_napi_del(&rq->napi);
	rq->netdev = NULL;
	rq->cntxt_id = rq->abs_id = 0;
	rq->desc = NULL;

	if (fl) {
		free_rx_bufs(adap, fl, fl->avail);
		dma_free_coherent(adap->pdev_dev, fl->size * 8 + STAT_LEN,
				  fl->desc, fl->addr);
		kfree(fl->sdesc);
		fl->sdesc = NULL;
		fl->cntxt_id = 0;
		fl->desc = NULL;
	}
}

/**
 *	t4_free_ofld_rxqs - free a block of consecutive Rx queues
 *	@adap: the adapter
 *	@n: number of queues
 *	@q: pointer to first queue
 *
 *	Release the resources of a consecutive block of offload Rx queues.
 */
void t4_free_ofld_rxqs(struct adapter *adap, int n, struct sge_ofld_rxq *q)
{
	for ( ; n; n--, q++)
		if (q->rspq.desc)
			free_rspq_fl(adap, &q->rspq, &q->fl);
}

/**
 *	t4_free_sge_resources - free SGE resources
 *	@adap: the adapter
 *
 *	Frees resources used by the SGE queue sets.
 */
void t4_free_sge_resources(struct adapter *adap)
{
	int i;
	struct sge_eth_rxq *eq = adap->sge.ethrxq;
	struct sge_eth_txq *etq = adap->sge.ethtxq;

	/* clean up Ethernet Tx/Rx queues */
	for (i = 0; i < adap->sge.ethqsets; i++, eq++, etq++) {
		if (eq->rspq.desc)
			free_rspq_fl(adap, &eq->rspq, &eq->fl);
		if (etq->q.desc) {
			t4_eth_eq_free(adap, adap->mbox, adap->pf, 0,
				       etq->q.cntxt_id);
			free_tx_desc(adap, &etq->q, etq->q.in_use, true);
			kfree(etq->q.sdesc);
			free_txq(adap, &etq->q);
		}
	}

	/* clean up TOE, RDMA and iSCSI Rx queues */
	t4_free_ofld_rxqs(adap, adap->sge.ofldqsets, adap->sge.ofldrxq);
	t4_free_ofld_rxqs(adap, adap->sge.rdmaqs, adap->sge.rdmarxq);
	t4_free_ofld_rxqs(adap, adap->sge.niscsiq, adap->sge.iscsirxq);

	/* clean up offload Tx queues */
	for (i = 0; i < ARRAY_SIZE(adap->sge.ofldtxq); i++) {
		struct sge_ofld_txq *q = &adap->sge.ofldtxq[i];

		if (q->q.desc) {
			tasklet_kill(&q->qresume_tsk);
			t4_ofld_eq_free(adap, adap->mbox, adap->pf,
					0, q->q.cntxt_id);
			free_tx_desc(adap, &q->q, q->q.in_use, false);
			kfree(q->q.sdesc);
			__skb_queue_purge(&q->sendq);
			free_txq(adap, &q->q);
		}
	}

	/* clean up control Tx queues */
	for (i = 0; i < ARRAY_SIZE(adap->sge.ctrlq); i++) {
		struct sge_ctrl_txq *cq = &adap->sge.ctrlq[i];

		if (cq->q.desc) {
			tasklet_kill(&cq->qresume_tsk);
			t4_ctrl_eq_free(adap, adap->mbox, adap->pf, 0,
					cq->q.cntxt_id);
			__skb_queue_purge(&cq->sendq);
			free_txq(adap, &cq->q);
		}
	}

	if (adap->sge.fw_evtq.desc)
		free_rspq_fl(adap, &adap->sge.fw_evtq, NULL);

	if (adap->sge.intrq.desc)
		free_rspq_fl(adap, &adap->sge.intrq, NULL);

	/* clear the reverse egress queue map */
	memset(adap->sge.egr_map, 0, sizeof(adap->sge.egr_map));
}

void t4_sge_start(struct adapter *adap)
{
	adap->sge.ethtxq_rover = 0;
	mod_timer(&adap->sge.rx_timer, jiffies + RX_QCHECK_PERIOD);
	mod_timer(&adap->sge.tx_timer, jiffies + TX_QCHECK_PERIOD);
}

/**
 *	t4_sge_stop - disable SGE operation
 *	@adap: the adapter
 *
 *	Stop tasklets and timers associated with the DMA engine.  Note that
 *	this is effective only if measures have been taken to disable any HW
 *	events that may restart them.
 */
void t4_sge_stop(struct adapter *adap)
{
	int i;
	struct sge *s = &adap->sge;

	if (in_interrupt())  /* actions below require waiting */
		return;

	if (s->rx_timer.function)
		del_timer_sync(&s->rx_timer);
	if (s->tx_timer.function)
		del_timer_sync(&s->tx_timer);

	for (i = 0; i < ARRAY_SIZE(s->ofldtxq); i++) {
		struct sge_ofld_txq *q = &s->ofldtxq[i];

		if (q->q.desc)
			tasklet_kill(&q->qresume_tsk);
	}
	for (i = 0; i < ARRAY_SIZE(s->ctrlq); i++) {
		struct sge_ctrl_txq *cq = &s->ctrlq[i];

		if (cq->q.desc)
			tasklet_kill(&cq->qresume_tsk);
	}
}

/**
 *	t4_sge_init - initialize SGE
 *	@adap: the adapter
 *
 *	Performs SGE initialization needed every time after a chip reset.
 *	We do not initialize any of the queues here, instead the driver
 *	top-level must request those individually.
 */
void t4_sge_init(struct adapter *adap)
{
	struct sge *s = &adap->sge;
	unsigned int fl_align_log = ilog2(FL_ALIGN);
	unsigned int s_hps;

	t4_set_reg_field(adap, A_SGE_CONTROL, V_PKTSHIFT(M_PKTSHIFT) |
			 F_EGRSTATUSPAGESIZE |
			 V_INGPADBOUNDARY(M_INGPADBOUNDARY),
			 V_INGPADBOUNDARY(fl_align_log - 5) |
			 V_PKTSHIFT(FL_PKTSHIFT) |
			 V_EGRSTATUSPAGESIZE(STAT_LEN != 64) | F_RXPKTCPLMODE);

        /*
	 * A FL with <= fl_starve_thres buffers is starving and a periodic
	 * timer will attempt to refill it.  This needs to be larger than the
	 * SGE's Egress Congestion Threshold.  If it isn't, then we can get
	 * stuck waiting for new packets while the SGE is waiting for us to
	 * give it more Free List entries.  (Note that the SGE's Egress
	 * Congestion Threshold is in units of 2 Free List pointers.)
	 */
	s->fl_starve_thres
		= G_EGRTHRESHOLD(t4_read_reg(adap, A_SGE_CONM_CTRL))*2 + 1;

	s_hps = (S_HOSTPAGESIZEPF0 +
		 (S_HOSTPAGESIZEPF1 - S_HOSTPAGESIZEPF0) * adap->pf);
	t4_set_reg_field(adap, A_SGE_HOST_PAGE_SIZE,
			 M_HOSTPAGESIZEPF0 << s_hps,
			 (PAGE_SHIFT - 10) << s_hps);
	t4_write_reg(adap, A_SGE_FL_BUFFER_SIZE0+RX_SMALL_PG_BUF*sizeof(u32),
		     PAGE_SIZE);
#if FL_PG_ORDER > 0
	t4_write_reg(adap, A_SGE_FL_BUFFER_SIZE0+RX_LARGE_PG_BUF*sizeof(u32),
		     PAGE_SIZE << FL_PG_ORDER);
#endif
	t4_write_reg(adap, A_SGE_FL_BUFFER_SIZE0+RX_SMALL_MTU_BUF*sizeof(u32),
		     FL_MTU_SMALL_BUFSIZE);
	t4_write_reg(adap, A_SGE_FL_BUFFER_SIZE0+RX_LARGE_MTU_BUF*sizeof(u32),
		     FL_MTU_LARGE_BUFSIZE);

	t4_write_reg(adap, A_SGE_INGRESS_RX_THRESHOLD,
		     V_THRESHOLD_0(s->counter_val[0]) |
		     V_THRESHOLD_1(s->counter_val[1]) |
		     V_THRESHOLD_2(s->counter_val[2]) |
		     V_THRESHOLD_3(s->counter_val[3]));

	t4_write_reg(adap, A_SGE_TIMER_VALUE_0_AND_1,
		     V_TIMERVALUE0(us_to_core_ticks(adap, s->timer_val[0])) |
		     V_TIMERVALUE1(us_to_core_ticks(adap, s->timer_val[1])));
	t4_write_reg(adap, A_SGE_TIMER_VALUE_2_AND_3,
		     V_TIMERVALUE2(us_to_core_ticks(adap, s->timer_val[2])) |
		     V_TIMERVALUE3(us_to_core_ticks(adap, s->timer_val[3])));
	t4_write_reg(adap, A_SGE_TIMER_VALUE_4_AND_5,
		     V_TIMERVALUE4(us_to_core_ticks(adap, s->timer_val[4])) |
		     V_TIMERVALUE5(us_to_core_ticks(adap, s->timer_val[5])));
	setup_timer(&s->rx_timer, sge_rx_timer_cb, (unsigned long)adap);
	setup_timer(&s->tx_timer, sge_tx_timer_cb, (unsigned long)adap);
	s->starve_thres = core_ticks_per_usec(adap) * 1000000;  /* 1 s */
	s->idma_state[0] = s->idma_state[1] = 0;
	spin_lock_init(&s->intrq_lock);
}
