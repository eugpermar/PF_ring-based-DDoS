/*
 * This file is part of the Chelsio T4 Ethernet driver for Linux.
 *
 * Copyright (C) 2003-2010 Chelsio Communications.  All rights reserved.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the LICENSE file included in this
 * release for licensing terms and conditions.
 */

/* This file should not be included directly.  Include common.h instead. */

#ifndef __T4_ADAPTER_H__
#define __T4_ADAPTER_H__

#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/cache.h>
#ifdef CONFIG_CHELSIO_T4_OFFLOAD
#include <linux/toedev.h>
#endif
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <asm/io.h>
#include "cxgb4_ofld.h"
#include "cxgb4_compat.h"
#include "t4_regs_values.h"

#ifdef T4_TRACE
# include "trace.h"
# define NTRACEBUFS 8
#endif

#define CH_ERR(adap, fmt, ...)   dev_err(adap->pdev_dev, fmt, ## __VA_ARGS__)
#define CH_WARN(adap, fmt, ...)  dev_warn(adap->pdev_dev, fmt, ## __VA_ARGS__)
#define CH_ALERT(adap, fmt, ...) dev_alert(adap->pdev_dev, fmt, ## __VA_ARGS__)

#define CH_WARN_RATELIMIT(adap, fmt, ...)  do {\
	if (printk_ratelimit()) \
		dev_warn(adap->pdev_dev, fmt, ## __VA_ARGS__); \
} while (0)

/*
 * More powerful macro that selectively prints messages based on msg_enable.
 * For info and debugging messages.
 */
#define CH_MSG(adapter, level, category, fmt, ...) do { \
	if ((adapter)->msg_enable & NETIF_MSG_##category) \
		dev_printk(KERN_##level, adapter->pdev_dev, fmt, \
			   ## __VA_ARGS__); \
} while (0)

#ifdef DEBUG
# define CH_DBG(adapter, category, fmt, ...) \
	CH_MSG(adapter, DEBUG, category, fmt, ## __VA_ARGS__)
#else
# define CH_DBG(adapter, category, fmt, ...)
#endif

#define CH_DUMP_MBOX(adap, mbox, data_reg, size) \
	CH_MSG(adap, INFO, MBOX, \
	       "mbox %u: %llx %llx %llx %llx %llx %llx %llx %llx\n", (mbox), \
	       (unsigned long long)t4_read_reg64(adap, data_reg), \
	       (unsigned long long)t4_read_reg64(adap, data_reg + 8), \
	       (unsigned long long)t4_read_reg64(adap, data_reg + 16), \
	       (unsigned long long)t4_read_reg64(adap, data_reg + 24), \
	       (unsigned long long)t4_read_reg64(adap, data_reg + 32), \
	       (unsigned long long)t4_read_reg64(adap, data_reg + 40), \
	       (unsigned long long)t4_read_reg64(adap, data_reg + 48), \
	       (unsigned long long)t4_read_reg64(adap, data_reg + 56));

/* Additional NETIF_MSG_* categories */
#define NETIF_MSG_MBOX 0x4000000
#define NETIF_MSG_MMIO 0x8000000

enum {
	MAX_ETH_QSETS = 32,           /* # of Ethernet Tx/Rx queue sets */
	MAX_OFLD_QSETS = 16,          /* # of offload Tx/Rx queue sets */
	MAX_CTRL_QUEUES = NCHAN,      /* # of control Tx queues */
	MAX_RDMA_QUEUES = NCHAN,      /* # of streaming RDMA Rx queues */
	MAX_ISCSI_QUEUES = NCHAN,     /* # of streaming iSCSI Rx queues */
};

enum {
	MAX_EGRQ = 128,         /* max # of egress queues, including FLs */
	MAX_INGQ = 64           /* max # of interrupt-capable ingress queues */
};

struct adapter;
struct vlan_group;
struct sge_eth_rxq;
struct sge_rspq;

struct port_info {
	struct adapter *adapter;
	struct vlan_group *vlan_grp;
	struct sge_eth_rxq *qs;       /* first Rx queue for this port */
	u16    viid;
	s16    xact_addr_filt;        /* index of exact MAC address filter */
	u16    rss_size;              /* size of VI's RSS table slice */
	s8     mdio_addr;
	u8     port_type;
	u8     mod_type;
	u8     port_id;
	u8     tx_chan;
	u8     lport;                 /* associated offload logical port */
	u8     rx_offload;            /* CSO, etc */
	u8     nqsets;                /* # of qsets */
	u8     first_qset;            /* index of first qset */
	struct link_config link_cfg;
};

/* port_info.rx_offload flags */
enum {
	RX_CSO = 1 << 0,
};

struct work_struct;
struct dentry;
struct proc_dir_entry;

enum {                                 /* adapter flags */
	FULL_INIT_DONE     = (1 << 0),
	USING_MSI          = (1 << 1),
	USING_MSIX         = (1 << 2),
	QUEUES_BOUND       = (1 << 3),
	FW_OK              = (1 << 4),
	RSS_TNLALLLOOKUP   = (1 << 5),
};

struct rx_sw_desc;

struct sge_fl {                     /* SGE free-buffer queue state */
	unsigned int avail;         /* # of available Rx buffers */
	unsigned int pend_cred;     /* new buffers since last FL DB ring */
	unsigned int cidx;          /* consumer index */
	unsigned int pidx;          /* producer index */
	unsigned long alloc_failed; /* # of times buffer allocation failed */
	unsigned long large_alloc_failed;
	unsigned long starving;
	/* RO fields */
	unsigned int cntxt_id;      /* SGE relative QID for the free list */
	unsigned int size;          /* capacity of free list */
	struct rx_sw_desc *sdesc;   /* address of SW Rx descriptor ring */
	__be64 *desc;               /* address of HW Rx descriptor ring */
	dma_addr_t addr;            /* bus address of HW ring start */
};

/* A packet gather list */
struct pkt_gl {
	union {
		skb_frag_t frags[MAX_SKB_FRAGS];
		struct sk_buff *skbs[MAX_SKB_FRAGS];
	} /*UNNAMED*/;
	void *va;                         /* virtual address of first byte */
	unsigned int nfrags;              /* # of fragments */
	unsigned int tot_len;             /* total length of fragments */
	bool useskbs;                     /* use skbs for fragments */
};

typedef int (*rspq_handler_t)(struct sge_rspq *q, const __be64 *rsp,
			      const struct pkt_gl *gl);

struct sge_rspq {                   /* state for an SGE response queue */
	struct napi_struct napi;
	const __be64 *cur_desc;     /* current descriptor in queue */
	unsigned int cidx;          /* consumer index */
	u8 gen;                     /* current generation bit */
	u8 intr_params;             /* interrupt holdoff parameters */
	u8 next_intr_params;        /* holdoff params for next interrupt */
	u8 pktcnt_idx;              /* interrupt packet threshold */
	u8 uld;                     /* ULD handling this queue */
	u8 idx;                     /* queue index within its group */
	int offset;                 /* offset into current Rx buffer */
	u16 cntxt_id;               /* SGE relative QID for the response Q */
	u16 abs_id;                 /* absolute SGE id for the response q */
	__be64 *desc;               /* address of HW response ring */
	dma_addr_t phys_addr;       /* physical address of the ring */
	unsigned int iqe_len;       /* entry size */
	unsigned int size;          /* capacity of response queue */
	struct adapter *adapter;
	struct net_device *netdev;  /* associated net device */
	rspq_handler_t handler;
};

struct sge_eth_stats {              /* Ethernet queue statistics */
	unsigned long pkts;         /* # of ethernet packets */
	unsigned long lro_pkts;     /* # of LRO super packets */
	unsigned long lro_merged;   /* # of wire packets merged by LRO */
	unsigned long rx_cso;       /* # of Rx checksum offloads */
	unsigned long vlan_ex;      /* # of Rx VLAN extractions */
	unsigned long rx_drops;     /* # of packets dropped due to no mem */
};

struct sge_eth_rxq {                /* a SW Ethernet Rx queue */
	struct sge_rspq rspq;
	struct sge_fl fl;
	bool useskbs;               /* one ingress packet per skb FL buffer */
	struct sge_eth_stats stats;
} ____cacheline_aligned_in_smp;

struct sge_ofld_stats {             /* offload queue statistics */
	unsigned long pkts;         /* # of packets */
	unsigned long imm;          /* # of immediate-data packets */
	unsigned long an;           /* # of asynchronous notifications */
	unsigned long nomem;        /* # of responses deferred due to no mem */
};

struct sge_ofld_rxq {               /* a SW offload Rx queue */
	struct sge_rspq rspq;
	struct sge_fl fl;
	bool useskbs;
	struct sge_ofld_stats stats;
} ____cacheline_aligned_in_smp;

struct tx_desc {
	__be64 flit[8];
};

struct tx_sw_desc;

struct eth_coalesce {
	unsigned int idx;
	unsigned int len;
	unsigned int flits;
	unsigned int max;
	unsigned char *ptr;
	unsigned char type;
	bool ison;
	bool intr;
};

struct sge_txq {
	unsigned int  in_use;       /* # of in-use Tx descriptors */
	unsigned int  size;         /* # of descriptors */
	unsigned int  cidx;         /* SW consumer index */
	unsigned int  pidx;         /* producer index */
	unsigned long stops;        /* # of times q has been stopped */
	unsigned long restarts;     /* # of queue restarts */
	unsigned int  cntxt_id;     /* SGE relative QID for the Tx Q */
	struct tx_desc *desc;       /* address of HW Tx descriptor ring */
	struct tx_sw_desc *sdesc;   /* address of SW Tx descriptor ring */
	struct eth_coalesce coalesce;
	struct sge_qstat *stat;     /* queue status entry */
	dma_addr_t    phys_addr;    /* physical address of the ring */
};

struct sge_eth_txq {                /* state for an SGE Ethernet Tx queue */
	struct sge_txq q;
	struct netdev_queue *txq;   /* associated netdev TX queue */
	unsigned long tso;          /* # of TSO requests */
	unsigned long tx_cso;       /* # of Tx checksum offloads */
	unsigned long vlan_ins;     /* # of Tx VLAN insertions */
	unsigned long mapping_err;  /* # of I/O MMU packet mapping errors */
	unsigned long coal_wr;      /* # of coalesce WR */
	unsigned long coal_pkts;    /* # of coalesced packets */
} ____cacheline_aligned_in_smp;

struct sge_ofld_txq {               /* state for an SGE offload Tx queue */
	struct sge_txq q;
	struct adapter *adap;
	struct sk_buff_head sendq;  /* list of backpressured packets */
	struct tasklet_struct qresume_tsk; /* restarts the queue */
	u8 full;                    /* the Tx ring is full */
	unsigned long mapping_err;  /* # of I/O MMU packet mapping errors */
} ____cacheline_aligned_in_smp;

struct sge_ctrl_txq {               /* state for an SGE control Tx queue */
	struct sge_txq q;
	struct adapter *adap;
	struct sk_buff_head sendq;  /* list of backpressured packets */
	struct tasklet_struct qresume_tsk; /* restarts the queue */
	u8 full;                    /* the Tx ring is full */
} ____cacheline_aligned_in_smp;

struct sge {
	/*
	 * Keep all the Tx queues before the Rx queues so we can tell easily
	 * what egr_map entries point at.
	 */
	struct sge_eth_txq ethtxq[MAX_ETH_QSETS];
	struct sge_ofld_txq ofldtxq[MAX_OFLD_QSETS];
	struct sge_ctrl_txq ctrlq[MAX_CTRL_QUEUES];

	struct sge_eth_rxq ethrxq[MAX_ETH_QSETS];
	struct sge_ofld_rxq ofldrxq[MAX_OFLD_QSETS];
	struct sge_ofld_rxq rdmarxq[MAX_RDMA_QUEUES];
	struct sge_ofld_rxq iscsirxq[MAX_ISCSI_QUEUES];
	struct sge_rspq fw_evtq ____cacheline_aligned_in_smp;

	struct sge_rspq intrq ____cacheline_aligned_in_smp;
	spinlock_t intrq_lock;

	u16 max_ethqsets;           /* # of available Ethernet queue sets */
	u16 ethqsets;               /* # of active Ethernet queue sets */
	u16 ethtxq_rover;           /* Tx queue to clean up next */
	u16 ofldqsets;              /* # of active offload queue sets */
	u16 rdmaqs;                 /* # of available RDMA Rx queues */
	u16 niscsiq;                /* # of available iSCSI Rx queues */
	u16 ofld_rxq[MAX_OFLD_QSETS];
	u16 rdma_rxq[NCHAN];
	u16 iscsi_rxq[NCHAN];
	u16 timer_val[SGE_NTIMERS];
	u8 counter_val[SGE_NCOUNTERS];
	u32 fl_starve_thres;        /* Free List starvation threshold */
	unsigned int starve_thres;
	u8 idma_state[2];
	unsigned int egr_start;
	unsigned int ingr_start;
	void *egr_map[MAX_EGRQ];    /* qid->queue egress queue map */
	struct sge_rspq *ingr_map[MAX_INGQ]; /* qid->queue ingress queue map */
	DECLARE_BITMAP(starving_fl, MAX_EGRQ);
	DECLARE_BITMAP(txq_maperr, MAX_EGRQ);
	DECLARE_BITMAP(blocked_fl, MAX_EGRQ);
	struct timer_list rx_timer; /* refills starving FLs */
	struct timer_list tx_timer; /* checks Tx queues */
};

#define for_each_ethrxq(sge, i) for (i = 0; i < (sge)->ethqsets; i++)
#define for_each_ofldrxq(sge, i) for (i = 0; i < (sge)->ofldqsets; i++)
#define for_each_rdmarxq(sge, i) for (i = 0; i < (sge)->rdmaqs; i++)
#define for_each_iscsirxq(sge, i) for (i = 0; i < (sge)->niscsiq; i++)

struct l2t_data;
struct filter_info;

struct t4_os_lock {
	spinlock_t lock;
};

struct t4_os_list {
	struct list_head list;
};	

struct adapter {
	void __iomem *regs;
	struct pci_dev *pdev;
	struct device *pdev_dev;
	unsigned long registered_device_map;
	unsigned long flags;

	const char *name;
	unsigned int mbox;
	unsigned int pf;
	int msg_enable;

	struct adapter_params params;
	struct cxgb4_virt_res vres;
	unsigned int swintr;

	unsigned int wol;

	struct {
		unsigned short vec;
		char desc[IFNAMSIZ + 10];
	} msix_info[MAX_INGQ + 1];

#ifdef T4_TRACE
	struct trace_buf *tb[NTRACEBUFS];
#endif

	/* T4 modules */
	struct sge sge;

	struct net_device *port[MAX_NPORTS];
	u8 chan_map[NCHAN];                   /* channel -> port map */

	unsigned int filter_mode;
	struct filter_info *filters;
	struct l2t_data *l2t;

	void *uld_handle[CXGB4_ULD_MAX];
	struct list_head list_node;

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	struct toedev tdev;
#endif
	struct tid_info tids;
	void **tid_release_head;
	spinlock_t tid_release_lock;
	struct work_struct tid_release_task;

#ifdef CONFIG_PCI_IOV
	struct delayed_work vf_monitor_task;
	unsigned int vf_monitor_mask;
#endif

	struct dentry *debugfs_root;
	struct proc_dir_entry *proc_root;

	spinlock_t mdio_lock;
	spinlock_t stats_lock;
	spinlock_t work_lock;

	/* support for single-threading access to adapter mailbox registers */
	struct t4_os_lock mbox_lock;
	struct t4_os_list mbox_list;

	struct mutex user_mutex;

	int tx_coal;

	spinlock_t win0_lock ____cacheline_aligned_in_smp;
};

/**
 * t4_read_reg - read a HW register
 * @adapter: the adapter
 * @reg_addr: the register address
 *
 * Returns the 32-bit value of the given HW register.
 */
static inline u32 t4_read_reg(adapter_t *adapter, u32 reg_addr)
{
	u32 val = readl(adapter->regs + reg_addr);

	CH_DBG(adapter, MMIO, "read register 0x%x value 0x%x\n", reg_addr,
	       val);
	return val;
}

/**
 * t4_write_reg - write a HW register
 * @adapter: the adapter
 * @reg_addr: the register address
 * @val: the value to write
 *
 * Write a 32-bit value into the given HW register.
 */
static inline void t4_write_reg(adapter_t *adapter, u32 reg_addr, u32 val)
{
	CH_DBG(adapter, MMIO, "setting register 0x%x to 0x%x\n", reg_addr,
	       val);
	writel(val, adapter->regs + reg_addr);
}

#ifndef readq
static inline u64 readq(const volatile void __iomem *addr)
{
	return readl(addr) + ((u64)readl(addr + 4) << 32);
}

static inline void writeq(u64 val, volatile void __iomem *addr)
{
	writel(val, addr);
	writel(val >> 32, addr + 4);
}
#endif

/**
 * t4_read_reg64 - read a 64-bit HW register
 * @adapter: the adapter
 * @reg_addr: the register address
 *
 * Returns the 64-bit value of the given HW register.
 */
static inline u64 t4_read_reg64(adapter_t *adapter, u32 reg_addr)
{
	u64 val = readq(adapter->regs + reg_addr);

	CH_DBG(adapter, MMIO, "64-bit read register %#x value %#llx\n",
	       reg_addr, (unsigned long long)val);
	return val;
}

/**
 * t4_write_reg64 - write a 64-bit HW register
 * @adapter: the adapter
 * @reg_addr: the register address
 * @val: the value to write
 *
 * Write a 64-bit value into the given HW register.
 */
static inline void t4_write_reg64(adapter_t *adapter, u32 reg_addr, u64 val)
{
	CH_DBG(adapter, MMIO, "setting register %#x to %#llx\n", reg_addr,
	       (unsigned long long)val);
	writeq(val, adapter->regs + reg_addr);
}

/**
 * t4_os_pci_write_cfg4 - 32-bit write to PCI config space
 * @adapter: the adapter
 * @reg: the register address
 * @val: the value to write
 *
 * Write a 32-bit value into the given register in PCI config space.
 */
static inline void t4_os_pci_write_cfg4(adapter_t *adapter, int reg, u32 val)
{
	pci_write_config_dword(adapter->pdev, reg, val);
}

/**
 * t4_os_pci_read_cfg4 - read a 32-bit value from PCI config space
 * @adapter: the adapter
 * @reg: the register address
 * @val: where to store the value read
 *
 * Read a 32-bit value from the given register in PCI config space.
 */
static inline void t4_os_pci_read_cfg4(adapter_t *adapter, int reg, u32 *val)
{
	pci_read_config_dword(adapter->pdev, reg, val); 
}

/**
 * t4_os_pci_write_cfg2 - 16-bit write to PCI config space
 * @adapter: the adapter
 * @reg: the register address
 * @val: the value to write
 *
 * Write a 16-bit value into the given register in PCI config space.
 */
static inline void t4_os_pci_write_cfg2(adapter_t *adapter, int reg, u16 val)
{
	pci_write_config_word(adapter->pdev, reg, val);
}

/**
 * t4_os_pci_read_cfg2 - read a 16-bit value from PCI config space
 * @adapter: the adapter
 * @reg: the register address
 * @val: where to store the value read
 *
 * Read a 16-bit value from the given register in PCI config space.
 */
static inline void t4_os_pci_read_cfg2(adapter_t *adapter, int reg, u16 *val)
{
	pci_read_config_word(adapter->pdev, reg, val); 
}

/**
 * t4_os_find_pci_capability - lookup a capability in the PCI capability list
 * @adapter: the adapter
 * @cap: the capability
 *
 * Return the address of the given capability within the PCI capability list.
 */
static inline int t4_os_find_pci_capability(adapter_t *adapter, int cap)
{
	return pci_find_capability(adapter->pdev, cap);
}

/**
 * t4_os_pci_save_state - save PCI config state
 * @adapter: the adapter
 *
 * Save the state of PCI config space.
 */
static inline int t4_os_pci_save_state(adapter_t *adapter)
{
	return pci_save_state(adapter->pdev);
}

/**
 * t4_os_pci_restore_state - restore PCI config state
 * @adapter: the adapter
 *
 * Restore previously saved PCI config space.
 */
static inline int t4_os_pci_restore_state(adapter_t *adapter)
{

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38))
	return pci_restore_state(adapter->pdev);
#else
	pci_restore_state(adapter->pdev);
	return(0);
#endif
}

/**
 * t4_os_set_hw_addr - store a port's MAC address in SW
 * @adapter: the adapter
 * @port_idx: the port index
 * @hw_addr: the Ethernet address
 *
 * Store the Ethernet address of the given port in SW.  Called by the common
 * code when it retrieves a port's Ethernet address from EEPROM.
 */
static inline void t4_os_set_hw_addr(adapter_t *adapter, int port_idx,
				     u8 hw_addr[])
{
	memcpy(adapter->port[port_idx]->dev_addr, hw_addr, ETH_ALEN);
	memcpy(adapter->port[port_idx]->perm_addr, hw_addr, ETH_ALEN);
}

/**
 * netdev2pinfo - return the port_info structure associated with a net_device
 * @dev: the netdev
 *
 * Return the struct port_info associated with a net_device
 */
static inline struct port_info *netdev2pinfo(const struct net_device *dev)
{
	return netdev_priv(dev);
}

/**
 * adap2pinfo - return the port_info of a port
 * @adap: the adapter
 * @idx: the port index
 *
 * Return the port_info structure for the port of the given index.
 */
static inline struct port_info *adap2pinfo(struct adapter *adap, int idx)
{
	return netdev_priv(adap->port[idx]);
}

/**
 * netdev2adap - return the adapter structure associated with a net_device
 * @dev: the netdev
 *
 * Return the struct adapter associated with a net_device
 */
static inline struct adapter *netdev2adap(const struct net_device *dev)
{
	return netdev2pinfo(dev)->adapter;
}

/**
 * t4_os_lock_init - initialize spinlock
 * @lock: the spinlock
 */
static inline void t4_os_lock_init(struct t4_os_lock *lock)
{
	spin_lock_init(&lock->lock);
}

/**
 * t4_os_trylock - try to acquire a spinlock
 * @lock: the spinlock
 *
 * Returns 1 if successful and 0 otherwise.
 */
static inline int t4_os_trylock(struct t4_os_lock *lock)
{
	return spin_trylock(&lock->lock);
}

/**
 * t4_os_lock - spin until lock is acquired
 * @lock: the spinlock
 */
static inline void t4_os_lock(struct t4_os_lock *lock)
{
	spin_lock(&lock->lock);
}

/**
 * t4_os_unlock - unlock a spinlock
 * @lock: the spinlock
 */
static inline void t4_os_unlock(struct t4_os_lock *lock)
{
	spin_unlock(&lock->lock);
}

/**
 * t4_os_init_list_head - initialize 
 * @head: head of list to initialize [to empty]
 */
static inline void t4_os_init_list_head(struct t4_os_list *head)
{
	INIT_LIST_HEAD(&head->list);
}

static inline struct t4_os_list *t4_os_list_first_entry(struct t4_os_list *head)
{
	return list_first_entry(&head->list, struct t4_os_list, list);
}

/**
 * t4_os_atomic_add_tail - Enqueue list element atomically onto list
 * @new: the entry to be addded to the queue
 * @head: current head of the linked list
 * @lock: lock to use to guarantee atomicity
 */
static inline void t4_os_atomic_add_tail(struct t4_os_list *new,
					 struct t4_os_list *head,
					 struct t4_os_lock *lock)
{
	t4_os_lock(lock);
	list_add_tail(&new->list, &head->list);
	t4_os_unlock(lock);
}

/**
 * t4_os_atomic_list_del - Dequeue list element atomically from list
 * @entry: the entry to be remove/dequeued from the list.
 * @lock: the spinlock
 */
static inline void t4_os_atomic_list_del(struct t4_os_list *entry,
					 struct t4_os_lock *lock)
{
	t4_os_lock(lock);
	list_del(&entry->list);
	t4_os_unlock(lock);
}

#define OFFLOAD_DEVMAP_BIT 15

#define tdev2adap(d) container_of(d, struct adapter, tdev)
#define cdev2adap(d) container_of(d, struct adapter, cdev)

void t4_os_portmod_changed(const struct adapter *adap, int port_id);
void t4_os_link_changed(struct adapter *adap, int port_id, int link_stat);

void *t4_alloc_mem(size_t size);
void t4_free_mem(void *addr);

void t4_free_sge_resources(struct adapter *adap);
void t4_free_ofld_rxqs(struct adapter *adap, int n, struct sge_ofld_rxq *q);
irq_handler_t t4_intr_handler(struct adapter *adap);
int t4_eth_xmit(struct sk_buff *skb, struct net_device *dev);
int t4_ethrx_handler(struct sge_rspq *q, const __be64 *rsp,
		     const struct pkt_gl *gl);
int t4_mgmt_tx(adapter_t *adap, struct sk_buff *skb);
int t4_ofld_send(struct adapter *adap, struct sk_buff *skb);
int t4_sge_alloc_rxq(struct adapter *adap, struct sge_rspq *iq, bool fwevtq,
		     struct net_device *dev, int intr_idx,
		     struct sge_fl *fl, rspq_handler_t hnd, int cong);
int t4_sge_alloc_eth_txq(struct adapter *adap, struct sge_eth_txq *txq,
			 struct net_device *dev, struct netdev_queue *netdevq,
			 unsigned int iqid);
int t4_sge_alloc_ctrl_txq(struct adapter *adap, struct sge_ctrl_txq *txq,
			  struct net_device *dev, unsigned int iqid,
			  unsigned int cmplqid);
int t4_sge_alloc_ofld_txq(struct adapter *adap, struct sge_ofld_txq *txq,
			  struct net_device *dev, unsigned int iqid);
irqreturn_t t4_sge_intr_msix(int irq, void *cookie);
void t4_sge_init(struct adapter *adap);
void t4_sge_start(struct adapter *adap);
void t4_sge_stop(struct adapter *adap);
int t4_sge_coalesce_handler(struct adapter *adap, struct sge_eth_txq *q);
#endif /* __T4_ADAPTER_H__ */
