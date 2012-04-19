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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/mii.h>
#include <linux/sockios.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/rtnetlink.h>
#include <linux/firmware.h>
#include <linux/log2.h>
#include <linux/sched.h>
#include <linux/string_helpers.h>
#include <linux/sort.h>
#include <linux/aer.h>
#include <linux/vmalloc.h>
#include <linux/notifier.h>
#include <linux/bitmap.h>
#include <linux/mutex.h>
#include <linux/crc32.h>
#include <net/neighbour.h>
#include <net/netevent.h>
#include <asm/uaccess.h>

#include "common.h"
#include "cxgbtool.h"
#include "t4_regs.h"
#include "t4_regs_values.h"
#include "t4_msg.h"
#include "t4_tcb.h"
#include "t4fw_interface.h"

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
#include "l2t.h"
#include "cxgb4_ofld.h"
#endif

#if 0
#include "../bonding/bonding.h"
#undef DRV_VERSION
#endif

#define DRV_VERSION "1.0.2.26"
#define DRV_DESC "Chelsio T4 Network Driver"

/*
 * Max interrupt hold-off timer value in us.  Queues fall back to this value
 * under extreme memory pressure so it's largish to give the system time to
 * recover.
 */
#define MAX_SGE_TIMERVAL 200U

enum {
	/*
	 * Physical Function provisioning constants.
	 */
	PFRES_NVI = 4,			/* # of Virtual Interfaces */
	PFRES_NETHCTRL = 128,		/* # of EQs used for ETH or CTRL Qs */
	PFRES_NIQFLINT = 128,		/* # of ingress Qs/w Free List(s)/intr */
	PFRES_NEQ = 256,		/* # of egress queues */
	PFRES_NIQ = 0,			/* # of ingress queues */
	PFRES_TC = 0,			/* PCI-E traffic class */
	PFRES_NEXACTF = 128,		/* # of exact MPS filters */

	PFRES_R_CAPS = FW_CMD_CAP_PF,
	PFRES_WX_CAPS = FW_CMD_CAP_PF,

#ifdef CONFIG_PCI_IOV
	/*
	 * Virtual Function provisioning constants.  We need two extra Ingress
	 * Queues with Interrupt capability to serve as the VF's Firmware
	 * Event Queue and Forwarded Interrupt Queue (when using MSI mode) --
	 * neither will have Free Lists associated with them).  For each
	 * Ethernet/Control Egress Queue and for each Free List, we need an
	 * Egress Context.
	 */
	VFRES_NPORTS = 1,		/* # of "ports" per VF */
	VFRES_NQSETS = 2,		/* # of "Queue Sets" per VF */

	VFRES_NVI = VFRES_NPORTS,	/* # of Virtual Interfaces */
	VFRES_NETHCTRL = VFRES_NQSETS,	/* # of EQs used for ETH or CTRL Qs */
	VFRES_NIQFLINT = VFRES_NQSETS+2,/* # of ingress Qs/w Free List(s)/intr */
	VFRES_NEQ = VFRES_NQSETS*2,	/* # of egress queues */
	VFRES_NIQ = 0,			/* # of non-fl/int ingress queues */
	VFRES_TC = 0,			/* PCI-E traffic class */
	VFRES_NEXACTF = 16,		/* # of exact MPS filters */

	VFRES_R_CAPS = FW_CMD_CAP_DMAQ|FW_CMD_CAP_VF|FW_CMD_CAP_PORT,
	VFRES_WX_CAPS = FW_CMD_CAP_DMAQ|FW_CMD_CAP_VF,
#endif
};

/*
 * Provide a Port Access Rights Mask for the specified PF/VF.  This is very
 * static and likely not to be useful in the long run.  We really need to
 * implement some form of persistent configuration which the firmware
 * controls.
 */
static unsigned int pfvfres_pmask(struct adapter *adapter,
				  unsigned int pf, unsigned int vf)
{
	unsigned int portn, portvec;

	/*
	 * Give PF's access to all of the ports.
	 */
	if (vf == 0)
		return M_FW_PFVF_CMD_PMASK;

	/*
	 * For VFs, we'll assign them access to the ports based purely on the
	 * PF.  We assign active ports in order, wrapping around if there are
	 * fewer active ports than PFs: e.g. active port[pf % nports].
	 * Unfortunately the adapter's port_info structs haven't been
	 * initialized yet so we have to compute this.
	 */
	if (adapter->params.nports == 0)
		return 0;

	portn = pf % adapter->params.nports;
	portvec = adapter->params.portvec;
	for (;;) {
		/*
		 * Isolate the lowest set bit in the port vector.  If we're at
		 * the port number that we want, return that as the pmask.
		 * otherwise mask that bit out of the port vector and
		 * decrement our port number ...
		 */
		unsigned int pmask = portvec ^ (portvec & (portvec-1));
		if (portn == 0)
			return pmask;
		portn--;
		portvec &= ~pmask;
	}
	/*NOTREACHED*/
}

enum {
	MAX_TXQ_ENTRIES      = 16384,
	MAX_CTRL_TXQ_ENTRIES = 1024,
	MAX_RSPQ_ENTRIES     = 16384,
	MAX_RX_BUFFERS       = 16384,
	MIN_TXQ_ENTRIES      = 32,
	MIN_CTRL_TXQ_ENTRIES = 32,
	MIN_RSPQ_ENTRIES     = 128,
	MIN_FL_ENTRIES       = 16
};

#ifdef CONFIG_PCI_IOV
enum {
	VF_MONITOR_PERIOD = 4 * HZ,
};
#endif

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
/*
 * Host shadow copy of ingress filter entry.  This is in host native format
 * and doesn't match the ordering or bit order, etc. of the hardware or the
 * firmware command.
 */
struct filter_entry {
	/*
	 * Administrative fields for filter.
	 */
	u32 valid:1;		/* filter allocated and valid */
	u32 locked:1;		/* filter is administratively locked */
	u32 pending:1;		/* filter action is pending firmware reply */
	u32 smtidx:8;		/* Source MAC Table index for smac */
	struct l2t_entry *l2t;	/* Layer Two Table entry for dmac */

	/*
	 * The filter itself.  Most of this is a straight copy of information
	 * provided by the extended ioctl().  Some fields are translated to
	 * internal forms -- for instance the Ingress Queue ID passed in from
	 * the ioctl() is translated into the Absolute Ingress Queue ID.
	 */
	struct ch_filter_specification fs;
};
#endif

#define PORT_MASK ((1 << MAX_NPORTS) - 1)

#define DFLT_MSG_ENABLE (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK | \
			 NETIF_MSG_TIMER | NETIF_MSG_IFDOWN | NETIF_MSG_IFUP |\
			 NETIF_MSG_RX_ERR | NETIF_MSG_TX_ERR)

#define EEPROM_MAGIC 0x38E2F10C

#define CH_DEVICE(devid) { PCI_VDEVICE(CHELSIO, devid), 0 }

static struct pci_device_id cxgb4_pci_tbl[] = {
	CH_DEVICE(0xa000),  /* PE10K FPGA */
	CH_DEVICE(0x4400),  /* T440-dbg */
	CH_DEVICE(0x4401),  /* T420-cr */
	CH_DEVICE(0x4402),  /* T422-cr */
	CH_DEVICE(0x4403),  /* T440-cr */
	CH_DEVICE(0x4404),  /* T420-bch */
	CH_DEVICE(0x4405),  /* T440-bch */
	CH_DEVICE(0x4406),  /* T440-ch */
	CH_DEVICE(0x4407),  /* T420-so */
	CH_DEVICE(0x4408),  /* T420-cx */
	CH_DEVICE(0x4409),  /* T420-bt */
	CH_DEVICE(0x440a),  /* T404-bt */
	{ 0, }
};

#define FW_FNAME "cxgb4/t4fw.bin"

MODULE_DESCRIPTION(DRV_DESC);
MODULE_AUTHOR("Chelsio Communications");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, cxgb4_pci_tbl);
MODULE_FIRMWARE(FW_FNAME);

/*
 * The master PF is normally PF0 but can be changed to any existing PF via a
 * module parameter.  Note that PF0 does have extra privileges and can access
 * all the other PFs' VPDs and the entire EEPROM which the other PFs cannot.
 */
static uint master_pf = 0;

module_param(master_pf, uint, 0644);
MODULE_PARM_DESC(master_pf, "Master Physical Function to which to attach");

/*
 * Default message set for the interfaces.  This can be changed after the
 * driver is loaded via "ethtool -s ethX msglvl N".
 */
static int dflt_msg_enable = DFLT_MSG_ENABLE;

module_param(dflt_msg_enable, int, 0644);
MODULE_PARM_DESC(dflt_msg_enable, "Chelsio T4 default message enable bitmap");

/*
 * The driver uses the best interrupt scheme available on a platform in the
 * order MSI-X, MSI, legacy INTx interrupts.  This parameter determines which
 * of these schemes the driver may consider as follows:
 *
 * msi = 2: choose from among all three options
 * msi = 1: only consider MSI and INTx interrupts
 * msi = 0: force INTx interrupts
 */
static int msi = 2;

module_param(msi, int, 0644);
MODULE_PARM_DESC(msi, "whether to use MSI-X, MSI or INTx");

/*
 * Queue interrupt hold-off timer values.  Queues default to the first of these
 * upon creation.
 */
static unsigned int intr_holdoff[SGE_NTIMERS - 1] = { 5, 10, 20, 50, 100 };

module_param_array(intr_holdoff, uint, NULL, 0644);
MODULE_PARM_DESC(intr_holdoff, "values for queue interrupt hold-off timers "
		 "0..4 in microseconds");

static unsigned int intr_cnt[SGE_NCOUNTERS - 1] = { 4, 8, 16 };

module_param_array(intr_cnt, uint, NULL, 0644);
MODULE_PARM_DESC(intr_cnt,
		 "thresholds 1..3 for queue interrupt packet counters");

/*
 * Use skb's for ingress packets.
 */
static int rx_useskbs = 0;

module_param(rx_useskbs, int, 0644);
MODULE_PARM_DESC(rx_useskbs, "one ingress packet per skb Free List buffer");

static int tx_coal = 1;

module_param(tx_coal, int, 0644);
MODULE_PARM_DESC(tx_coal, "use tx WR coalescing, if set to 2, coalescing "
		 " will be used most of the time improving packets per "
		 " second troughput but affecting latency");

static int vf_acls;

#ifdef CONFIG_PCI_IOV
module_param(vf_acls, bool, 0644);
MODULE_PARM_DESC(vf_acls, "if set enable virtualization L2 ACL enforcement");

static unsigned int num_vf[4];

module_param_array(num_vf, uint, NULL, 0644);
MODULE_PARM_DESC(num_vf, "number of VFs for each of PFs 0-3");
#endif

/*
 * If fw_attach is 0 the driver will not connect to FW.  This is intended only
 * for FW debugging.  fw_attach must be 1 for normal operation.
 */
static int fw_attach = 1;

module_param(fw_attach, int, 0644);
MODULE_PARM_DESC(fw_attach, "whether to connect to FW");

/*
 * The filter TCAM has a fixed portion and a variable portion.  The fixed
 * portion can match on source/destination IP IPv4/IPv6 addresses and TCP/UDP
 * ports.  The variable portion is 36 bits which can include things like Exact
 * Match MAC Index (9 bits), Ether Type (16 bits), IP Protocol (8 bits),
 * [Inner] VLAN Tag (17 bits), etc. which, if all were somehow selected, would
 * far exceed the 36-bit budget for this "compressed" header portion of the
 * filter.  Thus, we have a scarce resource which must be carefully managed.
 *
 * By default we set this up to mostly match the set of filter matching
 * capabilities of T3 but with accommodations for some of T4's more
 * interesting features:
 *
 *   { IP Fragment (1), MPS Match Type (3), IP Protocol (8),
 *     [Inner] VLAN (17), Port (3), FCoE (1) }
 */
enum {
	TP_VLAN_PRI_MAP_DEFAULT = HW_TPL_FR_MT_PR_IV_P_FC,
	TP_VLAN_PRI_MAP_FIRST = S_FCOE,
	TP_VLAN_PRI_MAP_LAST = S_FRAGMENTATION,
};

static unsigned int tp_vlan_pri_map = 0x3c3;

module_param(tp_vlan_pri_map, uint, 0644);
MODULE_PARM_DESC(tp_vlan_pri_map, "global compressed filter configuration");

static struct dentry *cxgb4_debugfs_root;
static struct proc_dir_entry *cxgb4_proc_root;

static LIST_HEAD(adapter_list);
static DEFINE_MUTEX(uld_mutex);
static struct cxgb4_uld_info ulds[CXGB4_ULD_MAX];
static const char *uld_str[] = { "RDMA", "iSCSI", "TOE" };

/**
 *	link_report - show link status and link speed/duplex
 *	@dev: the port whose settings are to be reported
 *
 *	Shows the link status, speed, and duplex of a port.
 */
static void link_report(struct net_device *dev)
{
	if (!netif_carrier_ok(dev))
		printk(KERN_INFO "%s: link down\n", dev->name);
	else {
		static const char *fc[] = { "no", "Rx", "Tx", "Tx/Rx" };

		const char *s = "10Mbps";
		const struct port_info *p = netdev_priv(dev);

		switch (p->link_cfg.speed) {
		case SPEED_10000:
			s = "10Gbps";
			break;
		case SPEED_1000:
			s = "1000Mbps";
			break;
		case SPEED_100:
			s = "100Mbps";
			break;
		}

		printk(KERN_INFO "%s: link up, %s, full-duplex, %s PAUSE\n",
		       dev->name, s, fc[p->link_cfg.fc]);
	}
}

/**
 *	t4_os_link_changed - handle link status changes
 *	@adapter: the adapter associated with the link change
 *	@port_id: the port index whose link status has changed
 *	@link_stat: the new status of the link
 *
 *	This is the OS-dependent handler for link status changes.  The OS
 *	neutral handler takes care of most of the processing for these events,
 *	then calls this handler for any OS-specific processing.
 */
void t4_os_link_changed(struct adapter *adapter, int port_id, int link_stat)
{
	struct net_device *dev = adapter->port[port_id];

	/* Skip changes from disabled ports. */
	if (netif_running(dev) && link_stat != netif_carrier_ok(dev)) {
		if (link_stat)
			netif_carrier_on(dev);
		else
			netif_carrier_off(dev);

		link_report(dev);
	}
}

/**
 *	t4_os_portmod_changed - handle port module changes
 *	@adap: the adapter associated with the module change
 *	@port_id: the port index whose module status has changed
 *
 *	This is the OS-dependent handler for port module changes.  It is
 *	invoked when a port module is removed or inserted for any OS-specific
 *	processing.
 */
void t4_os_portmod_changed(const struct adapter *adap, int port_id)
{
	static const char *mod_str[] = {
		NULL, "LR", "SR", "ER", "TWINAX", "active TWINAX", "LRM"
	};

	const struct net_device *dev = adap->port[port_id];
	const struct port_info *pi = netdev_priv(dev);

	if (pi->mod_type == FW_PORT_MOD_TYPE_NONE)
		printk(KERN_INFO "%s: port module unplugged\n", dev->name);
	else if (pi->mod_type < ARRAY_SIZE(mod_str))
		printk(KERN_INFO "%s: %s port module inserted\n", dev->name,
		       mod_str[pi->mod_type]);
	else if (pi->mod_type == FW_PORT_MOD_TYPE_NOTSUPPORTED)
		printk(KERN_INFO "%s: unsupported optical port module "
		 	"inserted\n", dev->name);
	else if (pi->mod_type == FW_PORT_MOD_TYPE_UNKNOWN)
		printk(KERN_INFO "%s: unknown port module inserted, forcing "
		       "TWINAX\n", dev->name);
	else
		printk(KERN_INFO "%s: unknown module type %d inserted\n",
		       dev->name, pi->mod_type);
}

/*
 * Configure the exact and hash address filters to handle a port's multicast
 * and secondary unicast MAC addresses.
 */
static int set_addr_filters(const struct net_device *dev, bool sleep)
{
	u64 mhash = 0;
	u64 uhash = 0;
	bool free = true;
	unsigned int offset, naddr;
	const u8 *addr[7];
	int ret;
	const struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	/* first do the secondary unicast addresses */
	for (offset = 0; ; offset += naddr) {
		naddr = collect_netdev_uc_list_addrs(dev, addr, offset,
						     ARRAY_SIZE(addr));
		if (naddr == 0)
			break;

		ret = t4_alloc_mac_filt(adapter, adapter->mbox, pi->viid, free,
					naddr, addr, NULL, &uhash, sleep);
		if (ret < 0)
			return ret;
		free = false;
	}

	/* next set up the multicast addresses */
	for (offset = 0; ; offset += naddr) {
		naddr = collect_netdev_mc_list_addrs(dev, addr, offset,
						     ARRAY_SIZE(addr));
		if (naddr == 0)
			break;

		ret = t4_alloc_mac_filt(adapter, adapter->mbox, pi->viid, free,
					naddr, addr, NULL, &mhash, sleep);
		if (ret < 0)
			return ret;
		free = false;
	}

	return t4_set_addr_hash(adapter, adapter->mbox, pi->viid, uhash != 0,
				uhash | mhash, sleep);
}

/*
 * Set Rx properties of a port, such as promiscruity, address filters, and MTU.
 * If @mtu is -1 it is left unchanged.
 */
static int set_rxmode(struct net_device *dev, int mtu, bool sleep_ok)
{
	int ret;
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	ret = set_addr_filters(dev, sleep_ok);
	if (ret == 0)
		ret = t4_set_rxmode(adapter, adapter->mbox, pi->viid, mtu,
				    (dev->flags & IFF_PROMISC) ? 1 : 0,
				    (dev->flags & IFF_ALLMULTI) ? 1 : 0,
				    1, -1, sleep_ok);
	return ret;
}

static void cxgb_set_rxmode(struct net_device *dev)
{
	/* unfortunately we can't return errors to the stack */
	set_rxmode(dev, -1, false);
}

/**
 *	link_start - enable a port
 *	@dev: the port to enable
 *
 *	Performs the MAC and PHY actions needed to enable a port.
 */
static int link_start(struct net_device *dev)
{
	int ret;
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	/*
	 * We do not set address filters and promiscuity here, the stack does
	 * that step explicitly.
	 */
	ret = t4_set_rxmode(adapter, adapter->mbox, pi->viid, dev->mtu, -1,
			    -1, -1, -1, true);
	if (ret == 0) {
		ret = t4_change_mac(adapter, adapter->mbox, pi->viid,
				    pi->xact_addr_filt, dev->dev_addr, true,
				    true);
		if (ret >= 0) {
			pi->xact_addr_filt = ret;
			ret = 0;
		}
	}
	if (ret == 0)
		ret = t4_link_start(adapter, adapter->mbox, pi->tx_chan,
				    &pi->link_cfg);
	if (ret == 0)
		ret = t4_enable_vi(adapter, adapter->mbox, pi->viid, true,
				   true);
	return ret;
}

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
/*
 * Clear a filter and release any of its resources that we own.  This also
 * clears the filter's "pending" status.
 */
static void clear_filter(struct adapter *adap, struct filter_entry *f)
{
	/*
	 * If the new or old filter have loopback rewriteing rules then we'll
	 * need to free any existing Layer Two Table (L2T) entries of the old
	 * filter rule.  The firmware will handle freeing up any Source MAC
	 * Table (SMT) entries used for rewriting Source MAC Addresses in
	 * loopback rules.
	 */
	if (f->l2t)
		cxgb4_l2t_release(f->l2t);

	/*
	 * The zeroing of the filter rule below clears the filter valid,
	 * pending, locked flags, l2t pointer, etc. so it's all we need for
	 * this operation.
	 */
	memset(f, 0, sizeof(*f));
}

/*
 * Handle a filter write/deletion reply.
 */
static void filter_rpl(struct adapter *adap, const struct cpl_set_tcb_rpl *rpl)
{
	unsigned int idx = GET_TID(rpl);

	if (idx >= adap->tids.ftid_base &&
	    (idx -= adap->tids.ftid_base) < adap->tids.nftids) {
		unsigned int ret = G_COOKIE(rpl->cookie);
		struct filter_entry *f = &adap->tids.ftid_tab[idx];

		if (ret == FW_FILTER_WR_FLT_DELETED) {
			/*
			 * Clear the filter when we get confirmation from the
			 * hardware that the filter has been deleted.
			 */
			clear_filter(adap, f);
		} else if (ret == FW_FILTER_WR_SMT_TBL_FULL) {
			CH_ERR(adap, "filter %u setup failed due to full SMT\n",
			       idx);
			clear_filter(adap, f);
		} else if (ret == FW_FILTER_WR_FLT_ADDED) {
			f->smtidx = (be64_to_cpu(rpl->oldval) >> 24) & 0xff;
			f->pending = 0;  /* asynchronous setup completed */
			f->valid = 1;
		} else {
			/*
			 * Something went wrong.  Issue a warning about the
			 * problem and clear everything out.
			 */
			CH_ERR(adap, "filter %u setup failed with error %u\n",
			       idx, ret);
			clear_filter(adap, f);
		}
	}
}
#endif

/*
 * Response queue handler for the FW event queue.
 */
static int fwevtq_handler(struct sge_rspq *q, const __be64 *rsp,
			  const struct pkt_gl *gl)
{
	u8 opcode = ((const struct rss_header *)rsp)->opcode;

	rsp++;                                          /* skip RSS header */

	if (likely(opcode == CPL_SGE_EGR_UPDATE)) {
		const struct cpl_sge_egr_update *p = (void *)rsp;
		unsigned int qid = G_EGR_QID(ntohl(p->opcode_qid));
		struct sge_txq *txq;

		txq = q->adapter->sge.egr_map[qid - q->adapter->sge.egr_start];
		if ((u8 *)txq < (u8 *)q->adapter->sge.ofldtxq) {
			struct sge_eth_txq *eq;

			eq = container_of(txq, struct sge_eth_txq, q);
			t4_sge_coalesce_handler(q->adapter, eq);	
		} else {
			struct sge_ofld_txq *oq;

			txq->restarts++;
			oq = container_of(txq, struct sge_ofld_txq, q);
			tasklet_schedule(&oq->qresume_tsk);
		}
	} else if (opcode == CPL_FW6_MSG || opcode == CPL_FW4_MSG) {
		const struct cpl_fw6_msg *p = (void *)rsp;

		if (p->type == FW6_TYPE_CMD_RPL)
			t4_handle_fw_rpl(q->adapter, p->data);
	}
#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	else if (opcode == CPL_L2T_WRITE_RPL) {
		const struct cpl_l2t_write_rpl *p = (void *)rsp;

		do_l2t_write_rpl(q->adapter, p);
	} else if (opcode == CPL_SET_TCB_RPL) {
		const struct cpl_set_tcb_rpl *p = (void *)rsp;

		filter_rpl(q->adapter, p);
	}
#endif
	else
		CH_ERR(q->adapter, "unexpected CPL %#x on FW event queue\n",
		       opcode);
	return 0;
}

/**
 *	uldrx_handler - response queue handler for ULD queues
 *	@q: the response queue that received the packet
 *	@rsp: the response queue descriptor holding the offload message
 *	@gl: the gather list of packet fragments
 *
 *	Deliver an ingress offload packet to a ULD.  All processing is done by
 *	the ULD, we just maintain statistics.
 */
static int uldrx_handler(struct sge_rspq *q, const __be64 *rsp,
			 const struct pkt_gl *gl)
{
	struct sge_ofld_rxq *rxq = container_of(q, struct sge_ofld_rxq, rspq);

	if (ulds[q->uld].rx_handler(q->adapter->uld_handle[q->uld], rsp, gl)) {
		rxq->stats.nomem++;
		return -1;
	}
	if (gl == NULL)
		rxq->stats.imm++;
	else if (gl == CXGB4_MSG_AN)
		rxq->stats.an++;
	else
		rxq->stats.pkts++;
	return 0;
}

static void cxgb_disable_msi(struct adapter *adapter)
{
	if (adapter->flags & USING_MSIX) {
		pci_disable_msix(adapter->pdev);
		adapter->flags &= ~USING_MSIX;
	} else if (adapter->flags & USING_MSI) {
		pci_disable_msi(adapter->pdev);
		adapter->flags &= ~USING_MSI;
	}
}

/*
 * Interrupt handler for non-data events used with MSI-X.
 */
static irqreturn_t t4_nondata_intr(int irq, void *cookie)
{
	struct adapter *adap = cookie;

	u32 v = t4_read_reg(adap, MYPF_REG(A_PL_PF_INT_CAUSE));
	if (v & F_PFSW) {
		adap->swintr = 1;
		t4_write_reg(adap, MYPF_REG(A_PL_PF_INT_CAUSE), v);
	}
	t4_slow_intr_handler(adap);
	return IRQ_HANDLED;
}

/*
 * Name the MSI-X interrupts.
 */
static void name_msix_vecs(struct adapter *adap)
{
	int i, j, msi_idx = 2, n = sizeof(adap->msix_info[0].desc);

	/* non-data interrupts */
	snprintf(adap->msix_info[0].desc, n, "%s", adap->name);

	/* FW events */
	snprintf(adap->msix_info[1].desc, n, "%s-FWeventq", adap->name);

	/* Ethernet queues */
	for_each_port(adap, j) {
		struct net_device *d = adap->port[j];
		const struct port_info *pi = netdev_priv(d);

		for (i = 0; i < pi->nqsets; i++, msi_idx++)
			snprintf(adap->msix_info[msi_idx].desc, n,
				 "%s (queue %d)", d->name, i);
	}

	/* offload queues */
	for_each_ofldrxq(&adap->sge, i)
		snprintf(adap->msix_info[msi_idx++].desc, n, "%s-ofld%d",
			 adap->name, i);
	for_each_rdmarxq(&adap->sge, i)
		snprintf(adap->msix_info[msi_idx++].desc, n, "%s-rdma%d",
			 adap->name, i);
	for_each_iscsirxq(&adap->sge, i)
		snprintf(adap->msix_info[msi_idx++].desc, n, "%s-iSCSI%d",
			 adap->name, i);
}

static int request_msix_queue_irqs(struct adapter *adap)
{
	struct sge *s = &adap->sge;
	int err, ethqidx, msi = 2;
#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	int ofldqidx = 0, rdmaqidx = 0, iscsiqidx = 0;
#endif

	err = request_irq(adap->msix_info[1].vec, t4_sge_intr_msix, 0,
			  adap->msix_info[1].desc, &s->fw_evtq);
	if (err)
		return err;

	for_each_ethrxq(s, ethqidx) {
		err = request_irq(adap->msix_info[msi].vec, t4_sge_intr_msix, 0,
				  adap->msix_info[msi].desc,
				  &s->ethrxq[ethqidx].rspq);
		if (err)
			goto unwind;
		msi++;
	}
#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	for_each_ofldrxq(s, ofldqidx) {
		err = request_irq(adap->msix_info[msi].vec, t4_sge_intr_msix, 0,
				  adap->msix_info[msi].desc,
				  &s->ofldrxq[ofldqidx].rspq);
		if (err)
			goto unwind;
		msi++;
	}
	for_each_rdmarxq(s, rdmaqidx) {
		err = request_irq(adap->msix_info[msi].vec, t4_sge_intr_msix, 0,
				  adap->msix_info[msi].desc,
				  &s->rdmarxq[rdmaqidx].rspq);
		if (err)
			goto unwind;
		msi++;
	}
	for_each_iscsirxq(s, iscsiqidx) {
		err = request_irq(adap->msix_info[msi].vec, t4_sge_intr_msix, 0,
				  adap->msix_info[msi].desc,
				  &s->iscsirxq[iscsiqidx].rspq);
		if (err)
			goto unwind;
		msi++;
	}
#endif
	return 0;

unwind:
#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	while (--iscsiqidx >= 0)
		free_irq(adap->msix_info[--msi].vec,
			 &s->iscsirxq[iscsiqidx].rspq);
	while (--rdmaqidx >= 0)
		free_irq(adap->msix_info[--msi].vec,
			 &s->rdmarxq[rdmaqidx].rspq);
	while (--ofldqidx >= 0)
		free_irq(adap->msix_info[--msi].vec,
			 &s->ofldrxq[ofldqidx].rspq);
#endif
	while (--ethqidx >= 0)
		free_irq(adap->msix_info[--msi].vec, &s->ethrxq[ethqidx].rspq);
	free_irq(adap->msix_info[1].vec, &s->fw_evtq);
	return err;
}

static void free_msix_queue_irqs(struct adapter *adap)
{
	int i, msi = 2;
	struct sge *s = &adap->sge;

	free_irq(adap->msix_info[1].vec, &s->fw_evtq);
	for_each_ethrxq(s, i)
		free_irq(adap->msix_info[msi++].vec, &s->ethrxq[i].rspq);
#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	for_each_ofldrxq(s, i)
		free_irq(adap->msix_info[msi++].vec, &s->ofldrxq[i].rspq);
	for_each_rdmarxq(s, i)
		free_irq(adap->msix_info[msi++].vec, &s->rdmarxq[i].rspq);
	for_each_iscsirxq(s, i)
		free_irq(adap->msix_info[msi++].vec, &s->iscsirxq[i].rspq);
#endif
}

/**
 *	setup_rss - configure RSS
 *	@adapter: the adapter
 *
 *	Sets up RSS to distribute packets to multiple receive queues.  We
 *	configure the RSS CPU lookup table to distribute to the number of HW
 *	receive queues, and the response queue lookup table to narrow that
 *	down to the response queues actually configured for each port.
 *	We always configure the RSS mapping for all ports since the mapping
 *	table has plenty of entries.
 */
static int setup_rss(struct adapter *adapter)
{
	int pidx;

	for_each_port(adapter, pidx) {
		struct port_info *pi = adap2pinfo(adapter, pidx);
		struct sge_eth_rxq *rxq = &adapter->sge.ethrxq[pi->first_qset];
		u16 rss[MAX_INGQ];
		int qs, err;

		for (qs = 0; qs < pi->nqsets; qs++)
			rss[qs] = rxq[qs].rspq.abs_id;

		err = t4_config_rss_range(adapter, adapter->mbox, pi->viid,
					  0, pi->rss_size, rss, pi->nqsets);
		/*
		 * If Tunnel All Lookup isn't specified in the global RSS
		 * Configuration, then we need to specify a default Ingress
		 * Queue for any ingress packets which aren't hashed.  We'll
		 * use our first ingress queue ...
		 */
		if (!err)
			err = t4_config_vi_rss(adapter, adapter->mbox, pi->viid,
					F_FW_RSS_VI_CONFIG_CMD_IP6FOURTUPEN |
					F_FW_RSS_VI_CONFIG_CMD_IP6TWOTUPEN |
					F_FW_RSS_VI_CONFIG_CMD_IP4FOURTUPEN |
					F_FW_RSS_VI_CONFIG_CMD_IP4TWOTUPEN |
					F_FW_RSS_VI_CONFIG_CMD_UDPEN,
					rss[0]);
		if (err)
			return err;
	}
	return 0;
}

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
/*
 * Return the channel of the ingress queue with the given qid.
 */
static unsigned int rxq_to_chan(const struct sge *p, unsigned int qid)
{
	qid -= p->ingr_start;
	return netdev2pinfo(p->ingr_map[qid]->netdev)->tx_chan;
}
#endif /* CONFIG_CHELSIO_T4_OFFLOAD */

/*
 * Wait until all NAPI handlers are descheduled.
 */
static void quiesce_rx(struct adapter *adap)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adap->sge.ingr_map); i++) {
		struct sge_rspq *q = adap->sge.ingr_map[i];

		if (q && q->handler)
			napi_disable(&q->napi);
	}
}

/*
 * Enable NAPI scheduling and interrupt generation for all Rx queues.
 */
static void enable_rx(struct adapter *adap)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adap->sge.ingr_map); i++) {
		struct sge_rspq *q = adap->sge.ingr_map[i];

		if (!q)
			continue;
		if (q->handler)
			napi_enable(&q->napi);
		/* 0-increment GTS to start the timer and enable interrupts */
		t4_write_reg(adap, MYPF_REG(A_SGE_PF_GTS),
			     V_SEINTARM(q->intr_params) |
			     V_INGRESSQID(q->cntxt_id));
	}
}

static int alloc_ofld_rxqs(struct adapter *adap, struct sge_ofld_rxq *q,
			   unsigned int nq, unsigned int per_chan, int msi_idx,
			   u16 *ids)
{
	int i, err;

	for (i = 0; i < nq; i++, q++) {
		if (msi_idx > 0)
			msi_idx++;
		err = t4_sge_alloc_rxq(adap, &q->rspq, false,
				       adap->port[i / per_chan],
				       msi_idx, &q->fl, uldrx_handler, 0);
		if (err)
			return err;
		memset(&q->stats, 0, sizeof(q->stats));
		if (ids)
			ids[i] = q->rspq.abs_id;
	}
	return 0;
}

/**
 *	setup_sge_queues - configure SGE Tx/Rx/response queues
 *	@adap: the adapter
 *
 *	Determines how many sets of SGE queues to use and initializes them.
 *	We support multiple queue sets per port if we have MSI-X, otherwise
 *	just one queue set per port.
 */
static int setup_sge_queues(struct adapter *adap)
{
	int err, msi_idx, i, j;
	struct sge *s = &adap->sge;

	bitmap_zero(s->starving_fl, MAX_EGRQ);
	bitmap_zero(s->txq_maperr, MAX_EGRQ);

	if (adap->flags & USING_MSIX)
		msi_idx = 1;         /* vector 0 is for non-queue interrupts */
	else {
		err = t4_sge_alloc_rxq(adap, &s->intrq, false, adap->port[0], 0,
				       NULL, NULL, -1);
		if (err)
			return err;
		msi_idx = -((int)s->intrq.abs_id + 1);
	}

	err = t4_sge_alloc_rxq(adap, &s->fw_evtq, true, adap->port[0],
			       msi_idx, NULL, fwevtq_handler, -1);
	if (err) {
freeout:	t4_free_sge_resources(adap);
		return err;
	}

	for_each_port(adap, i) {
		struct net_device *dev = adap->port[i];
		struct port_info *pi = netdev_priv(dev);
		struct sge_eth_rxq *q = &s->ethrxq[pi->first_qset];
		struct sge_eth_txq *t = &s->ethtxq[pi->first_qset];

		for (j = 0; j < pi->nqsets; j++, q++) {
			if (msi_idx > 0)
				msi_idx++;
			err = t4_sge_alloc_rxq(adap, &q->rspq, false, dev,
					       msi_idx, &q->fl,
					       t4_ethrx_handler,
					       1 << pi->tx_chan);
			if (err)
				goto freeout;
			q->rspq.idx = j;
			memset(&q->stats, 0, sizeof(q->stats));
		}
		for (j = 0; j < pi->nqsets; j++, t++) {
			err = t4_sge_alloc_eth_txq(adap, t, dev,
					netdev_get_tx_queue(dev, j),
					s->fw_evtq.cntxt_id);
			if (err)
				goto freeout;
		}
	}

	j = s->ofldqsets / adap->params.nports; /* ofld queues per channel */
	for_each_ofldrxq(s, i) {
		err = t4_sge_alloc_ofld_txq(adap, &s->ofldtxq[i],
					    adap->port[i / j],
					    s->fw_evtq.cntxt_id);
		if (err)
			goto freeout;
	}

#define ALLOC_OFLD_RXQS(firstq, nq, per_chan, ids) do { \
	err = alloc_ofld_rxqs(adap, firstq, nq, per_chan, msi_idx, ids); \
	if (err) \
		goto freeout; \
	if (msi_idx > 0) \
		msi_idx += nq; \
} while (0)

	ALLOC_OFLD_RXQS(s->ofldrxq, s->ofldqsets, j, s->ofld_rxq);
	ALLOC_OFLD_RXQS(s->rdmarxq, s->rdmaqs, 1, s->rdma_rxq);
	ALLOC_OFLD_RXQS(s->iscsirxq, s->niscsiq, 1, s->iscsi_rxq);

#undef ALLOC_OFLD_RXQS

	for_each_port(adap, i) {
		/*
		 * Note that ->rdmarxq[i].rspq.cntxt_id below is 0 if we don't
		 * have RDMA queues, and that's the right value.
		 */
		err = t4_sge_alloc_ctrl_txq(adap, &s->ctrlq[i], adap->port[i],
					    s->fw_evtq.cntxt_id,
					    s->rdmarxq[i].rspq.cntxt_id);
		if (err)
			goto freeout;
	}

	t4_write_reg(adap, A_MPS_TRC_RSS_CONTROL,
		     V_RSSCONTROL(netdev2pinfo(adap->port[0])->tx_chan) |
		     V_QUEUENUMBER(s->ethrxq[0].rspq.abs_id));
	return 0;
}

static int setup_loopback(struct adapter *adap)
{
	int i, err;
	u8 mac0[] = { 0, 0, 0, 0, 0, 0 };

	for_each_port(adap, i) {
		err = t4_change_mac(adap, adap->mbox, adap2pinfo(adap, i)->viid,
				    -1, mac0, true, false);
		if (err)
			return err;
	}
	return 0;
}

/*
 * Returns 0 if new FW was successfully loaded, a positive errno if a load was
 * started but failed, and a negative errno if flash load couldn't start.
 */
static int upgrade_fw(struct adapter *adap)
{
	int ret;
	u32 vers;
	const struct fw_hdr *hdr;
	const struct firmware *fw;
	struct device *dev = adap->pdev_dev;

	ret = request_firmware(&fw, FW_FNAME, dev);
	if (ret < 0) {
		dev_err(dev, "unable to load firmware image " FW_FNAME
			", error %d\n", ret);
		return ret;
	}

	hdr = (const struct fw_hdr *)fw->data;
	vers = ntohl(hdr->fw_ver);
	if (G_FW_HDR_FW_VER_MAJOR(vers) != FW_VERSION_MAJOR) {
		ret = -EINVAL;              /* wrong major version, won't do */
		goto out;
	}

	/*
	 * If the flash FW is unusable or we found something newer, load it.
	 */
	if (G_FW_HDR_FW_VER_MAJOR(adap->params.fw_vers) != FW_VERSION_MAJOR ||
	    vers > adap->params.fw_vers) {
		ret = -t4_load_fw(adap, fw->data, fw->size);
		if (!ret)
			dev_info(dev, "firmware upgraded to " FW_FNAME "\n");
	}
out:	release_firmware(fw);
	return ret;
}

/*
 * Allocate a chunk of memory using kmalloc or, if that fails, vmalloc.
 * The allocated memory is cleared.
 */
void *t4_alloc_mem(size_t size)
{
	void *p = kmalloc(size, GFP_KERNEL);

	if (!p)
		p = vmalloc(size);
	if (p)
		memset(p, 0, size);
	return p;
}

/*
 * Free memory allocated through alloc_mem().
 */
void t4_free_mem(void *addr)
{
	if (is_vmalloc_addr(addr))
		vfree(addr);
	else
		kfree(addr);
}

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
/*
 * Send a Work Request to write the filter at a specified index.  We construct
 * a Firmware Filter Work Request to have the work done and put the indicated
 * filter into "pending" mode which will prevent any further actions against
 * it till we get a reply from the firmware on the completion status of the
 * request.
 */
static int set_filter_wr(struct adapter *adapter, int fidx)
{
	struct filter_entry *f = &adapter->tids.ftid_tab[fidx];
	struct sk_buff *skb;
	struct fw_filter_wr *fwr;
	unsigned int ftid;

	/*
	 * If the new filter requires loopback Destination MAC and/or VLAN
	 * rewriting then we need to allocate a Layer 2 Table (L2T) entry for
	 * the filter.
	 */
	if (f->fs.newdmac || f->fs.newvlan) {
		/* allocate L2T entry for new filter */
		f->l2t = t4_l2t_alloc_switching(adapter->l2t);
		if (f->l2t == NULL)
			return -EAGAIN;
		if (t4_l2t_set_switching(adapter, f->l2t, f->fs.vlan,
					 f->fs.eport, f->fs.dmac)) {
			cxgb4_l2t_release(f->l2t);
			f->l2t = NULL;
			return -ENOMEM;
		}
	}

	ftid = adapter->tids.ftid_base + fidx;

	skb = alloc_skb(sizeof *fwr, GFP_KERNEL | __GFP_NOFAIL);
	fwr = (struct fw_filter_wr *)__skb_put(skb, sizeof *fwr);
	memset(fwr, 0, sizeof *fwr);

	/*
	 * It would be nice to put most of the following in t4_hw.c but most
	 * of the work is translating the cxgbtool ch_filter_specification
	 * into the Work Request and the definition of that structure is
	 * currently in cxgbtool.h which isn't appropriate to pull into the
	 * common code.  We may eventually try to come up with a more neutral
	 * filter specification structure but for now it's easiest to simply
	 * put this fairly direct code in line ...
	 */
	fwr->op_pkd = htonl(V_FW_WR_OP(FW_FILTER_WR));
	fwr->len16_pkd = htonl(V_FW_WR_LEN16(sizeof *fwr/16));
	fwr->tid_to_iq =
		htonl(V_FW_FILTER_WR_TID(ftid) |
		      V_FW_FILTER_WR_RQTYPE(f->fs.type) |
		      V_FW_FILTER_WR_NOREPLY(0) |
		      V_FW_FILTER_WR_IQ(f->fs.iq));
	fwr->del_filter_to_l2tix =
		htonl(V_FW_FILTER_WR_RPTTID(f->fs.rpttid) |
		      V_FW_FILTER_WR_DROP(f->fs.action == FILTER_DROP) |
		      V_FW_FILTER_WR_DIRSTEER(f->fs.dirsteer) |
		      V_FW_FILTER_WR_MASKHASH(f->fs.maskhash) |
		      V_FW_FILTER_WR_DIRSTEERHASH(f->fs.dirsteerhash) |
		      V_FW_FILTER_WR_LPBK(f->fs.action == FILTER_SWITCH) |
		      V_FW_FILTER_WR_DMAC(f->fs.newdmac) |
		      V_FW_FILTER_WR_SMAC(f->fs.newsmac) |
		      V_FW_FILTER_WR_INSVLAN(f->fs.newvlan == VLAN_INSERT ||
					     f->fs.newvlan == VLAN_REWRITE) |
		      V_FW_FILTER_WR_RMVLAN(f->fs.newvlan == VLAN_REMOVE ||
					    f->fs.newvlan == VLAN_REWRITE) |
		      V_FW_FILTER_WR_HITCNTS(f->fs.hitcnts) |
		      V_FW_FILTER_WR_TXCHAN(f->fs.eport) |
		      V_FW_FILTER_WR_PRIO(f->fs.prio) |
		      V_FW_FILTER_WR_L2TIX(f->l2t ? f->l2t->idx : 0));
	fwr->ethtype = htons(f->fs.val.ethtype);
	fwr->ethtypem = htons(f->fs.mask.ethtype);
	fwr->frag_to_ovlan_vldm =
		     (V_FW_FILTER_WR_FRAG(f->fs.val.frag) |
		      V_FW_FILTER_WR_FRAGM(f->fs.mask.frag) |
		      V_FW_FILTER_WR_IVLAN_VLD(f->fs.val.ivlan_vld) |
		      V_FW_FILTER_WR_OVLAN_VLD(f->fs.val.ovlan_vld) |
		      V_FW_FILTER_WR_IVLAN_VLDM(f->fs.mask.ivlan_vld) |
		      V_FW_FILTER_WR_OVLAN_VLDM(f->fs.mask.ovlan_vld));
	fwr->smac_sel = 0;
	fwr->rx_chan_rx_rpl_iq =
		htons(V_FW_FILTER_WR_RX_CHAN(0) |
		      V_FW_FILTER_WR_RX_RPL_IQ(adapter->sge.fw_evtq.abs_id));
	fwr->maci_to_matchtypem =
		htonl(V_FW_FILTER_WR_MACI(f->fs.val.macidx) |
		      V_FW_FILTER_WR_MACIM(f->fs.mask.macidx) |
		      V_FW_FILTER_WR_FCOE(f->fs.val.fcoe) |
		      V_FW_FILTER_WR_FCOEM(f->fs.mask.fcoe) |
		      V_FW_FILTER_WR_PORT(f->fs.val.iport) |
		      V_FW_FILTER_WR_PORTM(f->fs.mask.iport) |
		      V_FW_FILTER_WR_MATCHTYPE(f->fs.val.matchtype) |
		      V_FW_FILTER_WR_MATCHTYPEM(f->fs.mask.matchtype));
	fwr->ptcl = f->fs.val.proto;
	fwr->ptclm = f->fs.mask.proto;
	fwr->ttyp = f->fs.val.tos;
	fwr->ttypm = f->fs.mask.tos;
	fwr->ivlan = htons(f->fs.val.ivlan);
	fwr->ivlanm = htons(f->fs.mask.ivlan);
	fwr->ovlan = htons(f->fs.val.ovlan);
	fwr->ovlanm = htons(f->fs.mask.ovlan);
	memcpy(fwr->lip, f->fs.val.lip, sizeof fwr->lip);
	memcpy(fwr->lipm, f->fs.mask.lip, sizeof fwr->lipm);
	memcpy(fwr->fip, f->fs.val.fip, sizeof fwr->fip);
	memcpy(fwr->fipm, f->fs.mask.fip, sizeof fwr->fipm);
	fwr->lp = htons(f->fs.val.lport);
	fwr->lpm = htons(f->fs.mask.lport);
	fwr->fp = htons(f->fs.val.fport);
	fwr->fpm = htons(f->fs.mask.fport);
	if (f->fs.newsmac)
		memcpy(fwr->sma, f->fs.smac, sizeof fwr->sma);

	/*
	 * Mark the filter as "pending" and ship off the Filter Work Request.
	 * When we get the Work Request Reply we'll clear the pending status.
	 */
	f->pending = 1;
	set_wr_txq(skb, CPL_PRIORITY_CONTROL, 0);
	t4_ofld_send(adapter, skb);
	return 0;
}

/*
 * Delete the filter at a specified index.
 */
static int del_filter_wr(struct adapter *adapter, int fidx)
{
	struct filter_entry *f = &adapter->tids.ftid_tab[fidx];
	struct sk_buff *skb;
	struct fw_filter_wr *fwr;
	unsigned int len, ftid;

	len = sizeof *fwr;
	ftid = adapter->tids.ftid_base + fidx;

	skb = alloc_skb(len, GFP_KERNEL | __GFP_NOFAIL);
	fwr = (struct fw_filter_wr *)__skb_put(skb, len);
	t4_mk_filtdelwr(ftid, fwr, adapter->sge.fw_evtq.abs_id);

	/*
	 * Mark the filter as "pending" and ship off the Filter Work Request.
	 * When we get the Work Request Reply we'll clear the pending status.
	 */
	f->pending = 1;
	t4_mgmt_tx(adapter, skb);
	return 0;
}
#endif

static struct net_device_stats *cxgb_get_stats(struct net_device *dev)
{
	struct port_stats stats;
	struct port_info *p = netdev_priv(dev);
	struct adapter *adapter = p->adapter;
	struct net_device_stats *ns = &dev->stats;

	spin_lock(&adapter->stats_lock);
	t4_get_port_stats(adapter, p->tx_chan, &stats);
	spin_unlock(&adapter->stats_lock);

	ns->tx_bytes   = stats.tx_octets;
	ns->tx_packets = stats.tx_frames;
	ns->rx_bytes   = stats.rx_octets;
	ns->rx_packets = stats.rx_frames;
	ns->multicast  = stats.rx_mcast_frames;

	/* detailed rx_errors */
	ns->rx_length_errors = stats.rx_jabber + stats.rx_too_long +
			       stats.rx_runt;
	ns->rx_over_errors   = 0;
	ns->rx_crc_errors    = stats.rx_fcs_err;
	ns->rx_frame_errors  = stats.rx_symbol_err;
	ns->rx_fifo_errors   = stats.rx_ovflow0 + stats.rx_ovflow1 +
			       stats.rx_ovflow2 + stats.rx_ovflow3 +
			       stats.rx_trunc0 + stats.rx_trunc1 +
			       stats.rx_trunc2 + stats.rx_trunc3;
	ns->rx_missed_errors = 0;

	/* detailed tx_errors */
	ns->tx_aborted_errors   = 0;
	ns->tx_carrier_errors   = 0;
	ns->tx_fifo_errors      = 0;
	ns->tx_heartbeat_errors = 0;
	ns->tx_window_errors    = 0;

	ns->tx_errors = stats.tx_error_frames;
	ns->rx_errors = stats.rx_symbol_err + stats.rx_fcs_err +
		ns->rx_length_errors + stats.rx_len_err + ns->rx_fifo_errors;
	return ns;
}

/*
 * Implementation of ethtool operations.
 */

static u32 get_msglevel(struct net_device *dev)
{
	return netdev2adap(dev)->msg_enable;
}

static void set_msglevel(struct net_device *dev, u32 val)
{
	netdev2adap(dev)->msg_enable = val;
}

static char stats_strings[][ETH_GSTRING_LEN] = {
	"TxOctetsOK         ",
	"TxFramesOK         ",
	"TxBroadcastFrames  ",
	"TxMulticastFrames  ",
	"TxUnicastFrames    ",
	"TxErrorFrames      ",

	"TxFrames64         ",
	"TxFrames65To127    ",
	"TxFrames128To255   ",
	"TxFrames256To511   ",
	"TxFrames512To1023  ",
	"TxFrames1024To1518 ",
	"TxFrames1519ToMax  ",

	"TxFramesDropped    ",
	"TxPauseFrames      ",
	"TxPPP0Frames       ",
	"TxPPP1Frames       ",
	"TxPPP2Frames       ",
	"TxPPP3Frames       ",
	"TxPPP4Frames       ",
	"TxPPP5Frames       ",
	"TxPPP6Frames       ",
	"TxPPP7Frames       ",

	"RxOctetsOK         ",
	"RxFramesOK         ",
	"RxBroadcastFrames  ",
	"RxMulticastFrames  ",
	"RxUnicastFrames    ",

	"RxFramesTooLong    ",
	"RxJabberErrors     ",
	"RxFCSErrors        ",
	"RxLengthErrors     ",
	"RxSymbolErrors     ",
	"RxRuntFrames       ",

	"RxFrames64         ",
	"RxFrames65To127    ",
	"RxFrames128To255   ",
	"RxFrames256To511   ",
	"RxFrames512To1023  ",
	"RxFrames1024To1518 ",
	"RxFrames1519ToMax  ",

	"RxPauseFrames      ",
	"RxPPP0Frames       ",
	"RxPPP1Frames       ",
	"RxPPP2Frames       ",
	"RxPPP3Frames       ",
	"RxPPP4Frames       ",
	"RxPPP5Frames       ",
	"RxPPP6Frames       ",
	"RxPPP7Frames       ",

	"RxBG0FramesDropped ",
	"RxBG1FramesDropped ",
	"RxBG2FramesDropped ",
	"RxBG3FramesDropped ",
	"RxBG0FramesTrunc   ",
	"RxBG1FramesTrunc   ",
	"RxBG2FramesTrunc   ",
	"RxBG3FramesTrunc   ",

	"TSO                ",
	"TxCsumOffload      ",
	"RxCsumGood         ",
	"VLANextractions    ",
	"VLANinsertions     ",
	"GROPackets         ",
	"GROMerged          ",
};

static int get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(stats_strings);
	default:
		return -EOPNOTSUPP;
	}
}

#define T4_REGMAP_SIZE (160 * 1024)

static int get_regs_len(struct net_device *dev)
{
	return T4_REGMAP_SIZE;
}

static int get_eeprom_len(struct net_device *dev)
{
	return EEPROMSIZE;
}

static void get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	struct adapter *adapter = netdev2adap(dev);

	strcpy(info->driver, KBUILD_MODNAME);
	strcpy(info->version, DRV_VERSION);
	strcpy(info->bus_info, pci_name(adapter->pdev));

	if (!adapter->params.fw_vers)
		strcpy(info->fw_version, "N/A");
	else
		snprintf(info->fw_version, sizeof(info->fw_version),
			"%u.%u.%u.%u, TP %u.%u.%u.%u",
			G_FW_HDR_FW_VER_MAJOR(adapter->params.fw_vers),
			G_FW_HDR_FW_VER_MINOR(adapter->params.fw_vers),
			G_FW_HDR_FW_VER_MICRO(adapter->params.fw_vers),
			G_FW_HDR_FW_VER_BUILD(adapter->params.fw_vers),
			G_FW_HDR_FW_VER_MAJOR(adapter->params.tp_vers),
			G_FW_HDR_FW_VER_MINOR(adapter->params.tp_vers),
			G_FW_HDR_FW_VER_MICRO(adapter->params.tp_vers),
			G_FW_HDR_FW_VER_BUILD(adapter->params.tp_vers));
}

static void get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	if (stringset == ETH_SS_STATS)
		memcpy(data, stats_strings, sizeof(stats_strings));
}

/*
 * port stats maintained per queue of the port.  They should be in the same
 * order as in stats_strings above.
 */
struct queue_port_stats {
	u64 tso;
	u64 tx_csum;
	u64 rx_csum;
	u64 vlan_ex;
	u64 vlan_ins;
	u64 lro_pkts;
	u64 lro_merged;
};

static void collect_sge_port_stats(const struct adapter *adap,
		const struct port_info *p, struct queue_port_stats *s)
{
	int i;
	const struct sge_eth_txq *tx = &adap->sge.ethtxq[p->first_qset];
	const struct sge_eth_rxq *rx = &adap->sge.ethrxq[p->first_qset];

	memset(s, 0, sizeof(*s));
	for (i = 0; i < p->nqsets; i++, rx++, tx++) {
		s->tso += tx->tso;
		s->tx_csum += tx->tx_cso;
		s->rx_csum += rx->stats.rx_cso;
		s->vlan_ex += rx->stats.vlan_ex;
		s->vlan_ins += tx->vlan_ins;
		s->lro_pkts += rx->stats.lro_pkts;
		s->lro_merged += rx->stats.lro_merged;
	}
}

/* clear port-related stats maintained by the port's associated queues */
static void clear_sge_port_stats(struct adapter *adap, struct port_info *p)
{
	int i;
	struct sge_eth_txq *tx = &adap->sge.ethtxq[p->first_qset];
	struct sge_eth_rxq *rx = &adap->sge.ethrxq[p->first_qset];

	for (i = 0; i < p->nqsets; i++, rx++, tx++) {
		memset(&rx->stats, 0, sizeof(rx->stats));
		tx->tso = 0;
		tx->tx_cso = 0;
		tx->vlan_ins = 0;
		tx->coal_wr = 0;
		tx->coal_pkts = 0;
		rx->stats.lro_pkts = 0;
		rx->stats.lro_merged = 0;
	}
}

static void get_stats(struct net_device *dev, struct ethtool_stats *stats,
		      u64 *data)
{
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	t4_get_port_stats(adapter, pi->tx_chan, (struct port_stats *)data);

	data += sizeof(struct port_stats) / sizeof(u64);
	collect_sge_port_stats(adapter, pi, (struct queue_port_stats *)data);
}

/*
 * Return a version number to identify the type of adapter.  The scheme is:
 * - bits 0..9: chip version
 * - bits 10..15: chip revision
 * - bits 16..23: register dump version
 */
static inline unsigned int mk_adap_vers(const struct adapter *ap)
{
	return 4 | (ap->params.rev << 10) | (1 << 16);
}

static void reg_block_dump(struct adapter *ap, void *buf, unsigned int start,
			   unsigned int end)
{
	u32 *p = buf + start;

	for ( ; start <= end; start += sizeof(u32))
		*p++ = t4_read_reg(ap, start);
}

static void get_regs(struct net_device *dev, struct ethtool_regs *regs,
		     void *buf)
{
	static const unsigned int reg_ranges[] = {
		0x1008, 0x1108,
		0x1180, 0x11b4,
		0x11fc, 0x123c,
		0x1300, 0x173c,
		0x1800, 0x18fc,
		0x3000, 0x30d8,
		0x30e0, 0x5924,
		0x5960, 0x59d4,
		0x5a00, 0x5af8,
		0x6000, 0x6098,
		0x6100, 0x6150,
		0x6200, 0x6208,
		0x6240, 0x6248,
		0x6280, 0x6338,
		0x6370, 0x638c,
		0x6400, 0x643c,
		0x6500, 0x6524,
		0x6a00, 0x6a38,
		0x6a60, 0x6a78,
		0x6b00, 0x6b84,
		0x6bf0, 0x6c84,
		0x6cf0, 0x6d84,
		0x6df0, 0x6e84,
		0x6ef0, 0x6f84,
		0x6ff0, 0x7084,
		0x70f0, 0x7184,
		0x71f0, 0x7284,
		0x72f0, 0x7384,
		0x73f0, 0x7450,
		0x7500, 0x7530,
		0x7600, 0x761c,
		0x7680, 0x76cc,
		0x7700, 0x7798,
		0x77c0, 0x77fc,
		0x7900, 0x79fc,
		0x7b00, 0x7c38,
		0x7d00, 0x7efc,
		0x8dc0, 0x8e1c,
		0x8e30, 0x8e78,
		0x8ea0, 0x8f6c,
		0x8fc0, 0x9074,
		0x90fc, 0x90fc,
		0x9400, 0x9458,
		0x9600, 0x96bc,
		0x9800, 0x9808,
		0x9820, 0x983c,
		0x9850, 0x9864,
		0x9c00, 0x9c6c,
		0x9c80, 0x9cec,
		0x9d00, 0x9d6c,
		0x9d80, 0x9dec,
		0x9e00, 0x9e6c,
		0x9e80, 0x9eec,
		0x9f00, 0x9f6c,
		0x9f80, 0x9fec,
		0xd004, 0xd03c,
		0xdfc0, 0xdfe0,
		0xe000, 0xea7c,
		0xf000, 0x11190,
		0x19040, 0x1906c,
		0x19078, 0x19080,
		0x1908c, 0x19124,
		0x19150, 0x191b0,
		0x191d0, 0x191e8,
		0x19238, 0x1924c,
		0x193f8, 0x19474,
		0x19490, 0x194f8,
		0x19800, 0x19f30,
		0x1a000, 0x1a06c,
		0x1a0b0, 0x1a120,
		0x1a128, 0x1a138,
		0x1a190, 0x1a1c4,
		0x1a1fc, 0x1a1fc,
		0x1e040, 0x1e04c,
		0x1e284, 0x1e28c,
		0x1e2c0, 0x1e2c0,
		0x1e2e0, 0x1e2e0,
		0x1e300, 0x1e384,
		0x1e3c0, 0x1e3c8,
		0x1e440, 0x1e44c,
		0x1e684, 0x1e68c,
		0x1e6c0, 0x1e6c0,
		0x1e6e0, 0x1e6e0,
		0x1e700, 0x1e784,
		0x1e7c0, 0x1e7c8,
		0x1e840, 0x1e84c,
		0x1ea84, 0x1ea8c,
		0x1eac0, 0x1eac0,
		0x1eae0, 0x1eae0,
		0x1eb00, 0x1eb84,
		0x1ebc0, 0x1ebc8,
		0x1ec40, 0x1ec4c,
		0x1ee84, 0x1ee8c,
		0x1eec0, 0x1eec0,
		0x1eee0, 0x1eee0,
		0x1ef00, 0x1ef84,
		0x1efc0, 0x1efc8,
		0x1f040, 0x1f04c,
		0x1f284, 0x1f28c,
		0x1f2c0, 0x1f2c0,
		0x1f2e0, 0x1f2e0,
		0x1f300, 0x1f384,
		0x1f3c0, 0x1f3c8,
		0x1f440, 0x1f44c,
		0x1f684, 0x1f68c,
		0x1f6c0, 0x1f6c0,
		0x1f6e0, 0x1f6e0,
		0x1f700, 0x1f784,
		0x1f7c0, 0x1f7c8,
		0x1f840, 0x1f84c,
		0x1fa84, 0x1fa8c,
		0x1fac0, 0x1fac0,
		0x1fae0, 0x1fae0,
		0x1fb00, 0x1fb84,
		0x1fbc0, 0x1fbc8,
		0x1fc40, 0x1fc4c,
		0x1fe84, 0x1fe8c,
		0x1fec0, 0x1fec0,
		0x1fee0, 0x1fee0,
		0x1ff00, 0x1ff84,
		0x1ffc0, 0x1ffc8,
		0x20000, 0x2002c,
		0x20100, 0x2013c,
		0x20190, 0x201c8,
		0x20200, 0x20318,
		0x20400, 0x20528,
		0x20540, 0x20614,
		0x21000, 0x21040,
		0x2104c, 0x21060,
		0x210c0, 0x210ec,
		0x21200, 0x21268,
		0x21270, 0x21284,
		0x212fc, 0x21388,
		0x21400, 0x21404,
		0x21500, 0x21518,
		0x2152c, 0x2153c,
		0x21550, 0x21554,
		0x21600, 0x21600,
		0x21608, 0x21628,
		0x21630, 0x2163c,
		0x21700, 0x2171c,
		0x21780, 0x2178c,
		0x21800, 0x21c38,
		0x21c80, 0x21d7c,
		0x21e00, 0x21e04,
		0x22000, 0x2202c,
		0x22100, 0x2213c,
		0x22190, 0x221c8,
		0x22200, 0x22318,
		0x22400, 0x22528,
		0x22540, 0x22614,
		0x23000, 0x23040,
		0x2304c, 0x23060,
		0x230c0, 0x230ec,
		0x23200, 0x23268,
		0x23270, 0x23284,
		0x232fc, 0x23388,
		0x23400, 0x23404,
		0x23500, 0x23518,
		0x2352c, 0x2353c,
		0x23550, 0x23554,
		0x23600, 0x23600,
		0x23608, 0x23628,
		0x23630, 0x2363c,
		0x23700, 0x2371c,
		0x23780, 0x2378c,
		0x23800, 0x23c38,
		0x23c80, 0x23d7c,
		0x23e00, 0x23e04,
		0x24000, 0x2402c,
		0x24100, 0x2413c,
		0x24190, 0x241c8,
		0x24200, 0x24318,
		0x24400, 0x24528,
		0x24540, 0x24614,
		0x25000, 0x25040,
		0x2504c, 0x25060,
		0x250c0, 0x250ec,
		0x25200, 0x25268,
		0x25270, 0x25284,
		0x252fc, 0x25388,
		0x25400, 0x25404,
		0x25500, 0x25518,
		0x2552c, 0x2553c,
		0x25550, 0x25554,
		0x25600, 0x25600,
		0x25608, 0x25628,
		0x25630, 0x2563c,
		0x25700, 0x2571c,
		0x25780, 0x2578c,
		0x25800, 0x25c38,
		0x25c80, 0x25d7c,
		0x25e00, 0x25e04,
		0x26000, 0x2602c,
		0x26100, 0x2613c,
		0x26190, 0x261c8,
		0x26200, 0x26318,
		0x26400, 0x26528,
		0x26540, 0x26614,
		0x27000, 0x27040,
		0x2704c, 0x27060,
		0x270c0, 0x270ec,
		0x27200, 0x27268,
		0x27270, 0x27284,
		0x272fc, 0x27388,
		0x27400, 0x27404,
		0x27500, 0x27518,
		0x2752c, 0x2753c,
		0x27550, 0x27554,
		0x27600, 0x27600,
		0x27608, 0x27628,
		0x27630, 0x2763c,
		0x27700, 0x2771c,
		0x27780, 0x2778c,
		0x27800, 0x27c38,
		0x27c80, 0x27d7c,
		0x27e00, 0x27e04
	};

	int i;
	struct adapter *ap = netdev2adap(dev);

	regs->version = mk_adap_vers(ap);

	memset(buf, 0, T4_REGMAP_SIZE);
	for (i = 0; i < ARRAY_SIZE(reg_ranges); i += 2)
		reg_block_dump(ap, buf, reg_ranges[i], reg_ranges[i + 1]);
}

static int restart_autoneg(struct net_device *dev)
{
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	if (!netif_running(dev))
		return -EAGAIN;
	if (pi->link_cfg.autoneg != AUTONEG_ENABLE)
		return -EINVAL;
	t4_restart_aneg(adapter, adapter->mbox, pi->tx_chan);
	return 0;
}

static int identify_port(struct net_device *dev, u32 data)
{
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	if (data == 0)
		data = 2;     /* default to 2 seconds */

	return t4_identify_port(adapter, adapter->mbox, pi->viid, data * 5);
}

static unsigned int from_fw_linkcaps(unsigned int caps)
{
	unsigned int v = 0;

	if (caps & FW_PORT_CAP_SPEED_100M)
		v |= SUPPORTED_100baseT_Full;
	if (caps & FW_PORT_CAP_SPEED_1G)
		v |= SUPPORTED_1000baseT_Full;
	if (caps & FW_PORT_CAP_SPEED_10G)
		v |= SUPPORTED_10000baseT_Full;
	if (caps & FW_PORT_CAP_ANEG)
		v |= SUPPORTED_Autoneg;
	return v;
}

static unsigned int to_fw_linkcaps(unsigned int caps)
{
	unsigned int v = 0;

	if (caps & ADVERTISED_100baseT_Full)
		v |= FW_PORT_CAP_SPEED_100M;
	if (caps & ADVERTISED_1000baseT_Full)
		v |= FW_PORT_CAP_SPEED_1G;
	if (caps & ADVERTISED_10000baseT_Full)
		v |= FW_PORT_CAP_SPEED_10G;
	return v;
}

static int get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	const struct port_info *p = netdev_priv(dev);

	cmd->supported = from_fw_linkcaps(p->link_cfg.supported);
	cmd->advertising = from_fw_linkcaps(p->link_cfg.advertising);
	cmd->speed = netif_carrier_ok(dev) ? p->link_cfg.speed : -1;
	cmd->duplex = DUPLEX_FULL;

	if (p->port_type == FW_PORT_TYPE_BT_SGMII ||
	    p->port_type == FW_PORT_TYPE_BT_XAUI)
		cmd->port = PORT_TP;
	else if (p->port_type == FW_PORT_TYPE_FIBER_XFI ||
		 p->port_type == FW_PORT_TYPE_FIBER_XAUI)
		cmd->port = PORT_FIBRE;
	else
		cmd->port = PORT_FIBRE;

	cmd->phy_address = p->mdio_addr < 32 ? p->mdio_addr : 0;
	cmd->transceiver = p->mdio_addr < 32 ? XCVR_EXTERNAL : XCVR_INTERNAL;
	cmd->autoneg = p->link_cfg.autoneg;
	cmd->maxtxpkt = 0;
	cmd->maxrxpkt = 0;
	return 0;
}

static unsigned int speed_to_caps(int speed)
{
	if (speed == SPEED_100)
		return FW_PORT_CAP_SPEED_100M;
	if (speed == SPEED_1000)
		return FW_PORT_CAP_SPEED_1G;
	if (speed == SPEED_10000)
		return FW_PORT_CAP_SPEED_10G;
	return 0;
}

static int set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	unsigned int cap;
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;
	struct link_config *lc = &pi->link_cfg;

	if (cmd->duplex != DUPLEX_FULL)     /* only full-duplex supported */
		return -EINVAL;

	if (!(lc->supported & FW_PORT_CAP_ANEG)) {
		/*
		 * PHY offers a single speed.  See if that's what's
		 * being requested.
		 */
		if (cmd->autoneg == AUTONEG_DISABLE &&
		    (lc->supported & speed_to_caps(cmd->speed)))
				return 0;
		return -EINVAL;
	}

	if (cmd->autoneg == AUTONEG_DISABLE) {
		cap = speed_to_caps(cmd->speed);

		if (!(lc->supported & cap) || cmd->speed == SPEED_1000 ||
		    cmd->speed == SPEED_10000)
			return -EINVAL;
		lc->requested_speed = cap;
		lc->advertising = 0;
	} else {
		cap = to_fw_linkcaps(cmd->advertising);
		if (!(lc->supported & cap))
			return -EINVAL;
		lc->requested_speed = 0;
		lc->advertising = cap | FW_PORT_CAP_ANEG;
	}
	lc->autoneg = cmd->autoneg;

	if (netif_running(dev))
		return t4_link_start(adapter, adapter->mbox, pi->tx_chan, lc);
	return 0;
}

static void get_pauseparam(struct net_device *dev,
			   struct ethtool_pauseparam *epause)
{
	struct port_info *p = netdev_priv(dev);

	epause->autoneg = (p->link_cfg.requested_fc & PAUSE_AUTONEG) != 0;
	epause->rx_pause = (p->link_cfg.fc & PAUSE_RX) != 0;
	epause->tx_pause = (p->link_cfg.fc & PAUSE_TX) != 0;
}

static int set_pauseparam(struct net_device *dev,
			  struct ethtool_pauseparam *epause)
{
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;
	struct link_config *lc = &pi->link_cfg;

	if (epause->autoneg == AUTONEG_DISABLE)
		lc->requested_fc = 0;
	else if (lc->supported & FW_PORT_CAP_ANEG)
		lc->requested_fc = PAUSE_AUTONEG;
	else
		return -EINVAL;

	if (epause->rx_pause)
		lc->requested_fc |= PAUSE_RX;
	if (epause->tx_pause)
		lc->requested_fc |= PAUSE_TX;
	if (netif_running(dev))
		return t4_link_start(adapter, adapter->mbox, pi->tx_chan, lc);
	return 0;
}

static u32 get_rx_csum(struct net_device *dev)
{
	struct port_info *p = netdev_priv(dev);

	return p->rx_offload & RX_CSO;
}

static int set_rx_csum(struct net_device *dev, u32 data)
{
	struct port_info *p = netdev_priv(dev);

	if (data)
		p->rx_offload |= RX_CSO;
	else
		p->rx_offload &= ~RX_CSO;
	return 0;
}

static void get_sge_param(struct net_device *dev, struct ethtool_ringparam *e)
{
	const struct port_info *pi = netdev_priv(dev);
	const struct sge *s = &pi->adapter->sge;

	e->rx_max_pending = MAX_RX_BUFFERS;
	e->rx_mini_max_pending = MAX_RSPQ_ENTRIES;
	e->rx_jumbo_max_pending = 0;
	e->tx_max_pending = MAX_TXQ_ENTRIES;

	e->rx_pending = s->ethrxq[pi->first_qset].fl.size - 8;
	e->rx_mini_pending = s->ethrxq[pi->first_qset].rspq.size;
	e->rx_jumbo_pending = 0;
	e->tx_pending = s->ethtxq[pi->first_qset].q.size;
}

static int set_sge_param(struct net_device *dev, struct ethtool_ringparam *e)
{
	int i;
	const struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;
	struct sge *s = &adapter->sge;

	if (e->rx_pending > MAX_RX_BUFFERS || e->rx_jumbo_pending ||
	    e->tx_pending > MAX_TXQ_ENTRIES ||
	    e->rx_mini_pending > MAX_RSPQ_ENTRIES ||
	    e->rx_mini_pending < MIN_RSPQ_ENTRIES ||
	    e->rx_pending < MIN_FL_ENTRIES || e->tx_pending < MIN_TXQ_ENTRIES)
		return -EINVAL;

	if (adapter->flags & FULL_INIT_DONE)
		return -EBUSY;

	for (i = 0; i < pi->nqsets; ++i) {
		s->ethtxq[pi->first_qset + i].q.size = e->tx_pending;
		s->ethrxq[pi->first_qset + i].fl.size = e->rx_pending + 8;
		s->ethrxq[pi->first_qset + i].rspq.size = e->rx_mini_pending;
	}
	return 0;
}

static int closest_timer(const struct sge *s, int time)
{
	int i, delta, match = 0, min_delta = INT_MAX;

	for (i = 0; i < ARRAY_SIZE(s->timer_val); i++) {
		delta = time - s->timer_val[i];
		if (delta < 0)
			delta = -delta;
		if (delta < min_delta) {
			min_delta = delta;
			match = i;
		}
	}
	return match;
}

static int closest_thres(const struct sge *s, int thres)
{
	int i, delta, match = 0, min_delta = INT_MAX;

	for (i = 0; i < ARRAY_SIZE(s->counter_val); i++) {
		delta = thres - s->counter_val[i];
		if (delta < 0)
			delta = -delta;
		if (delta < min_delta) {
			min_delta = delta;
			match = i;
		}
	}
	return match;
}

/*
 * Return a queue's interrupt hold-off time in us.  0 means no timer.
 */
static unsigned int qtimer_val(const struct adapter *adap,
			       const struct sge_rspq *q)
{
	unsigned int idx = G_QINTR_TIMER_IDX(q->intr_params);

	return idx < SGE_NTIMERS ? adap->sge.timer_val[idx] : 0;
}

/**
 *	set_rspq_intr_params - set a queue's interrupt holdoff parameters
 *	@q: the Rx queue
 *	@us: the hold-off time in us, or 0 to disable timer
 *	@cnt: the hold-off packet count, or 0 to disable counter
 *
 *	Sets an Rx queue's interrupt hold-off time and packet count.  At least
 *	one of the two needs to be enabled for the queue to generate interrupts.
 */
static int set_rspq_intr_params(struct sge_rspq *q,
				unsigned int us, unsigned int cnt)
{
	struct adapter *adap = q->adapter;

	if ((us | cnt) == 0)
		cnt = 1;

	if (cnt) {
		int err;
		u32 v, new_idx;

		new_idx = closest_thres(&adap->sge, cnt);
		if (q->desc && q->pktcnt_idx != new_idx) {
			/* the queue has already been created, update it */
			v = V_FW_PARAMS_MNEM(FW_PARAMS_MNEM_DMAQ) |
			    V_FW_PARAMS_PARAM_X(FW_PARAMS_PARAM_DMAQ_IQ_INTCNTTHRESH) |
			    V_FW_PARAMS_PARAM_YZ(q->cntxt_id);
			err = t4_set_params(adap, adap->mbox, adap->pf, 0, 1,
					    &v, &new_idx);
			if (err)
				return err;
		}
		q->pktcnt_idx = new_idx;
	}

	us = us == 0 ? X_TIMERREG_RESTART_COUNTER : closest_timer(&adap->sge, us);
	q->intr_params = V_QINTR_TIMER_IDX(us) | V_QINTR_CNT_EN(cnt > 0);
	return 0;
}


/**
 *	set_rx_intr_params - set a net devices's RX interrupt holdoff paramete!
 *	@dev: the network device
 *	@us: the hold-off time in us, or 0 to disable timer
 *	@cnt: the hold-off packet count, or 0 to disable counter
 *
 *	Set the RX interrupt hold-off parameters for a network device.
 */
static int set_rx_intr_params(struct net_device *dev,
                              unsigned int us, unsigned int cnt)
{
	int i, err;
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adap = pi->adapter;
	struct sge_eth_rxq *q = &adap->sge.ethrxq[pi->first_qset];

	for (i = 0; i < pi->nqsets; i++, q++) {
		err = set_rspq_intr_params(&q->rspq, us, cnt);
		if (err)
			return err;
	}
	return 0;
}

static int set_coalesce(struct net_device *dev,
			struct ethtool_coalesce *coalesce)
{
	return set_rx_intr_params(dev, coalesce->rx_coalesce_usecs,
				  coalesce->rx_max_coalesced_frames);
}

static int get_coalesce(struct net_device *dev, struct ethtool_coalesce *c)
{
	const struct port_info *pi = netdev_priv(dev);
	const struct adapter *adap = pi->adapter;
	const struct sge_rspq *rq = &adap->sge.ethrxq[pi->first_qset].rspq;

	c->rx_coalesce_usecs = qtimer_val(adap, rq);
	c->rx_max_coalesced_frames = (rq->intr_params & F_QINTR_CNT_EN) ?
		adap->sge.counter_val[rq->pktcnt_idx] : 0;
	return 0;
}

/*
 * The next two routines implement eeprom read/write from physical addresses.
 */
static int eeprom_rd_phys(struct adapter *adap, unsigned int phys_addr, u32 *v)
{
	int vaddr = t4_eeprom_ptov(phys_addr, adap->pf, EEPROMPFSIZE);

	return vaddr < 0 ? vaddr : t4_seeprom_read(adap, vaddr, v);
}

static int eeprom_wr_phys(struct adapter *adap, unsigned int phys_addr, u32 v)
{
	int vaddr = t4_eeprom_ptov(phys_addr, adap->pf, EEPROMPFSIZE);

	return vaddr < 0 ? vaddr : t4_seeprom_write(adap, vaddr, v);
}

static int get_eeprom(struct net_device *dev, struct ethtool_eeprom *e,
		      u8 *data)
{
	int i, err = 0;
	struct adapter *adapter = netdev2adap(dev);

	u8 *buf = t4_alloc_mem(EEPROMSIZE);
	if (!buf)
		return -ENOMEM;

	e->magic = EEPROM_MAGIC;
	for (i = e->offset & ~3; !err && i < e->offset + e->len; i += 4)
		err = eeprom_rd_phys(adapter, i, (u32 *)&buf[i]);

	if (!err)
		memcpy(data, buf + e->offset, e->len);
	t4_free_mem(buf);
	return err;
}

static int set_eeprom(struct net_device *dev, struct ethtool_eeprom *eeprom,
		      u8 *data)
{
	u8 *buf;
	int err = 0;
	u32 aligned_offset, aligned_len, *p;
	struct adapter *adapter = netdev2adap(dev);

	if (eeprom->magic != EEPROM_MAGIC)
		return -EINVAL;

	aligned_offset = eeprom->offset & ~3;
	aligned_len = (eeprom->len + (eeprom->offset & 3) + 3) & ~3;

	if (adapter->pf > 0) {
		u32 start = 1024 + adapter->pf * EEPROMPFSIZE;

		if (aligned_offset < start ||
		    aligned_offset + aligned_len > start + EEPROMPFSIZE)
			return -EPERM;
	}

	if (aligned_offset != eeprom->offset || aligned_len != eeprom->len) {
		/*
		 * RMW possibly needed for first or last words.
		 */
		buf = t4_alloc_mem(aligned_len);
		if (!buf)
			return -ENOMEM;
		err = eeprom_rd_phys(adapter, aligned_offset, (u32 *)buf);
		if (!err && aligned_len > 4)
			err = eeprom_rd_phys(adapter,
					     aligned_offset + aligned_len - 4,
					     (u32 *)&buf[aligned_len - 4]);
		if (err)
			goto out;
		memcpy(buf + (eeprom->offset & 3), data, eeprom->len);
	} else
		buf = data;

	err = t4_seeprom_wp(adapter, 0);
	if (err)
		goto out;

	for (p = (u32 *)buf; !err && aligned_len; aligned_len -= 4, p++) {
		err = eeprom_wr_phys(adapter, aligned_offset, *p);
		aligned_offset += 4;
	}

	if (!err)
		err = t4_seeprom_wp(adapter, 1);
out:
	if (buf != data)
		t4_free_mem(buf);
	return err;
}

#define WOL_SUPPORTED (WAKE_BCAST | WAKE_MAGIC)
#define BCAST_CRC 0xa0ccc1a6

static void get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	wol->supported = WAKE_BCAST | WAKE_MAGIC;
	wol->wolopts = netdev2adap(dev)->wol;
	memset(&wol->sopass, 0, sizeof(wol->sopass));
}

static int set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	int err = 0;
	struct port_info *pi = netdev_priv(dev);

	if (wol->wolopts & ~WOL_SUPPORTED)
		return -EINVAL;
	t4_wol_magic_enable(pi->adapter, pi->tx_chan,
			    (wol->wolopts & WAKE_MAGIC) ? dev->dev_addr : NULL);
	if (wol->wolopts & WAKE_BCAST) {
		err = t4_wol_pat_enable(pi->adapter, pi->tx_chan, 0xfe, ~0ULL,
					~0ULL, 0, false);
		if (!err)
			err = t4_wol_pat_enable(pi->adapter, pi->tx_chan, 1,
						~6ULL, ~0ULL, BCAST_CRC, true);
	} else
		t4_wol_pat_enable(pi->adapter, pi->tx_chan, 0, 0, 0, 0, false);
	return err;
}

#define TSO_FLAGS (NETIF_F_TSO | NETIF_F_TSO6 | NETIF_F_TSO_ECN)

static int set_tso(struct net_device *dev, u32 value)
{
	if (value)
		dev->features |= TSO_FLAGS;
	else
		dev->features &= ~TSO_FLAGS;
	return 0;
}

static struct ethtool_ops cxgb_ethtool_ops = {
	.get_settings      = get_settings,
	.set_settings      = set_settings,
	.get_drvinfo       = get_drvinfo,
	.get_msglevel      = get_msglevel,
	.set_msglevel      = set_msglevel,
	.get_ringparam     = get_sge_param,
	.set_ringparam     = set_sge_param,
	.get_coalesce      = get_coalesce,
	.set_coalesce      = set_coalesce,
	.get_eeprom_len    = get_eeprom_len,
	.get_eeprom        = get_eeprom,
	.set_eeprom        = set_eeprom,
	.get_pauseparam    = get_pauseparam,
	.set_pauseparam    = set_pauseparam,
	.get_rx_csum       = get_rx_csum,
	.set_rx_csum       = set_rx_csum,
	.set_tx_csum       = ethtool_op_set_tx_ipv6_csum,
	.set_sg            = ethtool_op_set_sg,
	.get_link          = ethtool_op_get_link,
	.get_strings       = get_strings,
	.phys_id           = identify_port,
	.nway_reset        = restart_autoneg,
	.get_sset_count    = get_sset_count,
	.get_ethtool_stats = get_stats,
	.get_regs_len      = get_regs_len,
	.get_regs          = get_regs,
	.get_wol           = get_wol,
	.set_wol           = set_wol,
	.set_tso           = set_tso,
};

/*
 * debugfs support
 */

/*
 * Read a vector of numbers from a user space buffer.  Each number must be
 * between min and max inclusive and in the given base.
 */
static int rd_usr_int_vec(const char __user *buf, size_t usr_len, int vec_len,
			  unsigned long *vals, unsigned long min,
			  unsigned long max, int base)
{
	size_t l;
	unsigned long v;
	char c, word[68], *end;

	while (usr_len) {
		/* skip whitespace to beginning of next word */
		while (usr_len) {
			if (get_user(c, buf))
				return -EFAULT;
			if (!isspace(c))
				break;
			usr_len--;
			buf++;
		}

		if (!usr_len)
			break;
		if (!vec_len)
			return -EINVAL;              /* too many numbers */

		/* get next word (possibly going beyond its end) */
		l = min(usr_len, sizeof(word) - 1);
		if (copy_from_user(word, buf, l))
			return -EFAULT;
		word[l] = '\0';

		v = simple_strtoul(word, &end, base);
		l = end - word;
		if (!l)
			return -EINVAL;              /* catch embedded '\0's */
		if (*end && !isspace(*end))
			return -EINVAL;
		/*
		 * Complain if we encountered a too long sequence of digits.
		 * The most we can consume in one iteration is for a 64-bit
		 * number in binary.  Too bad simple_strtoul doesn't catch
		 * overflows.
		 */
		if (l > 64)
			return -EINVAL;
		if (v < min || v > max)
			return -ERANGE;
		*vals++ = v;
		vec_len--;
		usr_len -= l;
		buf += l;
	}
	if (vec_len)
		return -EINVAL;                      /* not enough numbers */
	return 0;
}

/*
 * generic seq_file support for showing a table of size rows x width.
 */
struct seq_tab {
	int (*show)(struct seq_file *seq, void *v, int idx);
	unsigned int rows;        /* # of entries */
	unsigned char width;      /* size in bytes of each entry */
	unsigned char skip_first; /* whether the first line is a header */
	char data[0];             /* the table data */
};

static void *seq_tab_get_idx(struct seq_tab *tb, loff_t pos)
{
	pos -= tb->skip_first;
	return pos >= tb->rows ? NULL : &tb->data[pos * tb->width];
}

static void *seq_tab_start(struct seq_file *seq, loff_t *pos)
{
	struct seq_tab *tb = seq->private;

	if (tb->skip_first && *pos == 0)
		return SEQ_START_TOKEN;

	return seq_tab_get_idx(tb, *pos);
}

static void *seq_tab_next(struct seq_file *seq, void *v, loff_t *pos)
{
	v = seq_tab_get_idx(seq->private, *pos + 1);
	if (v)
		++*pos;
	return v;
}

static void seq_tab_stop(struct seq_file *seq, void *v)
{
}

static int seq_tab_show(struct seq_file *seq, void *v)
{
	const struct seq_tab *tb = seq->private;

	/*
	 * index is bogus when v isn't within data, eg when it's
	 * SEQ_START_TOKEN, but that's OK
	 */
	return tb->show(seq, v, ((char *)v - tb->data) / tb->width);
}

static const struct seq_operations seq_tab_ops = {
	.start = seq_tab_start,
	.next  = seq_tab_next,
	.stop  = seq_tab_stop,
	.show  = seq_tab_show
};

struct seq_tab *seq_open_tab(struct file *f, unsigned int rows,
			     unsigned int width, unsigned int have_header,
			     int (*show)(struct seq_file *seq, void *v, int i))
{
	struct seq_tab *p;

	p = __seq_open_private(f, &seq_tab_ops, sizeof(*p) + rows * width);
	if (p) {
		p->show = show;
		p->rows = rows;
		p->width = width;
		p->skip_first = have_header != 0;
	}
	return p;
}

/*
 * Trim the size of a seq_tab to the supplied number of rows.  The opration is
 * irreversible.
 */
static int seq_tab_trim(struct seq_tab *p, unsigned int new_rows)
{
	if (new_rows > p->rows)
		return -EINVAL;
	p->rows = new_rows;
	return 0;
}

static int cim_la_show(struct seq_file *seq, void *v, int idx)
{
	if (v == SEQ_START_TOKEN)
		seq_puts(seq, "Status   Data      PC     LS0Stat  LS0Addr "
			 "            LS0Data\n");
	else {
		const u32 *p = v;

		seq_printf(seq,
			"  %02x   %x%07x %x%07x %08x %08x %08x%08x%08x%08x\n",
			(p[0] >> 4) & 0xff, p[0] & 0xf, p[1] >> 4, p[1] & 0xf,
			p[2] >> 4, p[2] & 0xf, p[3], p[4], p[5], p[6], p[7]);
	}
	return 0;
}

static int cim_la_show_3in1(struct seq_file *seq, void *v, int idx)
{
	if (v == SEQ_START_TOKEN)
		seq_puts(seq, "Status   Data      PC\n");
	else {
		const u32 *p = v;

		seq_printf(seq, "  %02x   %08x %08x\n", p[5] & 0xff, p[6],
			   p[7]);
		seq_printf(seq, "  %02x   %02x%06x %02x%06x\n",
			   (p[3] >> 8) & 0xff, p[3] & 0xff, p[4] >> 8,
			   p[4] & 0xff, p[5] >> 8);
		seq_printf(seq, "  %02x   %x%07x %x%07x\n", (p[0] >> 4) & 0xff,
			   p[0] & 0xf, p[1] >> 4, p[1] & 0xf, p[2] >> 4);
	}
	return 0;
}

static int cim_la_open(struct inode *inode, struct file *file)
{
	int ret;
	unsigned int cfg;
	struct seq_tab *p;
	struct adapter *adap = inode->i_private;

	ret = t4_cim_read(adap, A_UP_UP_DBG_LA_CFG, 1, &cfg);
	if (ret)
		return ret;

	p = seq_open_tab(file, adap->params.cim_la_size / 8, 8 * sizeof(u32), 1,
		cfg & F_UPDBGLACAPTPCONLY ? cim_la_show_3in1 : cim_la_show);
	if (!p)
		return -ENOMEM;

	ret = t4_cim_read_la(adap, (u32 *)p->data, NULL);
	if (ret)
		seq_release_private(inode, file);
	return ret;
}

static const struct file_operations cim_la_fops = {
	.owner   = THIS_MODULE,
	.open    = cim_la_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};

static int cim_pif_la_show(struct seq_file *seq, void *v, int idx)
{
	const u32 *p = v;

	if (v == SEQ_START_TOKEN)
		seq_puts(seq, "Cntl ID DataBE   Addr                 Data\n");
	else if (idx < CIM_PIFLA_SIZE)
		seq_printf(seq, " %02x  %02x  %04x  %08x %08x%08x%08x%08x\n",
			   (p[5] >> 22) & 0xff, (p[5] >> 16) & 0x3f,
			   p[5] & 0xffff, p[4], p[3], p[2], p[1], p[0]);
	else {
		if (idx == CIM_PIFLA_SIZE)
			seq_puts(seq, "\nCntl ID               Data\n");
		seq_printf(seq, " %02x  %02x %08x%08x%08x%08x\n",
			   (p[4] >> 6) & 0xff, p[4] & 0x3f,
			   p[3], p[2], p[1], p[0]);
	}
	return 0;
}

static int cim_pif_la_open(struct inode *inode, struct file *file)
{
	struct seq_tab *p;
	struct adapter *adap = inode->i_private;

	p = seq_open_tab(file, 2 * CIM_PIFLA_SIZE, 6 * sizeof(u32), 1,
			 cim_pif_la_show);
	if (!p)
		return -ENOMEM;

	t4_cim_read_pif_la(adap, (u32 *)p->data,
			   (u32 *)p->data + 6 * CIM_PIFLA_SIZE, NULL, NULL);
	return 0;
}

static const struct file_operations cim_pif_la_fops = {
	.owner   = THIS_MODULE,
	.open    = cim_pif_la_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};

static int cim_ma_la_show(struct seq_file *seq, void *v, int idx)
{
	const u32 *p = v;

	if (v == SEQ_START_TOKEN)
		seq_puts(seq, "\n");
	else if (idx < CIM_MALA_SIZE)
		seq_printf(seq, "%02x%08x%08x%08x%08x\n",
			   p[4], p[3], p[2], p[1], p[0]);
	else {
		if (idx == CIM_MALA_SIZE)
			seq_puts(seq,
				 "\nCnt ID Tag UE       Data       RDY VLD\n");
		seq_printf(seq, "%3u %2u  %x   %u %08x%08x  %u   %u\n",
			   (p[2] >> 10) & 0xff, (p[2] >> 7) & 7,
			   (p[2] >> 3) & 0xf, (p[2] >> 2) & 1,
			   (p[1] >> 2) | ((p[2] & 3) << 30),
			   (p[0] >> 2) | ((p[1] & 3) << 30), (p[0] >> 1) & 1,
			   p[0] & 1);
	}
	return 0;
}

static int cim_ma_la_open(struct inode *inode, struct file *file)
{
	struct seq_tab *p;
	struct adapter *adap = inode->i_private;

	p = seq_open_tab(file, 2 * CIM_MALA_SIZE, 5 * sizeof(u32), 1,
			 cim_ma_la_show);
	if (!p)
		return -ENOMEM;

	t4_cim_read_ma_la(adap, (u32 *)p->data,
			  (u32 *)p->data + 5 * CIM_MALA_SIZE);
	return 0;
}

static const struct file_operations cim_ma_la_fops = {
	.owner   = THIS_MODULE,
	.open    = cim_ma_la_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};

static int cim_qcfg_show(struct seq_file *seq, void *v)
{
	static const char *qname[] = {
		"TP0", "TP1", "ULP", "SGE0", "SGE1", "NC-SI",
		"ULP0", "ULP1", "ULP2", "ULP3", "SGE", "NC-SI"
	};

	int i;
	u16 base[CIM_NUM_IBQ + CIM_NUM_OBQ];
	u16 size[CIM_NUM_IBQ + CIM_NUM_OBQ];
	u16 thres[CIM_NUM_IBQ];
	u32 obq_wr[2 * CIM_NUM_OBQ], *wr = obq_wr;
	u32 stat[4 * (CIM_NUM_IBQ + CIM_NUM_OBQ)], *p = stat;
	struct adapter *adap = seq->private;

	i = t4_cim_read(adap, A_UP_IBQ_0_RDADDR, ARRAY_SIZE(stat), stat);
	if (!i)
		i = t4_cim_read(adap, A_UP_OBQ_0_REALADDR, ARRAY_SIZE(obq_wr),
				obq_wr);
	if (i)
		return i;

	t4_read_cimq_cfg(adap, base, size, thres);

	seq_printf(seq,
		   "Queue  Base  Size Thres RdPtr WrPtr  SOP  EOP Avail\n");
	for (i = 0; i < CIM_NUM_IBQ; i++, p += 4)
		seq_printf(seq, "%5s %5x %5u %4u %6x  %4x %4u %4u %5u\n",
			   qname[i], base[i], size[i], thres[i],
			   G_IBQRDADDR(p[0]), G_IBQWRADDR(p[1]),
			   G_QUESOPCNT(p[3]), G_QUEEOPCNT(p[3]),
			   G_QUEREMFLITS(p[2]) * 16);
	for ( ; i < CIM_NUM_IBQ + CIM_NUM_OBQ; i++, p += 4, wr += 2)
		seq_printf(seq, "%5s %5x %5u %11x  %4x %4u %4u %5u\n",
			   qname[i], base[i], size[i],
			   G_QUERDADDR(p[0]) & 0x3fff, wr[0] - base[i],
			   G_QUESOPCNT(p[3]), G_QUEEOPCNT(p[3]),
			   G_QUEREMFLITS(p[2]) * 16);
	return 0;
}

static int cim_qcfg_open(struct inode *inode, struct file *file)
{
	return single_open(file, cim_qcfg_show, inode->i_private);
}

static const struct file_operations cim_qcfg_fops = {
	.owner   = THIS_MODULE,
	.open    = cim_qcfg_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int cimq_show(struct seq_file *seq, void *v, int idx)
{
	const u32 *p = v;

	seq_printf(seq, "%#06x: %08x %08x %08x %08x\n", idx * 16, p[0], p[1],
		   p[2], p[3]);
	return 0;
}

static int cim_ibq_open(struct inode *inode, struct file *file)
{
	int ret;
	struct seq_tab *p;
	unsigned int qid = (uintptr_t)inode->i_private & 7;
	struct adapter *adap = inode->i_private - qid;

	p = seq_open_tab(file, CIM_IBQ_SIZE, 4 * sizeof(u32), 0, cimq_show);
	if (!p)
		return -ENOMEM;

	ret = t4_read_cim_ibq(adap, qid, (u32 *)p->data, CIM_IBQ_SIZE * 4);
	if (ret < 0)
		seq_release_private(inode, file);
	else
		ret = 0;
	return ret;
}

static const struct file_operations cim_ibq_fops = {
	.owner   = THIS_MODULE,
	.open    = cim_ibq_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};

static int cim_obq_open(struct inode *inode, struct file *file)
{
	int ret;
	struct seq_tab *p;
	unsigned int qid = (uintptr_t)inode->i_private & 7;
	struct adapter *adap = inode->i_private - qid;

	p = seq_open_tab(file, 6 * CIM_IBQ_SIZE, 4 * sizeof(u32), 0, cimq_show);
	if (!p)
		return -ENOMEM;

	ret = t4_read_cim_obq(adap, qid, (u32 *)p->data, 6 * CIM_IBQ_SIZE * 4);
	if (ret < 0)
		seq_release_private(inode, file);
	else {
		seq_tab_trim(p, ret / 4);
		ret = 0;
	}
	return ret;
}

static const struct file_operations cim_obq_fops = {
	.owner   = THIS_MODULE,
	.open    = cim_obq_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};

struct field_desc {
	const char *name;
	unsigned int start;
	unsigned int width;
};

static void field_desc_show(struct seq_file *seq, u64 v,
			    const struct field_desc *p)
{
	char buf[32];
	int line_size = 0;

	while (p->name) {
		u64 mask = (1ULL << p->width) - 1;
		int len = scnprintf(buf, sizeof(buf), "%s: %llu", p->name,
				    ((unsigned long long)v >> p->start) & mask);

		if (line_size + len >= 79) {
			line_size = 8;
			seq_puts(seq, "\n        ");
		}
		seq_printf(seq, "%s ", buf);
		line_size += len + 1;
		p++;
	}
	seq_putc(seq, '\n');
}

static struct field_desc tp_la0[] = {
	{ "RcfOpCodeOut", 60, 4 },
	{ "State", 56, 4 },
	{ "WcfState", 52, 4 },
	{ "RcfOpcSrcOut", 50, 2 },
	{ "CRxError", 49, 1 },
	{ "ERxError", 48, 1 },
	{ "SanityFailed", 47, 1 },
	{ "SpuriousMsg", 46, 1 },
	{ "FlushInputMsg", 45, 1 },
	{ "FlushInputCpl", 44, 1 },
	{ "RssUpBit", 43, 1 },
	{ "RssFilterHit", 42, 1 },
	{ "Tid", 32, 10 },
	{ "InitTcb", 31, 1 },
	{ "LineNumber", 24, 7 },
	{ "Emsg", 23, 1 },
	{ "EdataOut", 22, 1 },
	{ "Cmsg", 21, 1 },
	{ "CdataOut", 20, 1 },
	{ "EreadPdu", 19, 1 },
	{ "CreadPdu", 18, 1 },
	{ "TunnelPkt", 17, 1 },
	{ "RcfPeerFin", 16, 1 },
	{ "RcfReasonOut", 12, 4 },
	{ "TxCchannel", 10, 2 },
	{ "RcfTxChannel", 8, 2 },
	{ "RxEchannel", 6, 2 },
	{ "RcfRxChannel", 5, 1 },
	{ "RcfDataOutSrdy", 4, 1 },
	{ "RxDvld", 3, 1 },
	{ "RxOoDvld", 2, 1 },
	{ "RxCongestion", 1, 1 },
	{ "TxCongestion", 0, 1 },
	{ NULL }
};

static int tp_la_show(struct seq_file *seq, void *v, int idx)
{
	const u64 *p = v;

	field_desc_show(seq, *p, tp_la0);
	return 0;
}

static int tp_la_show2(struct seq_file *seq, void *v, int idx)
{
	const u64 *p = v;

	if (idx)
		seq_putc(seq, '\n');
	field_desc_show(seq, p[0], tp_la0);
	if (idx < (TPLA_SIZE / 2 - 1) || p[1] != ~0ULL)
		field_desc_show(seq, p[1], tp_la0);
	return 0;
}

static int tp_la_show3(struct seq_file *seq, void *v, int idx)
{
	static struct field_desc tp_la1[] = {
		{ "CplCmdIn", 56, 8 },
		{ "CplCmdOut", 48, 8 },
		{ "ESynOut", 47, 1 },
		{ "EAckOut", 46, 1 },
		{ "EFinOut", 45, 1 },
		{ "ERstOut", 44, 1 },
		{ "SynIn", 43, 1 },
		{ "AckIn", 42, 1 },
		{ "FinIn", 41, 1 },
		{ "RstIn", 40, 1 },
		{ "DataIn", 39, 1 },
		{ "DataInVld", 38, 1 },
		{ "PadIn", 37, 1 },
		{ "RxBufEmpty", 36, 1 },
		{ "RxDdp", 35, 1 },
		{ "RxFbCongestion", 34, 1 },
		{ "TxFbCongestion", 33, 1 },
		{ "TxPktSumSrdy", 32, 1 },
		{ "RcfUlpType", 28, 4 },
		{ "Eread", 27, 1 },
		{ "Ebypass", 26, 1 },
		{ "Esave", 25, 1 },
		{ "Static0", 24, 1 },
		{ "Cread", 23, 1 },
		{ "Cbypass", 22, 1 },
		{ "Csave", 21, 1 },
		{ "CPktOut", 20, 1 },
		{ "RxPagePoolFull", 18, 2 },
		{ "RxLpbkPkt", 17, 1 },
		{ "TxLpbkPkt", 16, 1 },
		{ "RxVfValid", 15, 1 },
		{ "SynLearned", 14, 1 },
		{ "SetDelEntry", 13, 1 },
		{ "SetInvEntry", 12, 1 },
		{ "CpcmdDvld", 11, 1 },
		{ "CpcmdSave", 10, 1 },
		{ "RxPstructsFull", 8, 2 },
		{ "EpcmdDvld", 7, 1 },
		{ "EpcmdFlush", 6, 1 },
		{ "EpcmdTrimPrefix", 5, 1 },
		{ "EpcmdTrimPostfix", 4, 1 },
		{ "ERssIp4Pkt", 3, 1 },
		{ "ERssIp6Pkt", 2, 1 },
		{ "ERssTcpUdpPkt", 1, 1 },
		{ "ERssFceFipPkt", 0, 1 },
		{ NULL }
	};
	static struct field_desc tp_la2[] = {
		{ "CplCmdIn", 56, 8 },
		{ "MpsVfVld", 55, 1 },
		{ "MpsPf", 52, 3 },
		{ "MpsVf", 44, 8 },
		{ "SynIn", 43, 1 },
		{ "AckIn", 42, 1 },
		{ "FinIn", 41, 1 },
		{ "RstIn", 40, 1 },
		{ "DataIn", 39, 1 },
		{ "DataInVld", 38, 1 },
		{ "PadIn", 37, 1 },
		{ "RxBufEmpty", 36, 1 },
		{ "RxDdp", 35, 1 },
		{ "RxFbCongestion", 34, 1 },
		{ "TxFbCongestion", 33, 1 },
		{ "TxPktSumSrdy", 32, 1 },
		{ "RcfUlpType", 28, 4 },
		{ "Eread", 27, 1 },
		{ "Ebypass", 26, 1 },
		{ "Esave", 25, 1 },
		{ "Static0", 24, 1 },
		{ "Cread", 23, 1 },
		{ "Cbypass", 22, 1 },
		{ "Csave", 21, 1 },
		{ "CPktOut", 20, 1 },
		{ "RxPagePoolFull", 18, 2 },
		{ "RxLpbkPkt", 17, 1 },
		{ "TxLpbkPkt", 16, 1 },
		{ "RxVfValid", 15, 1 },
		{ "SynLearned", 14, 1 },
		{ "SetDelEntry", 13, 1 },
		{ "SetInvEntry", 12, 1 },
		{ "CpcmdDvld", 11, 1 },
		{ "CpcmdSave", 10, 1 },
		{ "RxPstructsFull", 8, 2 },
		{ "EpcmdDvld", 7, 1 },
		{ "EpcmdFlush", 6, 1 },
		{ "EpcmdTrimPrefix", 5, 1 },
		{ "EpcmdTrimPostfix", 4, 1 },
		{ "ERssIp4Pkt", 3, 1 },
		{ "ERssIp6Pkt", 2, 1 },
		{ "ERssTcpUdpPkt", 1, 1 },
		{ "ERssFceFipPkt", 0, 1 },
		{ NULL }
	};
	const u64 *p = v;

	if (idx)
		seq_putc(seq, '\n');
	field_desc_show(seq, p[0], tp_la0);
	if (idx < (TPLA_SIZE / 2 - 1) || p[1] != ~0ULL)
		field_desc_show(seq, p[1], (p[0] & BIT(17)) ? tp_la2 : tp_la1);
	return 0;
}

static int tp_la_open(struct inode *inode, struct file *file)
{
	struct seq_tab *p;
	struct adapter *adap = inode->i_private;

	switch (G_DBGLAMODE(t4_read_reg(adap, A_TP_DBG_LA_CONFIG))) {
	case 2:
		p = seq_open_tab(file, TPLA_SIZE / 2, 2 * sizeof(u64), 0,
				 tp_la_show2);
		break;
	case 3:
		p = seq_open_tab(file, TPLA_SIZE / 2, 2 * sizeof(u64), 0,
				 tp_la_show3);
		break;
	default:
		p = seq_open_tab(file, TPLA_SIZE, sizeof(u64), 0, tp_la_show);
	}
	if (!p)
		return -ENOMEM;

	t4_tp_read_la(adap, (u64 *)p->data, NULL);
	return 0;
}

static ssize_t tp_la_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *pos)
{
	int err;
	char s[32];
	unsigned long val;
	size_t size = min(sizeof(s) - 1, count);
	struct adapter *adap = file->f_path.dentry->d_inode->i_private;

	if (copy_from_user(s, buf, size))
		return -EFAULT;
	s[size] = '\0';
	err = strict_strtoul(s, 0, &val);
	if (err)
		return err;
	if (val > 0xffff)
		return -EINVAL;
	adap->params.tp.la_mask = val << 16;
	t4_set_reg_field(adap, A_TP_DBG_LA_CONFIG, 0xffff0000U,
			 adap->params.tp.la_mask);
	return count;
}

static const struct file_operations tp_la_fops = {
	.owner   = THIS_MODULE,
	.open    = tp_la_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private,
	.write   = tp_la_write
};

static int ulprx_la_show(struct seq_file *seq, void *v, int idx)
{
	const u32 *p = v;

	if (v == SEQ_START_TOKEN)
		seq_puts(seq, "      Pcmd        Type   Message"
			 "                Data\n");
	else
		seq_printf(seq, "%08x%08x  %4x  %08x  %08x%08x%08x%08x\n",
			   p[1], p[0], p[2], p[3], p[7], p[6], p[5], p[4]);
	return 0;
}

static int ulprx_la_open(struct inode *inode, struct file *file)
{
	struct seq_tab *p;
	struct adapter *adap = inode->i_private;

	p = seq_open_tab(file, ULPRX_LA_SIZE, 8 * sizeof(u32), 1,
			 ulprx_la_show);
	if (!p)
		return -ENOMEM;

	t4_ulprx_read_la(adap, (u32 *)p->data);
	return 0;
}

static const struct file_operations ulprx_la_fops = {
	.owner   = THIS_MODULE,
	.open    = ulprx_la_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};

/*
 * Format a value in a unit that differs from the value's native unit by the
 * given factor.
 */
static char *unit_conv(char *buf, size_t len, unsigned int val,
		       unsigned int factor)
{
	unsigned int rem = val % factor;

	if (rem == 0)
		snprintf(buf, len, "%u", val / factor);
	else {
		while (rem % 10 == 0)
			rem /= 10;
		snprintf(buf, len, "%u.%u", val / factor, rem);
	}
	return buf;
}

static int clk_show(struct seq_file *seq, void *v)
{
	char buf[32];
	struct adapter *adap = seq->private;
	unsigned int cclk_ps = 1000000000 / adap->params.vpd.cclk;  /* in ps */
	u32 res = t4_read_reg(adap, A_TP_TIMER_RESOLUTION);
	unsigned int tre = G_TIMERRESOLUTION(res);
	unsigned int dack_re = G_DELAYEDACKRESOLUTION(res);
	unsigned long long tp_tick_us = (cclk_ps << tre) / 1000000; /* in us */

	seq_printf(seq, "Core clock period: %s ns\n",
		   unit_conv(buf, sizeof(buf), cclk_ps, 1000));
	seq_printf(seq, "TP timer tick: %s us\n",
		   unit_conv(buf, sizeof(buf), (cclk_ps << tre), 1000000));
	seq_printf(seq, "TCP timestamp tick: %s us\n",
		   unit_conv(buf, sizeof(buf),
			     (cclk_ps << G_TIMESTAMPRESOLUTION(res)), 1000000));
	seq_printf(seq, "DACK tick: %s us\n",
		   unit_conv(buf, sizeof(buf), (cclk_ps << dack_re), 1000000));
	seq_printf(seq, "DACK timer: %u us\n",
		   ((cclk_ps << dack_re) / 1000000) *
		   t4_read_reg(adap, A_TP_DACK_TIMER));
	seq_printf(seq, "Retransmit min: %llu us\n",
		   tp_tick_us * t4_read_reg(adap, A_TP_RXT_MIN));
	seq_printf(seq, "Retransmit max: %llu us\n",
		   tp_tick_us * t4_read_reg(adap, A_TP_RXT_MAX));
	seq_printf(seq, "Persist timer min: %llu us\n",
		   tp_tick_us * t4_read_reg(adap, A_TP_PERS_MIN));
	seq_printf(seq, "Persist timer max: %llu us\n",
		   tp_tick_us * t4_read_reg(adap, A_TP_PERS_MAX));
	seq_printf(seq, "Keepalive idle timer: %llu us\n",
		   tp_tick_us * t4_read_reg(adap, A_TP_KEEP_IDLE));
	seq_printf(seq, "Keepalive interval: %llu us\n",
		   tp_tick_us * t4_read_reg(adap, A_TP_KEEP_INTVL));
	seq_printf(seq, "Initial SRTT: %llu us\n",
		   tp_tick_us * t4_read_reg(adap, A_TP_INIT_SRTT));
	seq_printf(seq, "FINWAIT2 timer: %llu us\n",
		   tp_tick_us * t4_read_reg(adap, A_TP_FINWAIT2_TIMER));

	return 0;
}

static int clk_open(struct inode *inode, struct file *file)
{
	return single_open(file, clk_show, inode->i_private);
}

static const struct file_operations clk_debugfs_fops = {
	.owner   = THIS_MODULE,
	.open    = clk_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

/*
 * Firmware Device Log dump.
 * =========================
 */
const char *devlog_level_strings[] = {
	[FW_DEVLOG_LEVEL_EMERG]		= "EMERG",
	[FW_DEVLOG_LEVEL_CRIT]		= "CRIT",
	[FW_DEVLOG_LEVEL_ERR]		= "ERR",
	[FW_DEVLOG_LEVEL_NOTICE]	= "NOTICE",
	[FW_DEVLOG_LEVEL_INFO]		= "INFO",
	[FW_DEVLOG_LEVEL_DEBUG]		= "DEBUG"
};

const char *devlog_facility_strings[] = {
	[FW_DEVLOG_FACILITY_CORE]	= "CORE",
	[FW_DEVLOG_FACILITY_SCHED]	= "SCHED",
	[FW_DEVLOG_FACILITY_TIMER]	= "TIMER",
	[FW_DEVLOG_FACILITY_RES]	= "RES",
	[FW_DEVLOG_FACILITY_HW]		= "HW",
	[FW_DEVLOG_FACILITY_FLR]	= "FLR",
	[FW_DEVLOG_FACILITY_DMAQ]	= "DMAQ",
	[FW_DEVLOG_FACILITY_PHY]	= "PHY",
	[FW_DEVLOG_FACILITY_MAC]	= "MAC",
	[FW_DEVLOG_FACILITY_PORT]	= "PORT",
	[FW_DEVLOG_FACILITY_VI]		= "VI",
	[FW_DEVLOG_FACILITY_FILTER]	= "FILTER",
	[FW_DEVLOG_FACILITY_ACL]	= "ACL",
	[FW_DEVLOG_FACILITY_TM]		= "TM",
	[FW_DEVLOG_FACILITY_QFC]	= "QFC",
	[FW_DEVLOG_FACILITY_DCB]	= "DCB",
	[FW_DEVLOG_FACILITY_ETH]	= "ETH",
	[FW_DEVLOG_FACILITY_OFLD]	= "OFLD",
	[FW_DEVLOG_FACILITY_RI]		= "RI",
	[FW_DEVLOG_FACILITY_ISCSI]	= "ISCSI",
	[FW_DEVLOG_FACILITY_FCOE]	= "FCOE",
	[FW_DEVLOG_FACILITY_FOISCSI]	= "FOISCSI",
	[FW_DEVLOG_FACILITY_FOFCOE]	= "FOFCOE"
};

/*
 * Information gathered by Device Log Open routine for the display routine.
 */
struct devlog_info {
	unsigned int nentries;		/* number of entries in log[] */
	unsigned int first;		/* first [temporal] entry in log[] */
	struct fw_devlog_e log[0];	/* Firmware Device Log */
};

/*
 * Dump a Firmaware Device Log entry.
 */
static int devlog_show(struct seq_file *seq, void *v)
{
	if (v == SEQ_START_TOKEN)
		seq_printf(seq, "%10s  %15s  %8s  %8s  %s\n",
			   "Seq#", "Tstamp", "Level", "Facility", "Message");
	else {
		struct devlog_info *dinfo = seq->private;
		int fidx = (uintptr_t)v - 2;
		unsigned long index;
		struct fw_devlog_e *e;

		/*
		 * Get a pointer to the log entry to display.  Skip unused log
		 * entries.
		 */
		index = dinfo->first + fidx;
		if (index >= dinfo->nentries)
			index -= dinfo->nentries;
		e = &dinfo->log[index];
		if (e->timestamp == 0)
			return 0;

		/*
		 * Print the message.  This depends on the firmware using
		 * exactly the same formating strings as the kernel so we may
		 * eventually have to put a format interpreter in here ...
		 */
		seq_printf(seq, "%10d  %15llu  %8s  %8s  ",
			   e->seqno, e->timestamp,
			   (e->level < ARRAY_SIZE(devlog_level_strings)
			    ? devlog_level_strings[e->level]
			    : "UNKNOWN"),
			   (e->facility < ARRAY_SIZE(devlog_facility_strings)
			    ? devlog_facility_strings[e->facility]
			    : "UNKNOWN"));
		seq_printf(seq, e->fmt, e->params[0], e->params[1],
			   e->params[2], e->params[3], e->params[4],
			   e->params[5], e->params[6], e->params[7]);
	}

	return 0;
}

/*
 * Sequential File Operations for Device Log.
 */
static inline void *devlog_get_idx(struct devlog_info *dinfo, loff_t pos)
{
	if (pos > dinfo->nentries)
		return NULL;

	return (void *)(uintptr_t)(pos + 1);
}

static void *devlog_start(struct seq_file *seq, loff_t *pos)
{
	struct devlog_info *dinfo = seq->private;

	return (*pos
		? devlog_get_idx(dinfo, *pos)
		: SEQ_START_TOKEN);
}

static void *devlog_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct devlog_info *dinfo = seq->private;

	(*pos)++;
	return devlog_get_idx(dinfo, *pos);
}

static void devlog_stop(struct seq_file *seq, void *v)
{
}

static const struct seq_operations devlog_seq_ops = {
	.start = devlog_start,
	.next  = devlog_next,
	.stop  = devlog_stop,
	.show  = devlog_show
};

/*
 * Set up for reading the firmware's device log.  We read the entire log here
 * and then display it incrementally in devlog_show().
 */
static int devlog_open(struct inode *inode, struct file *file)
{
	struct adapter *adap = inode->i_private;
	struct devlog_params *dparams = &adap->params.devlog;
	struct devlog_info *dinfo;
	unsigned int index;
	u64 ftstamp;
	int ret;

	/*
	 * If we don't know where the log is we can't do anything.
	 */
	if (dparams->start == 0)
		return -ENXIO;

	/*
	 * Allocate the space to read in the firmware's device log and set up
	 * for the iterated call to our display function.
	 */
	dinfo = __seq_open_private(file, &devlog_seq_ops,
				   sizeof *dinfo + dparams->size);
	if (dinfo == NULL)
		return -ENOMEM;

	/*
	 * Record the basic log buffer information and read in the raw log.
	 */
	dinfo->nentries = (dparams->size / sizeof (struct fw_devlog_e));
	dinfo->first = 0;
	ret = t4_memory_read(adap, dparams->memtype, dparams->start,
			  dparams->size, (__be32 *)dinfo->log);
	if (ret) {
		seq_release_private(inode, file);
		return ret;
	}

	/*
	 * Translate log multi-byte integral elements into host native format
	 * and determine where the first entry in the log is.
	 */
	for (ftstamp = ~0ULL, index = 0; index < dinfo->nentries; index++) {
		struct fw_devlog_e *e = &dinfo->log[index];
		int i;

		if (e->timestamp == 0)
			continue;

		e->timestamp = be64_to_cpu(e->timestamp);
		e->seqno = be32_to_cpu(e->seqno);
		for (i = 0; i < 8; i++)
			e->params[i] = be32_to_cpu(e->params[i]);

		if (e->timestamp < ftstamp) {
			ftstamp = e->timestamp;
			dinfo->first = index;
		}
	}

	return 0;
}

static const struct file_operations devlog_fops = {
	.owner   = THIS_MODULE,
	.open    = devlog_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};

static int mbox_show(struct seq_file *seq, void *v)
{
	static const char *owner[] = { "none", "FW", "driver", "unknown" };

	int i;
	unsigned int mbox = (uintptr_t)seq->private & 7;
	struct adapter *adap = seq->private - mbox;
	void __iomem *addr = adap->regs + PF_REG(mbox, A_CIM_PF_MAILBOX_DATA);
	void __iomem *ctrl = addr + MBOX_LEN;

	i = G_MBOWNER(readl(ctrl));
	seq_printf(seq, "mailbox owned by %s\n\n", owner[i]);

	for (i = 0; i < MBOX_LEN; i += 8)
		seq_printf(seq, "%016llx\n",
			   (unsigned long long)readq(addr + i));
	return 0;
}

static int mbox_open(struct inode *inode, struct file *file)
{
	return single_open(file, mbox_show, inode->i_private);
}

static ssize_t mbox_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *pos)
{
	int i;
	char c = '\n', s[256];
	unsigned long long data[8];
	const struct inode *ino;
	unsigned int mbox;
	struct adapter *adap;
	void __iomem *addr;
	void __iomem *ctrl;

	if (count > sizeof(s) - 1 || !count)
		return -EINVAL;
	if (copy_from_user(s, buf, count))
		return -EFAULT;
	s[count] = '\0';

	if (sscanf(s, "%llx %llx %llx %llx %llx %llx %llx %llx%c", &data[0],
		   &data[1], &data[2], &data[3], &data[4], &data[5], &data[6],
		   &data[7], &c) < 8 || c != '\n')
		return -EINVAL;

	ino = file->f_path.dentry->d_inode;
	mbox = (uintptr_t)ino->i_private & 7;
	adap = ino->i_private - mbox;
	addr = adap->regs + PF_REG(mbox, A_CIM_PF_MAILBOX_DATA);
	ctrl = addr + MBOX_LEN;

	if (G_MBOWNER(readl(ctrl)) != X_MBOWNER_PL)
		return -EBUSY;

	for (i = 0; i < 8; i++)
		writeq(data[i], addr + 8 * i);

	writel(F_MBMSGVALID | V_MBOWNER(X_MBOWNER_FW), ctrl);
	return count;
}

static const struct file_operations mbox_debugfs_fops = {
	.owner   = THIS_MODULE,
	.open    = mbox_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
	.write   = mbox_write
};

static int mem_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t mem_read(struct file *file, char __user *buf, size_t count,
			loff_t *ppos)
{
	u32 memoffset;
	loff_t pos = *ppos;
	loff_t avail = file->f_path.dentry->d_inode->i_size;
	unsigned int mem = (uintptr_t)file->private_data & 3;
	struct adapter *adap = file->private_data - mem;
	__be32 *data;

	if (pos < 0)
		return -EINVAL;
	if (pos >= avail)
		return 0;
	if (count > avail - pos)
		count = avail - pos;

	/* Offset into the region of memory which is being accessed
	 * MEM_EDC0 = 0
	 * MEM_EDC1 = 1
	 * MEM_MC   = 2
	 */
	memoffset = (mem * ( 5 * 1024 * 1024));

	while (count) {
		size_t len;
		int ret, ofst;

		data = t4_alloc_mem(MEMWIN0_APERTURE);
		if (!data)
			return -ENOMEM;

		ret = t4_mem_win_read(adap, (pos + memoffset), data);
		if (ret) {
			t4_free_mem(data);
			return ret;
		}

		ofst = pos % MEMWIN0_APERTURE;
		len = min(count, (size_t) MEMWIN0_APERTURE - ofst);
		if (copy_to_user(buf, (u8 *)data + ofst, len)) {
	                t4_free_mem(data);
			return -EFAULT;
		}

		buf += len;
		pos += len;
		count -= len;
	        t4_free_mem(data);
	}
	count = pos - *ppos;
	*ppos = pos;
	return count;
}

static const struct file_operations mem_debugfs_fops = {
	.owner   = THIS_MODULE,
	.open    = mem_open,
	.read    = mem_read,
};

static ssize_t flash_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	loff_t pos = *ppos;
	loff_t avail = file->f_path.dentry->d_inode->i_size;
	struct adapter *adap = file->private_data;

	if (pos < 0)
		return -EINVAL;
	if (pos >= avail)
		return 0;
	if (count > avail - pos)
		count = avail - pos;

	while (count) {
		size_t len;
		int ret, ofst;
		u8 data[256];

		ofst = pos & 3;
		len = min(count + ofst, sizeof(data));
		ret = t4_read_flash(adap, pos - ofst, (len + 3) / 4,
				    (u32 *)data, 1);
		if (ret)
			return ret;

		len -= ofst;
		if (copy_to_user(buf, data + ofst, len))
			return -EFAULT;

		buf += len;
		pos += len;
		count -= len;
	}
	count = pos - *ppos;
	*ppos = pos;
	return count;
}

static const struct file_operations flash_debugfs_fops = {
	.owner   = THIS_MODULE,
	.open    = mem_open,
	.read    = flash_read,
};

static inline void tcamxy2valmask(u64 x, u64 y, u8 *addr, u64 *mask)
{
	*mask = x | y;
	y = cpu_to_be64(y);
	memcpy(addr, (char *)&y + 2, ETH_ALEN);
}

static int mps_tcam_show(struct seq_file *seq, void *v)
{
	if (v == SEQ_START_TOKEN)
		seq_puts(seq, "Idx  Ethernet address     Mask     Vld Ports PF"
			 "  VF Repl P0 P1 P2 P3  ML\n");
	else {
		u64 mask;
		u8 addr[ETH_ALEN];
		struct adapter *adap = seq->private;
		unsigned int idx = (uintptr_t)v - 2;
		u64 tcamy = t4_read_reg64(adap, MPS_CLS_TCAM_Y_L(idx));
		u64 tcamx = t4_read_reg64(adap, MPS_CLS_TCAM_X_L(idx));
		u32 cls_lo = t4_read_reg(adap, MPS_CLS_SRAM_L(idx));
		u32 cls_hi = t4_read_reg(adap, MPS_CLS_SRAM_H(idx));

		if (tcamx & tcamy) {
			seq_printf(seq, "%3u         -\n", idx);
			goto out;
		}

		tcamxy2valmask(tcamx, tcamy, addr, &mask);
		seq_printf(seq, "%3u %02x:%02x:%02x:%02x:%02x:%02x %012llx"
			   "%3c   %#x%4u%4d%4c%4u%3u%3u%3u %#x\n",
			   idx, addr[0], addr[1], addr[2], addr[3], addr[4],
			   addr[5], (unsigned long long)mask,
			   (cls_lo & F_SRAM_VLD) ? 'Y' : 'N', G_PORTMAP(cls_hi),
			   G_PF(cls_lo),
			   (cls_lo & F_VF_VALID) ? G_VF(cls_lo) : -1,
			   (cls_lo & F_REPLICATE) ? 'Y' : 'N',
			   G_SRAM_PRIO0(cls_lo), G_SRAM_PRIO1(cls_lo),
			   G_SRAM_PRIO2(cls_lo), G_SRAM_PRIO3(cls_lo),
			   (cls_lo >> S_MULTILISTEN0) & 0xf);
	}
out:	return 0;
}

static inline void *mps_tcam_get_idx(loff_t pos)
{
	return (pos <= FW_CLS_TCAM_NUM_ENTRIES
		? (void *)(uintptr_t)(pos + 1)
		: NULL);
}

static void *mps_tcam_start(struct seq_file *seq, loff_t *pos)
{
	return *pos ? mps_tcam_get_idx(*pos) : SEQ_START_TOKEN;
}

static void *mps_tcam_next(struct seq_file *seq, void *v, loff_t *pos)
{
	++*pos;
	return mps_tcam_get_idx(*pos);
}

static void mps_tcam_stop(struct seq_file *seq, void *v)
{
}

static const struct seq_operations mps_tcam_seq_ops = {
	.start = mps_tcam_start,
	.next  = mps_tcam_next,
	.stop  = mps_tcam_stop,
	.show  = mps_tcam_show
};

static int mps_tcam_open(struct inode *inode, struct file *file)
{
	int res = seq_open(file, &mps_tcam_seq_ops);

	if (!res) {
		struct seq_file *seq = file->private_data;
		seq->private = inode->i_private;
	}
	return res;
}

static const struct file_operations mps_tcam_debugfs_fops = {
	.owner   = THIS_MODULE,
	.open    = mps_tcam_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release,
};

static int sge_qinfo_show(struct seq_file *seq, void *v)
{
	struct adapter *adap = seq->private;
	int eth_entries = DIV_ROUND_UP(adap->sge.ethqsets, 4);
	int toe_entries = DIV_ROUND_UP(adap->sge.ofldqsets, 4);
	int rdma_entries = DIV_ROUND_UP(adap->sge.rdmaqs, 4);
	int iscsi_entries = DIV_ROUND_UP(adap->sge.niscsiq, 4);
	int ctrl_entries = DIV_ROUND_UP(MAX_CTRL_QUEUES, 4);
	int i, r = (uintptr_t)v - 1;

	if (r)
		seq_putc(seq, '\n');

#define S3(fmt_spec, s, v) \
	seq_printf(seq, "%-12s", s); \
	for (i = 0; i < n; ++i) \
		seq_printf(seq, " %16" fmt_spec, v); \
		seq_putc(seq, '\n');
#define S(s, v) S3("s", s, v)
#define T(s, v) S3("u", s, tx[i].v)
#define R(s, v) S3("u", s, rx[i].v)

	if (r < eth_entries) {
		const struct sge_eth_rxq *rx = &adap->sge.ethrxq[r * 4];
		const struct sge_eth_txq *tx = &adap->sge.ethtxq[r * 4];
		int n = min(4, adap->sge.ethqsets - 4 * r);

		S("QType:", "Ethernet");
		S("Interface:",
		  rx[i].rspq.netdev ? rx[i].rspq.netdev->name : "N/A");
		T("TxQ ID:", q.cntxt_id);
		T("TxQ size:", q.size);
		T("TxQ inuse:", q.in_use);
		R("RspQ ID:", rspq.abs_id);
		R("RspQ size:", rspq.size);
		R("RspQE size:", rspq.iqe_len);
		S3("u", "Intr delay:", qtimer_val(adap, &rx[i].rspq));
		S3("u", "Intr pktcnt:",
		   adap->sge.counter_val[rx[i].rspq.pktcnt_idx]);
		R("FL ID:", fl.cntxt_id);
		R("FL size:", fl.size - 8);
		R("FL avail:", fl.avail);
	} else if ((r -= eth_entries) < toe_entries) {
		const struct sge_ofld_rxq *rx = &adap->sge.ofldrxq[r * 4];
		const struct sge_ofld_txq *tx = &adap->sge.ofldtxq[r * 4];
		int n = min(4, adap->sge.ofldqsets - 4 * r);

		S("QType:", "TOE");
		T("TxQ ID:", q.cntxt_id);
		T("TxQ size:", q.size);
		T("TxQ inuse:", q.in_use);
		R("RspQ ID:", rspq.abs_id);
		R("RspQ size:", rspq.size);
		R("RspQE size:", rspq.iqe_len);
		S3("u", "Intr delay:", qtimer_val(adap, &rx[i].rspq));
		S3("u", "Intr pktcnt:",
		   adap->sge.counter_val[rx[i].rspq.pktcnt_idx]);
		R("FL ID:", fl.cntxt_id);
		R("FL size:", fl.size - 8);
		R("FL avail:", fl.avail);
	} else if ((r -= toe_entries) < rdma_entries) {
		const struct sge_ofld_rxq *rx = &adap->sge.rdmarxq[r * 4];
		int n = min(4, adap->sge.rdmaqs - 4 * r);

		S("QType:", "RDMA");
		R("RspQ ID:", rspq.abs_id);
		R("RspQ size:", rspq.size);
		R("RspQE size:", rspq.iqe_len);
		S3("u", "Intr delay:", qtimer_val(adap, &rx[i].rspq));
		S3("u", "Intr pktcnt:",
		   adap->sge.counter_val[rx[i].rspq.pktcnt_idx]);
		R("FL ID:", fl.cntxt_id);
		R("FL size:", fl.size - 8);
		R("FL avail:", fl.avail);
	} else if ((r -= rdma_entries) < iscsi_entries) {
		const struct sge_ofld_rxq *rx = &adap->sge.iscsirxq[r * 4];
		int n = min(4, adap->sge.niscsiq - 4 * r);

		S("QType:", "iSCSI");
		R("RspQ ID:", rspq.abs_id);
		R("RspQ size:", rspq.size);
		R("RspQE size:", rspq.iqe_len);
		S3("u", "Intr delay:", qtimer_val(adap, &rx[i].rspq));
		S3("u", "Intr pktcnt:",
		   adap->sge.counter_val[rx[i].rspq.pktcnt_idx]);
		R("FL ID:", fl.cntxt_id);
		R("FL size:", fl.size - 8);
		R("FL avail:", fl.avail);
	} else if ((r -= iscsi_entries) < ctrl_entries) {
		const struct sge_ctrl_txq *tx = &adap->sge.ctrlq[r * 4];
		int n = min(4, adap->params.nports - 4 * r);

		S("QType:", "Control");
		T("TxQ ID:", q.cntxt_id);
		T("TxQ size:", q.size);
		T("TxQ inuse:", q.in_use);
	} else if ((r -= ctrl_entries) == 0) {
		const struct sge_rspq *evtq = &adap->sge.fw_evtq;

		seq_printf(seq, "%-12s %16s\n", "QType:", "FW event queue");
		seq_printf(seq, "%-12s %16u\n", "RspQ ID:", evtq->abs_id);
		seq_printf(seq, "%-12s %16u\n", "Intr delay:",
			   qtimer_val(adap, evtq));
		seq_printf(seq, "%-12s %16u\n", "Intr pktcnt:",
			   adap->sge.counter_val[evtq->pktcnt_idx]);
	}
#undef R
#undef T
#undef S
#undef S3
	return 0;
}

static int sge_queue_entries(const struct adapter *adap)
{
	return DIV_ROUND_UP(adap->sge.ethqsets, 4) +
	       DIV_ROUND_UP(adap->sge.ofldqsets, 4) +
	       DIV_ROUND_UP(adap->sge.rdmaqs, 4) +
	       DIV_ROUND_UP(adap->sge.niscsiq, 4) +
	       DIV_ROUND_UP(MAX_CTRL_QUEUES, 4) + 1;
}

static void *sge_queue_start(struct seq_file *seq, loff_t *pos)
{
	int entries = sge_queue_entries(seq->private);

	return *pos < entries ? (void *)((uintptr_t)*pos + 1) : NULL;
}

static void sge_queue_stop(struct seq_file *seq, void *v)
{
}

static void *sge_queue_next(struct seq_file *seq, void *v, loff_t *pos)
{
	int entries = sge_queue_entries(seq->private);

	++*pos;
	return *pos < entries ? (void *)((uintptr_t)*pos + 1) : NULL;
}

static const struct seq_operations sge_qinfo_seq_ops = {
	.start = sge_queue_start,
	.next  = sge_queue_next,
	.stop  = sge_queue_stop,
	.show  = sge_qinfo_show
};

static int sge_qinfo_open(struct inode *inode, struct file *file)
{
	int res = seq_open(file, &sge_qinfo_seq_ops);

	if (!res) {
		struct seq_file *seq = file->private_data;
		seq->private = inode->i_private;
	}
	return res;
}

static const struct file_operations sge_qinfo_debugfs_fops = {
	.owner   = THIS_MODULE,
	.open    = sge_qinfo_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release,
};

static int blocked_fl_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t blocked_fl_read(struct file *filp, char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	int len;
	const struct adapter *adap = filp->private_data;
	char buf[(MAX_EGRQ + 3) / 4 + MAX_EGRQ / 32 + 2]; /* includes ,/\n/\0 */

	len = bitmap_scnprintf(buf, sizeof(buf) - 1, adap->sge.blocked_fl,
			       MAX_EGRQ);
	len += sprintf(buf + len, "\n");
	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static ssize_t blocked_fl_write(struct file *filp, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	int err;
	DECLARE_BITMAP(t, MAX_EGRQ);
	struct adapter *adap = filp->private_data;

	err = bitmap_parse_user(ubuf, count, t, MAX_EGRQ);
	if (err)
		return err;

	bitmap_copy(adap->sge.blocked_fl, t, MAX_EGRQ);
	return count;
}

static const struct file_operations blocked_fl_fops = {
	.owner   = THIS_MODULE,
	.open    = blocked_fl_open,
	.read    = blocked_fl_read,
	.write   = blocked_fl_write,
	.llseek  = generic_file_llseek,
};

#define DMABUF 1
#if DMABUF

#define DMABUF_SZ (64 * 1024)

static void *dma_virt;
static dma_addr_t dma_phys;

static ssize_t dma_read(struct file *file, char __user *buf, size_t count,
			loff_t *ppos)
{
	return simple_read_from_buffer(buf, count, ppos, dma_virt, DMABUF_SZ);
}

static ssize_t dma_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	loff_t pos = *ppos;
	size_t avail = DMABUF_SZ;

	file->f_path.dentry->d_inode->i_size = avail;
	if (pos < 0)
		return -EINVAL;
	if (pos >= avail)
		return 0;
	if (count > avail - pos)
		count = avail - pos;
	if (copy_from_user(dma_virt + pos, buf, count))
		return -EFAULT;
	*ppos = pos + count;
	return count;
}

static const struct file_operations dma_debugfs_fops = {
	.owner   = THIS_MODULE,
	.open    = mem_open,
	.read    = dma_read,
	.write   = dma_write
};
#endif

#ifdef T4_TRACE
static void __devinit alloc_trace_bufs(struct adapter *adap)
{
	int i;
	char s[32];

	for (i = 0; i < ARRAY_SIZE(adap->tb); ++i) {
		sprintf(s, "sge_q%d", i);
		adap->tb[i] = t4_trace_alloc(adap->debugfs_root, s, 512);
	}
}

static void free_trace_bufs(struct adapter *adap)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adap->tb); ++i)
		t4_trace_free(adap->tb[i]);
}
#else
# define alloc_trace_bufs(adapter)
# define free_trace_bufs(adapter)
#endif

static void __devinit set_debugfs_file_size(struct dentry *de, loff_t size)
{
	if (!IS_ERR(de) && de->d_inode)
		de->d_inode->i_size = size;
}

static void __devinit add_debugfs_mem(struct adapter *adap, const char *name,
				      unsigned int idx, unsigned int size_mb)
{
	struct dentry *de;

	de = debugfs_create_file(name, S_IRUSR, adap->debugfs_root,
				 (void *)adap + idx, &mem_debugfs_fops);
	set_debugfs_file_size(de, size_mb << 20);
}

struct cxgb4_debugfs_entry {
	const char *name;
	const struct file_operations *ops;
	mode_t mode;
	unsigned char data;
};

static int __devinit setup_debugfs(struct adapter *adap)
{
	static struct cxgb4_debugfs_entry debugfs_files[] = {
		{ "blocked_fl", &blocked_fl_fops, S_IRUSR | S_IWUSR, 0 },
		{ "cim_la", &cim_la_fops, S_IRUSR, 0 },
		{ "cim_pif_la", &cim_pif_la_fops, S_IRUSR, 0 },
		{ "cim_ma_la", &cim_ma_la_fops, S_IRUSR, 0 },
		{ "cim_qcfg", &cim_qcfg_fops, S_IRUSR, 0 },
		{ "clk", &clk_debugfs_fops, S_IRUSR, 0 },
		{ "devlog", &devlog_fops, S_IRUSR, 0 },
		{ "mbox0", &mbox_debugfs_fops, S_IRUSR | S_IWUSR, 0 },
		{ "mbox1", &mbox_debugfs_fops, S_IRUSR | S_IWUSR, 1 },
		{ "mbox2", &mbox_debugfs_fops, S_IRUSR | S_IWUSR, 2 },
		{ "mbox3", &mbox_debugfs_fops, S_IRUSR | S_IWUSR, 3 },
		{ "mbox4", &mbox_debugfs_fops, S_IRUSR | S_IWUSR, 4 },
		{ "mbox5", &mbox_debugfs_fops, S_IRUSR | S_IWUSR, 5 },
		{ "mbox6", &mbox_debugfs_fops, S_IRUSR | S_IWUSR, 6 },
		{ "mbox7", &mbox_debugfs_fops, S_IRUSR | S_IWUSR, 7 },
		{ "mps_tcam", &mps_tcam_debugfs_fops, S_IRUSR, 0 },
		{ "ibq_tp0",  &cim_ibq_fops, S_IRUSR, 0 },
		{ "ibq_tp1",  &cim_ibq_fops, S_IRUSR, 1 },
		{ "ibq_ulp",  &cim_ibq_fops, S_IRUSR, 2 },
		{ "ibq_sge0", &cim_ibq_fops, S_IRUSR, 3 },
		{ "ibq_sge1", &cim_ibq_fops, S_IRUSR, 4 },
		{ "ibq_ncsi", &cim_ibq_fops, S_IRUSR, 5 },
		{ "obq_ulp0", &cim_obq_fops, S_IRUSR, 0 },
		{ "obq_ulp1", &cim_obq_fops, S_IRUSR, 1 },
		{ "obq_ulp2", &cim_obq_fops, S_IRUSR, 2 },
		{ "obq_ulp3", &cim_obq_fops, S_IRUSR, 3 },
		{ "obq_sge",  &cim_obq_fops, S_IRUSR, 4 },
		{ "obq_ncsi", &cim_obq_fops, S_IRUSR, 5 },
		{ "sge_qinfo", &sge_qinfo_debugfs_fops, S_IRUSR, 0 },
		{ "tp_la", &tp_la_fops, S_IRUSR, 0 },
		{ "ulprx_la", &ulprx_la_fops, S_IRUSR, 0 },
	};

	int i;
	struct dentry *de;

	if (!adap->debugfs_root)
		return -1;

	/* debugfs support is best effort */
	for (i = 0; i < ARRAY_SIZE(debugfs_files); i++)
		de = debugfs_create_file(debugfs_files[i].name,
				debugfs_files[i].mode, adap->debugfs_root,
				(void *)adap + debugfs_files[i].data,
				debugfs_files[i].ops);

	i = t4_read_reg(adap, A_MA_TARGET_MEM_ENABLE);
	if (i & F_EDRAM0_ENABLE)
		add_debugfs_mem(adap, "edc0", MEM_EDC0, 5);
	if (i & F_EDRAM1_ENABLE)
		add_debugfs_mem(adap, "edc1", MEM_EDC1, 5);
	if (i & F_EXT_MEM_ENABLE)
		add_debugfs_mem(adap, "mc", MEM_MC,
			G_EXT_MEM_SIZE(t4_read_reg(adap, A_MA_EXT_MEMORY_BAR)));

	de = debugfs_create_file("flash", S_IRUSR, adap->debugfs_root, adap,
				 &flash_debugfs_fops);
	set_debugfs_file_size(de, adap->params.sf_size);

#if DMABUF
	dma_virt = dma_alloc_coherent(adap->pdev_dev, DMABUF_SZ, &dma_phys,
				      GFP_KERNEL);
	if (dma_virt) {
		printk("DMA buffer at bus address %#llx, virtual 0x%p\n",
			(unsigned long long)dma_phys, dma_virt);
		de = debugfs_create_file("dmabuf", 0644, adap->debugfs_root,
					 adap, &dma_debugfs_fops);
		set_debugfs_file_size(de, DMABUF_SZ);
	}
#endif

	alloc_trace_bufs(adap);
	return 0;
}

/*
 * /proc support
 */

#define DEFINE_SIMPLE_PROC_FILE(name) \
static int name##_open(struct inode *inode, struct file *file) \
{ \
	return single_open(file, name##_show, PDE(inode)->data); \
} \
static const struct file_operations name##_proc_fops = { \
	.owner   = THIS_MODULE, \
	.open    = name##_open, \
	.read    = seq_read, \
	.llseek  = seq_lseek, \
	.release = single_release \
}

static int sge_stats_show(struct seq_file *seq, void *v)
{
	struct adapter *adap = seq->private;
	int eth_entries = DIV_ROUND_UP(adap->sge.ethqsets, 4);
	int toe_entries = DIV_ROUND_UP(adap->sge.ofldqsets, 4);
	int rdma_entries = DIV_ROUND_UP(adap->sge.rdmaqs, 4);
	int iscsi_entries = DIV_ROUND_UP(adap->sge.niscsiq, 4);
	int ctrl_entries = DIV_ROUND_UP(MAX_CTRL_QUEUES, 4);
	int i, r = (uintptr_t)v - 1;

	if (r)
		seq_putc(seq, '\n');

#define S3(fmt_spec, s, v) \
	seq_printf(seq, "%-12s", s); \
	for (i = 0; i < n; ++i) \
		seq_printf(seq, " %16" fmt_spec, v); \
		seq_putc(seq, '\n');
#define S(s, v) S3("s", s, v)
#define T3(fmt_spec, s, v) S3(fmt_spec, s, tx[i].v)
#define T(s, v) T3("lu", s, v)
#define R3(fmt_spec, s, v) S3(fmt_spec, s, qs[i].v)
#define R(s, v) R3("lu", s, v)

	if (r < eth_entries) {
		const struct sge_eth_rxq *qs = &adap->sge.ethrxq[r * 4];
		const struct sge_eth_txq *tx = &adap->sge.ethtxq[r * 4];
		int n = min(4, adap->sge.ethqsets - 4 * r);

		S("QType:", "Ethernet");
		S("Interface:",
		  qs[i].rspq.netdev ? qs[i].rspq.netdev->name : "N/A");
		R("RxPackets:", stats.pkts);
		R("RxCSO:", stats.rx_cso);
		R("VLANxtract:", stats.vlan_ex);
		R("LROmerged:", stats.lro_merged);
		R("LROpackets:", stats.lro_pkts);
		R("RxDrops:", stats.rx_drops);
		T("TSO:", tso);
		T("TxCSO:", tx_cso);
		T("VLANins:", vlan_ins);
		T("TxQFull:", q.stops);
		T("TxQRestarts:", q.restarts);
		T("TxMapErr:", mapping_err);
		T("TxCoalWR:", coal_wr);
		T("TxCoalPkt:", coal_pkts);
		R("FLAllocErr:", fl.alloc_failed);
		R("FLLrgAlcErr:", fl.large_alloc_failed);
		R("FLstarving:", fl.starving);
	} else if ((r -= eth_entries) < toe_entries) {
		const struct sge_ofld_rxq *qs = &adap->sge.ofldrxq[r * 4];
		const struct sge_ofld_txq *tx = &adap->sge.ofldtxq[r * 4];
		int n = min(4, adap->sge.ofldqsets - 4 * r);

		S("QType:", "TOE");
		R("RxPackets:", stats.pkts);
		R("RxImmPkts:", stats.imm);
		R("RxNoMem:", stats.nomem);
		T("TxQFull:", q.stops);
		T("TxQRestarts:", q.restarts);
		T("TxMapErr:", mapping_err);
		R("FLAllocErr:", fl.alloc_failed);
		R("FLLrgAlcErr:", fl.large_alloc_failed);
		R("FLstarving:", fl.starving);
	} else if ((r -= toe_entries) < rdma_entries) {
		const struct sge_ofld_rxq *qs = &adap->sge.rdmarxq[r * 4];
		int n = min(4, adap->sge.rdmaqs - 4 * r);

		S("QType:", "RDMA");
		R("RxPackets:", stats.pkts);
		R("RxImmPkts:", stats.imm);
		R("RxAN:", stats.an);
		R("RxNoMem:", stats.nomem);
		R("FLAllocErr:", fl.alloc_failed);
		R("FLstarving:", fl.starving);
	} else if ((r -= rdma_entries) < iscsi_entries) {
		const struct sge_ofld_rxq *qs = &adap->sge.iscsirxq[r * 4];
		int n = min(4, adap->sge.niscsiq - 4 * r);

		S("QType:", "iSCSI");
		R("RxPackets:", stats.pkts);
		R("RxImmPkts:", stats.imm);
		R("RxNoMem:", stats.nomem);
		R("FLAllocErr:", fl.alloc_failed);
		R("FLstarving:", fl.starving);
	} else if ((r -= iscsi_entries) < ctrl_entries) {
		const struct sge_ctrl_txq *tx = &adap->sge.ctrlq[r * 4];
		int n = min(4, MAX_CTRL_QUEUES - 4 * r);

		S("QType:", "Control");
		T("TxQFull:", q.stops);
		T("TxQRestarts:", q.restarts);
	} else if ((r -= ctrl_entries) == 0) {
		seq_printf(seq, "%-12s %16s\n", "QType:", "FW event queue");
	}
#undef R
#undef T
#undef S
#undef R3
#undef T3
#undef S3
	return 0;
}

static const struct seq_operations sge_stats_seq_ops = {
	.start = sge_queue_start,
	.next  = sge_queue_next,
	.stop  = sge_queue_stop,
	.show  = sge_stats_show
};

static int sge_stats_open(struct inode *inode, struct file *file)
{
	int res = seq_open(file, &sge_stats_seq_ops);

	if (!res) {
		struct seq_file *seq = file->private_data;
		seq->private = PDE(inode)->data;
	}
	return res;
}

static const struct file_operations sge_stats_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = sge_stats_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release,
};

static int sched_show(struct seq_file *seq, void *v)
{
	int i;
	unsigned int map, kbps, ipg, mode;
	unsigned int pace_tab[NTX_SCHED];
	struct adapter *adap = seq->private;

	map = t4_read_reg(adap, A_TP_TX_MOD_QUEUE_REQ_MAP);
	mode = G_TIMERMODE(t4_read_reg(adap, A_TP_MOD_CONFIG));
	t4_read_pace_tbl(adap, pace_tab);

	seq_printf(seq, "Scheduler  Mode   Channel  Rate (Kbps)   "
		      "Class IPG (0.1 ns)   Flow IPG (us)\n");
	for (i = 0; i < NTX_SCHED; ++i, map >>= 2) {
		t4_get_tx_sched(adap, i, &kbps, &ipg);
		seq_printf(seq, "    %u      %-5s     %u     ", i,
			   (mode & (1 << i)) ? "flow" : "class", map & 3);
		if (kbps)
			seq_printf(seq, "%9u     ", kbps);
		else
			seq_puts(seq, " disabled     ");

		if (ipg)
			seq_printf(seq, "%13u        ", ipg);
		else
			seq_puts(seq, "     disabled        ");

		if (pace_tab[i])
			seq_printf(seq, "%10u\n", pace_tab[i]);
		else
			seq_puts(seq, "  disabled\n");
	}
	return 0;
}

static int sched_open(struct inode *inode, struct file *file)
{
	return single_open(file, sched_show, PDE(inode)->data);
}

static const struct file_operations sched_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = sched_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int lb_stats_show(struct seq_file *seq, void *v)
{
	static const char *stat_name[] = {
		"OctetsOK:", "FramesOK:", "BcastFrames:", "McastFrames:",
		"UcastFrames:", "ErrorFrames:", "Frames64:", "Frames65To127:",
		"Frames128To255:", "Frames256To511:", "Frames512To1023:",
		"Frames1024To1518:", "Frames1519ToMax:", "FramesDropped:",
		"BG0FramesDropped:", "BG1FramesDropped:", "BG2FramesDropped:",
		"BG3FramesDropped:", "BG0FramesTrunc:", "BG1FramesTrunc:",
		"BG2FramesTrunc:", "BG3FramesTrunc:"
	};

	int i, j;
	u64 *p0, *p1;
	struct lb_port_stats s[2];

	memset(s, 0, sizeof(s));

	for (i = 0; i < 4; i += 2) {
		t4_get_lb_stats(seq->private, i, &s[0]);
		t4_get_lb_stats(seq->private, i + 1, &s[1]);

		p0 = &s[0].octets;
		p1 = &s[1].octets;
		seq_printf(seq, "%s                       Loopback %u          "
			   " Loopback %u\n", i == 0 ? "" : "\n", i, i + 1);

		for (j = 0; j < ARRAY_SIZE(stat_name); j++)
			seq_printf(seq, "%-17s %20llu %20llu\n", stat_name[j],
				   (unsigned long long)*p0++,
				   (unsigned long long)*p1++);
	}
	return 0;
}

DEFINE_SIMPLE_PROC_FILE(lb_stats);

static int pm_stats_show(struct seq_file *seq, void *v)
{
	static const char *pm_stats[] = {
		"Read:", "Write bypass:", "Write mem:", "Flush:", "FIFO wait:"
	};

	int i;
	u32 tx_cnt[PM_NSTATS], rx_cnt[PM_NSTATS];
	u64 tx_cyc[PM_NSTATS], rx_cyc[PM_NSTATS];
	struct adapter *adap = seq->private;

	t4_pmtx_get_stats(adap, tx_cnt, tx_cyc);
	t4_pmrx_get_stats(adap, rx_cnt, rx_cyc);

	seq_puts(seq, "                Tx count            Tx cycles    "
		 "Rx count            Rx cycles\n");
	for (i = 0; i < PM_NSTATS; i++)
		seq_printf(seq, "%-13s %10u %20llu  %10u %20llu\n",
			   pm_stats[i], tx_cnt[i],
			   (unsigned long long)tx_cyc[i], rx_cnt[i],
			   (unsigned long long)rx_cyc[i]);
	return 0;
}

static int pm_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, pm_stats_show, PDE(inode)->data);
}

static ssize_t pm_stats_clear(struct file *file, const char __user *buf,
			      size_t count, loff_t *pos)
{
	struct adapter *adap = PDE(file->f_path.dentry->d_inode)->data;

	t4_write_reg(adap, A_PM_RX_STAT_CONFIG, 0);
	t4_write_reg(adap, A_PM_TX_STAT_CONFIG, 0);
	return count;
}

static const struct file_operations pm_stats_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = pm_stats_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
	.write   = pm_stats_clear
};

static int tcp_stats_show(struct seq_file *seq, void *v)
{
	struct tp_tcp_stats v4, v6;
	struct adapter *adap = seq->private;

	spin_lock(&adap->stats_lock);
	t4_tp_get_tcp_stats(adap, &v4, &v6);
	spin_unlock(&adap->stats_lock);

	seq_puts(seq,
		 "                                IP                 IPv6\n");
	seq_printf(seq, "OutRsts:      %20u %20u\n",
		   v4.tcpOutRsts, v6.tcpOutRsts);
	seq_printf(seq, "InSegs:       %20llu %20llu\n",
		   (unsigned long long)v4.tcpInSegs,
		   (unsigned long long)v6.tcpInSegs);
	seq_printf(seq, "OutSegs:      %20llu %20llu\n",
		   (unsigned long long)v4.tcpOutSegs,
		   (unsigned long long)v6.tcpOutSegs);
	seq_printf(seq, "RetransSegs:  %20llu %20llu\n",
		   (unsigned long long)v4.tcpRetransSegs,
		   (unsigned long long)v6.tcpRetransSegs);
	return 0;
}

DEFINE_SIMPLE_PROC_FILE(tcp_stats);

static int tp_err_stats_show(struct seq_file *seq, void *v)
{
	struct tp_err_stats stats;
	struct adapter *adap = seq->private;

	spin_lock(&adap->stats_lock);
	t4_tp_get_err_stats(adap, &stats);
	spin_unlock(&adap->stats_lock);

	seq_puts(seq, "                 channel 0  channel 1  channel 2  "
		      "channel 3\n");
	seq_printf(seq, "macInErrs:      %10u %10u %10u %10u\n",
		   stats.macInErrs[0], stats.macInErrs[1], stats.macInErrs[2],
		   stats.macInErrs[3]);
	seq_printf(seq, "hdrInErrs:      %10u %10u %10u %10u\n",
		   stats.hdrInErrs[0], stats.hdrInErrs[1], stats.hdrInErrs[2],
		   stats.hdrInErrs[3]);
	seq_printf(seq, "tcpInErrs:      %10u %10u %10u %10u\n",
		   stats.tcpInErrs[0], stats.tcpInErrs[1], stats.tcpInErrs[2],
		   stats.tcpInErrs[3]);
	seq_printf(seq, "tcp6InErrs:     %10u %10u %10u %10u\n",
		   stats.tcp6InErrs[0], stats.tcp6InErrs[1],
		   stats.tcp6InErrs[2], stats.tcp6InErrs[3]);
	seq_printf(seq, "tnlCongDrops:   %10u %10u %10u %10u\n",
		   stats.tnlCongDrops[0], stats.tnlCongDrops[1],
		   stats.tnlCongDrops[2], stats.tnlCongDrops[3]);
	seq_printf(seq, "tnlTxDrops:     %10u %10u %10u %10u\n",
		   stats.tnlTxDrops[0], stats.tnlTxDrops[1],
		   stats.tnlTxDrops[2], stats.tnlTxDrops[3]);
	seq_printf(seq, "ofldVlanDrops:  %10u %10u %10u %10u\n",
		   stats.ofldVlanDrops[0], stats.ofldVlanDrops[1],
		   stats.ofldVlanDrops[2], stats.ofldVlanDrops[3]);
	seq_printf(seq, "ofldChanDrops:  %10u %10u %10u %10u\n\n",
		   stats.ofldChanDrops[0], stats.ofldChanDrops[1],
		   stats.ofldChanDrops[2], stats.ofldChanDrops[3]);
	seq_printf(seq, "ofldNoNeigh:    %u\nofldCongDefer:  %u\n",
		   stats.ofldNoNeigh, stats.ofldCongDefer);
	return 0;
}

DEFINE_SIMPLE_PROC_FILE(tp_err_stats);

static int cpl_stats_show(struct seq_file *seq, void *v)
{
	struct tp_cpl_stats stats;
	struct adapter *adap = seq->private;

	spin_lock(&adap->stats_lock);
	t4_tp_get_cpl_stats(adap, &stats);
	spin_unlock(&adap->stats_lock);

	seq_puts(seq, "                 channel 0  channel 1  channel 2  "
		      "channel 3\n");
	seq_printf(seq, "CPL requests:   %10u %10u %10u %10u\n",
		   stats.req[0], stats.req[1], stats.req[2], stats.req[3]);
	seq_printf(seq, "CPL responses:  %10u %10u %10u %10u\n",
		   stats.rsp[0], stats.rsp[1], stats.rsp[2], stats.rsp[3]);
	return 0;
}

DEFINE_SIMPLE_PROC_FILE(cpl_stats);

static int rdma_stats_show(struct seq_file *seq, void *v)
{
	struct tp_rdma_stats stats;
	struct adapter *adap = seq->private;

	spin_lock(&adap->stats_lock);
	t4_tp_get_rdma_stats(adap, &stats);
	spin_unlock(&adap->stats_lock);

	seq_printf(seq, "NoRQEModDefferals: %u\n", stats.rqe_dfr_mod);
	seq_printf(seq, "NoRQEPktDefferals: %u\n", stats.rqe_dfr_pkt);
	return 0;
}

DEFINE_SIMPLE_PROC_FILE(rdma_stats);

static int fcoe_stats_show(struct seq_file *seq, void *v)
{
	struct tp_fcoe_stats stats[4];
	struct adapter *adap = seq->private;

	spin_lock(&adap->stats_lock);
	t4_get_fcoe_stats(adap, 0, &stats[0]);
	t4_get_fcoe_stats(adap, 1, &stats[1]);
	t4_get_fcoe_stats(adap, 2, &stats[2]);
	t4_get_fcoe_stats(adap, 3, &stats[3]);
	spin_unlock(&adap->stats_lock);

	seq_puts(seq, "                   channel 0        channel 1        "
		      "channel 2        channel 3\n");
	seq_printf(seq, "octetsDDP:  %16llu %16llu %16llu %16llu\n",
		   stats[0].octetsDDP, stats[1].octetsDDP, stats[2].octetsDDP,
		   stats[3].octetsDDP);
	seq_printf(seq, "framesDDP:  %16u %16u %16u %16u\n", stats[0].framesDDP,
		   stats[1].framesDDP, stats[2].framesDDP, stats[3].framesDDP);
	seq_printf(seq, "framesDrop: %16u %16u %16u %16u\n",
		   stats[0].framesDrop, stats[1].framesDrop,
		   stats[2].framesDrop, stats[3].framesDrop);
	return 0;
}

DEFINE_SIMPLE_PROC_FILE(fcoe_stats);

static int ddp_stats_show(struct seq_file *seq, void *v)
{
	struct tp_usm_stats stats;
	struct adapter *adap = seq->private;

	spin_lock(&adap->stats_lock);
	t4_get_usm_stats(adap, &stats);
	spin_unlock(&adap->stats_lock);

	seq_printf(seq, "Frames: %u\n", stats.frames);
	seq_printf(seq, "Octets: %llu\n", (unsigned long long)stats.octets);
	seq_printf(seq, "Drops:  %u\n", stats.drops);
	return 0;
}

DEFINE_SIMPLE_PROC_FILE(ddp_stats);

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
/*
 * Filter Table.
 */

/*
 * Global filter infor used to decode individual filter entries.  This is
 * grabbed once when we open the /proc filters node to save endlessly
 * rereading the registers.
 */
struct filters_global_info {
	struct adapter *adapter;
	u32 tp_vlan_pri_map;
	u32 tp_ingress_config;
};

static void filters_show_ipaddr(struct seq_file *seq,
				int type, u8 *addr, u8 *addrm)
{
	int noctets, octet;

	seq_puts(seq, " ");
	if (type == 0) {
		noctets = 4;
		seq_printf(seq, "%48s", " ");
	} else
		noctets = 16;

	for (octet = 0; octet < noctets; octet++)
		seq_printf(seq, "%02x", addr[octet]);
	seq_puts(seq, "/");
	for (octet = 0; octet < noctets; octet++)
		seq_printf(seq, "%02x", addrm[octet]);
}

static int filters_show(struct seq_file *seq, void *v)
{
	struct filters_global_info *gconf = seq->private;
	u32 fconf = gconf->tp_vlan_pri_map;
	u32 tpiconf = gconf->tp_ingress_config;
	int i;

	if (v == SEQ_START_TOKEN) {
		seq_puts(seq, " Idx");
		for (i = TP_VLAN_PRI_MAP_FIRST; i <= TP_VLAN_PRI_MAP_LAST; i++) {
			switch (fconf & (1 << i)) {
			    case 0:
				/* compressed filter field not enabled */
				break;

			    case F_FCOE:
				seq_puts(seq, " FCoE");
				break;

			    case F_PORT:
				seq_puts(seq, " Port");
				break;

			    case F_VNIC_ID:
				if ((tpiconf & F_VNIC) == 0)
					seq_puts(seq, "     vld:oVLAN");
				else
					seq_puts(seq, "   VFvld:PF:VF");
				break;

			    case F_VLAN:
				seq_puts(seq, "     vld:iVLAN");
				break;

			    case F_TOS:
				seq_puts(seq, "   TOS");
				break;

			    case F_PROTOCOL:
				seq_puts(seq, "  Prot");
				break;

			    case F_ETHERTYPE:
				seq_puts(seq, "   EthType");
				break;

			    case F_MACMATCH:
				seq_puts(seq, "  MACIdx");
				break;

			    case F_MPSHITTYPE:
				seq_puts(seq, " MPS");
				break;

			    case F_FRAGMENTATION:
				seq_puts(seq, " Frag");
				break;
			}
		}
		seq_printf(seq, " %65s %65s %9s %9s %s\n",
			   "LIP", "FIP", "LPORT", "FPORT", "Action");
	} else {
		int fidx = (uintptr_t)v - 2;
		struct filter_entry *f = &gconf->adapter->tids.ftid_tab[fidx];

		/* if this entry isn't filled in just return */
		if (!f->valid)
			return 0;

		/*
		 * Filter index.
		 */
		seq_printf(seq, "%4d", fidx);

		/*
		 * Compressed header portion of filter.
		 */
		for (i = TP_VLAN_PRI_MAP_FIRST; i <= TP_VLAN_PRI_MAP_LAST; i++) {
			switch (fconf & (1 << i)) {
			    case 0:
				/* compressed filter field not enabled */
				break;

			    case F_FCOE:
				seq_printf(seq, "  %1d/%1d",
					   f->fs.val.fcoe, f->fs.mask.fcoe);
				break;

			    case F_PORT:
				seq_printf(seq, "  %1d/%1d",
					   f->fs.val.iport, f->fs.mask.iport);
				break;

			    case F_VNIC_ID:
				if ((tpiconf & F_VNIC) == 0)
				  seq_printf(seq, " %1d:%04x/%1d:%04x",
					     f->fs.val.ovlan_vld,
					     f->fs.val.ovlan,
					     f->fs.mask.ovlan_vld,
					     f->fs.mask.ovlan);
				else
				  seq_printf(seq, " %1d:%1x:%02x/%1d:%1x:%02x",
					     f->fs.val.ovlan_vld,
					     (f->fs.val.ovlan >> 7) & 0x7,
					     f->fs.val.ovlan & 0x7f,
					     f->fs.mask.ovlan_vld,
					     (f->fs.mask.ovlan >> 7) & 0x7,
					     f->fs.mask.ovlan & 0x7f);
				break;

			    case F_VLAN:
				seq_printf(seq, " %1d:%04x/%1d:%04x",
					   f->fs.val.ivlan_vld,
					   f->fs.val.ivlan,
					   f->fs.mask.ivlan_vld,
					   f->fs.mask.ivlan);
				break;

			    case F_TOS:
				seq_printf(seq, " %02x/%02x",
					   f->fs.val.tos, f->fs.mask.tos);
				break;

			    case F_PROTOCOL:
				seq_printf(seq, " %02x/%02x",
					   f->fs.val.proto, f->fs.mask.proto);
				break;

			    case F_ETHERTYPE:
				seq_printf(seq, " %04x/%04x",
					   f->fs.val.ethtype, f->fs.mask.ethtype);
				break;

			    case F_MACMATCH:
				seq_printf(seq, " %03x/%03x",
					   f->fs.val.macidx, f->fs.mask.macidx);
				break;

			    case F_MPSHITTYPE:
				seq_printf(seq, " %1x/%1x",
					   f->fs.val.matchtype,
					   f->fs.mask.matchtype);
				break;

			    case F_FRAGMENTATION:
				seq_printf(seq, "  %1d/%1d",
					   f->fs.val.frag, f->fs.mask.frag);
				break;
			}
		}

		/*
		 * Fixed portion of filter.
		 */
		filters_show_ipaddr(seq, f->fs.type,
				    f->fs.val.lip, f->fs.mask.lip);
		filters_show_ipaddr(seq, f->fs.type,
				    f->fs.val.fip, f->fs.mask.fip);
		seq_printf(seq, " %04x/%04x %04x/%04x",
			   f->fs.val.lport, f->fs.mask.lport,
			   f->fs.val.fport, f->fs.mask.fport);

		/*
		 * Variable length filter action.
		 */
		if (f->fs.action == FILTER_DROP)
			seq_puts(seq, " Drop");
		else if (f->fs.action == FILTER_SWITCH) {
			seq_printf(seq, " Switch: port=%d", f->fs.eport);
			if (f->fs.newdmac)
				seq_printf(seq,
					   ", dmac=%02x:%02x:%02x:%02x:%02x:%02x"
					   ", l2tidx=%d",
					   f->fs.dmac[0], f->fs.dmac[1],
					   f->fs.dmac[2], f->fs.dmac[3],
					   f->fs.dmac[4], f->fs.dmac[5],
					   f->l2t->idx);
			if (f->fs.newsmac)
				seq_printf(seq,
					   ", smac=%02x:%02x:%02x:%02x:%02x:%02x"
					   ", smtidx=%d",
					   f->fs.smac[0], f->fs.smac[1],
					   f->fs.smac[2], f->fs.smac[3],
					   f->fs.smac[4], f->fs.smac[5],
					   f->smtidx);
			if (f->fs.newvlan == VLAN_REMOVE)
				seq_printf(seq, ", vlan=none");
			else if (f->fs.newvlan == VLAN_INSERT)
				seq_printf(seq, ", vlan=insert(%x)",
					   f->fs.vlan);
			else if (f->fs.newvlan == VLAN_REWRITE)
				seq_printf(seq, ", vlan=rewrite(%x)",
					   f->fs.vlan);
		} else {
			seq_puts(seq, " Pass: Q=");
			if (f->fs.dirsteer == 0) {
				seq_puts(seq, "RSS");
				if (f->fs.maskhash)
					seq_puts(seq, "(TCB=hash)");
			} else {
				seq_printf(seq, "%d", f->fs.iq);
				if (f->fs.dirsteerhash == 0)
					seq_puts(seq, "(QID)");
				else
					seq_puts(seq, "(hash)");
			}
		}
		if (f->fs.prio)
			seq_puts(seq, " Prio");
		if (f->fs.rpttid)
			seq_puts(seq, " RptTID");
		seq_puts(seq, "\n");
	}
	return 0;
}

static inline void *filters_get_idx(struct adapter *adapter, loff_t pos)
{
	if (pos > adapter->tids.nftids)
		return NULL;

	return (void *)(uintptr_t)(pos + 1);
}

static void *filters_start(struct seq_file *seq, loff_t *pos)
{
	struct filters_global_info *gconf = seq->private;

	return (*pos
		? filters_get_idx(gconf->adapter, *pos)
		: SEQ_START_TOKEN);
}

static void *filters_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct filters_global_info *gconf = seq->private;

	(*pos)++;
	return filters_get_idx(gconf->adapter, *pos);
}

static void filters_stop(struct seq_file *seq, void *v)
{
}

static const struct seq_operations filters_seq_ops = {
	.start = filters_start,
	.next  = filters_next,
	.stop  = filters_stop,
	.show  = filters_show
};

static int filters_open(struct inode *inode, struct file *file)
{
	struct adapter *adapter = PDE(inode)->data;
	struct filters_global_info *gconf =
		__seq_open_private(file, &filters_seq_ops, sizeof *gconf);

	if (gconf == NULL)
		return -ENOMEM;
	if (adapter->tids.nftids == 0 ||
	    adapter->tids.ftid_tab == NULL)
		return -EOPNOTSUPP;

	gconf->adapter = adapter;
	t4_read_indirect(adapter, A_TP_PIO_ADDR, A_TP_PIO_DATA,
			 &gconf->tp_vlan_pri_map, 1,
			 A_TP_VLAN_PRI_MAP);
	t4_read_indirect(adapter, A_TP_PIO_ADDR, A_TP_PIO_DATA,
			 &gconf->tp_ingress_config, 1,
			 A_TP_INGRESS_CONFIG);

	return 0;
}

static const struct file_operations filters_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = filters_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};
#endif

static int tx_rate_show(struct seq_file *seq, void *v)
{
	u64 nrate[NCHAN], orate[NCHAN];
	struct adapter *adap = seq->private;

	t4_get_chan_txrate(adap, nrate, orate);
	seq_puts(seq, "              channel 0   channel 1   channel 2   "
		 "channel 3\n");
	seq_printf(seq, "NIC B/s:     %10llu  %10llu  %10llu  %10llu\n",
		   (unsigned long long)nrate[0], (unsigned long long)nrate[1],
		   (unsigned long long)nrate[2], (unsigned long long)nrate[3]);
	seq_printf(seq, "Offload B/s: %10llu  %10llu  %10llu  %10llu\n",
		   (unsigned long long)orate[0], (unsigned long long)orate[1],
		   (unsigned long long)orate[2], (unsigned long long)orate[3]);
	return 0;
}

DEFINE_SIMPLE_PROC_FILE(tx_rate);

/*
 * RSS Table.
 */

static int rss_show(struct seq_file *seq, void *v, int idx)
{
	u16 *entry = v;

	seq_printf(seq, "%4d:  %4u  %4u  %4u  %4u  %4u  %4u  %4u  %4u\n",
		   idx * 8, entry[0], entry[1], entry[2], entry[3], entry[4],
		   entry[5], entry[6], entry[7]);
	return 0;
}

static int rss_open(struct inode *inode, struct file *file)
{
	int ret;
	struct seq_tab *p;

	p = seq_open_tab(file, RSS_NENTRIES / 8, 8 * sizeof(u16), 0, rss_show);
	if (!p)
		return -ENOMEM;
	ret = t4_read_rss(PDE(inode)->data, (u16 *)p->data);
	if (ret)
		seq_release_private(inode, file);
	return ret;
}

static const struct file_operations rss_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = rss_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};

/*
 * RSS Configuration.
 */

/*
 * Small utility function to return the strings "yes" or "no" if the supplied
 * argument is non-zero.
 */
static const char *yesno(int x)
{
	static const char *yes = "yes";
	static const char *no = "no";
	return x ? yes : no;
}

static int rss_config_show(struct seq_file *seq, void *v)
{
	struct adapter *adapter = seq->private;
	static const char *keymode[] = {
		"global",
		"global and per-VF scramble",
		"per-PF and per-VF scramble",
		"per-VF and per-VF scramble",
	};
	u32 rssconf;

	rssconf = t4_read_reg(adapter, A_TP_RSS_CONFIG);
	seq_printf(seq, "TP_RSS_CONFIG: %#x\n", rssconf);
	seq_printf(seq, "  Tnl4TupEnIpv6: %3s\n", yesno(rssconf & F_TNL4TUPENIPV6));
	seq_printf(seq, "  Tnl2TupEnIpv6: %3s\n", yesno(rssconf & F_TNL2TUPENIPV6));
	seq_printf(seq, "  Tnl4TupEnIpv4: %3s\n", yesno(rssconf & F_TNL4TUPENIPV4));
	seq_printf(seq, "  Tnl2TupEnIpv4: %3s\n", yesno(rssconf & F_TNL2TUPENIPV4));
	seq_printf(seq, "  TnlTcpSel:     %3s\n", yesno(rssconf & F_TNLTCPSEL));
	seq_printf(seq, "  TnlIp6Sel:     %3s\n", yesno(rssconf & F_TNLIP6SEL));
	seq_printf(seq, "  TnlVrtSel:     %3s\n", yesno(rssconf & F_TNLVRTSEL));
	seq_printf(seq, "  TnlMapEn:      %3s\n", yesno(rssconf & F_TNLMAPEN));
	seq_printf(seq, "  OfdHashSave:   %3s\n", yesno(rssconf & F_OFDHASHSAVE));
	seq_printf(seq, "  OfdVrtSel:     %3s\n", yesno(rssconf & F_OFDVRTSEL));
	seq_printf(seq, "  OfdMapEn:      %3s\n", yesno(rssconf & F_OFDMAPEN));
	seq_printf(seq, "  OfdLkpEn:      %3s\n", yesno(rssconf & F_OFDLKPEN));
	seq_printf(seq, "  Syn4TupEnIpv6: %3s\n", yesno(rssconf & F_SYN4TUPENIPV6));
	seq_printf(seq, "  Syn2TupEnIpv6: %3s\n", yesno(rssconf & F_SYN2TUPENIPV6));
	seq_printf(seq, "  Syn4TupEnIpv4: %3s\n", yesno(rssconf & F_SYN4TUPENIPV4));
	seq_printf(seq, "  Syn2TupEnIpv4: %3s\n", yesno(rssconf & F_SYN2TUPENIPV4));
	seq_printf(seq, "  Syn4TupEnIpv6: %3s\n", yesno(rssconf & F_SYN4TUPENIPV6));
	seq_printf(seq, "  SynIp6Sel:     %3s\n", yesno(rssconf & F_SYNIP6SEL));
	seq_printf(seq, "  SynVrt6Sel:    %3s\n", yesno(rssconf & F_SYNVRTSEL));
	seq_printf(seq, "  SynMapEn:      %3s\n", yesno(rssconf & F_SYNMAPEN));
	seq_printf(seq, "  SynLkpEn:      %3s\n", yesno(rssconf & F_SYNLKPEN));
	seq_printf(seq, "  ChnEn:         %3s\n", yesno(rssconf & F_CHANNELENABLE));
	seq_printf(seq, "  PrtEn:         %3s\n", yesno(rssconf & F_PORTENABLE));
	seq_printf(seq, "  TnlAllLkp:     %3s\n", yesno(rssconf & F_TNLALLLOOKUP));
	seq_printf(seq, "  VrtEn:         %3s\n", yesno(rssconf & F_VIRTENABLE));
	seq_printf(seq, "  CngEn:         %3s\n", yesno(rssconf & F_CONGESTIONENABLE));
	seq_printf(seq, "  HashToeplitz:  %3s\n", yesno(rssconf & F_HASHTOEPLITZ));
	seq_printf(seq, "  Udp4En:        %3s\n", yesno(rssconf & F_UDPENABLE));
	seq_printf(seq, "  Disable:       %3s\n", yesno(rssconf & F_DISABLE));

	seq_puts(seq, "\n");

	rssconf = t4_read_reg(adapter, A_TP_RSS_CONFIG_TNL);
	seq_printf(seq, "TP_RSS_CONFIG_TNL: %#x\n", rssconf);
	seq_printf(seq, "  MaskSize:      %3d\n", G_MASKSIZE(rssconf));
	seq_printf(seq, "  MaskFilter:    %3d\n", G_MASKFILTER(rssconf));
	seq_printf(seq, "  UseWireCh:     %3s\n", yesno(rssconf & F_USEWIRECH));

	seq_puts(seq, "\n");

	rssconf = t4_read_reg(adapter, A_TP_RSS_CONFIG_OFD);
	seq_printf(seq, "TP_RSS_CONFIG_OFD: %#x\n", rssconf);
	seq_printf(seq, "  MaskSize:      %3d\n", G_MASKSIZE(rssconf));
	seq_printf(seq, "  RRCplMapEn:    %3s\n", yesno(rssconf & F_RRCPLMAPEN));
	seq_printf(seq, "  RRCplQueWidth: %3d\n", G_RRCPLQUEWIDTH(rssconf));

	seq_puts(seq, "\n");

	rssconf = t4_read_reg(adapter, A_TP_RSS_CONFIG_SYN);
	seq_printf(seq, "TP_RSS_CONFIG_SYN: %#x\n", rssconf);
	seq_printf(seq, "  MaskSize:      %3d\n", G_MASKSIZE(rssconf));
	seq_printf(seq, "  UseWireCh:     %3s\n", yesno(rssconf & F_USEWIRECH));

	seq_puts(seq, "\n");

	rssconf = t4_read_reg(adapter, A_TP_RSS_CONFIG_VRT);
	seq_printf(seq, "TP_RSS_CONFIG_VRT: %#x\n", rssconf);
	seq_printf(seq, "  VfRdRg:        %3s\n", yesno(rssconf & F_VFRDRG));
	seq_printf(seq, "  VfRdEn:        %3s\n", yesno(rssconf & F_VFRDEN));
	seq_printf(seq, "  VfPerrEn:      %3s\n", yesno(rssconf & F_VFPERREN));
	seq_printf(seq, "  KeyPerrEn:     %3s\n", yesno(rssconf & F_KEYPERREN));
	seq_printf(seq, "  DisVfVlan:     %3s\n", yesno(rssconf & F_DISABLEVLAN));
	seq_printf(seq, "  EnUpSwt:       %3s\n", yesno(rssconf & F_ENABLEUP0));
	seq_printf(seq, "  HashDelay:     %3d\n", G_HASHDELAY(rssconf));
	seq_printf(seq, "  VfWrAddr:      %3d\n", G_VFWRADDR(rssconf));
	seq_printf(seq, "  KeyMode:       %s\n", keymode[G_KEYMODE(rssconf)]);
	seq_printf(seq, "  VfWrEn:        %3s\n", yesno(rssconf & F_VFWREN));
	seq_printf(seq, "  KeyWrEn:       %3s\n", yesno(rssconf & F_KEYWREN));
	seq_printf(seq, "  KeyWrAddr:     %3d\n", G_KEYWRADDR(rssconf));

	seq_puts(seq, "\n");

	rssconf = t4_read_reg(adapter, A_TP_RSS_CONFIG_CNG);
	seq_printf(seq, "TP_RSS_CONFIG_CNG: %#x\n", rssconf);
	seq_printf(seq, "  ChnCount3:     %3s\n", yesno(rssconf & F_CHNCOUNT3));
	seq_printf(seq, "  ChnCount2:     %3s\n", yesno(rssconf & F_CHNCOUNT2));
	seq_printf(seq, "  ChnCount1:     %3s\n", yesno(rssconf & F_CHNCOUNT1));
	seq_printf(seq, "  ChnCount0:     %3s\n", yesno(rssconf & F_CHNCOUNT0));
	seq_printf(seq, "  ChnUndFlow3:   %3s\n", yesno(rssconf & F_CHNUNDFLOW3));
	seq_printf(seq, "  ChnUndFlow2:   %3s\n", yesno(rssconf & F_CHNUNDFLOW2));
	seq_printf(seq, "  ChnUndFlow1:   %3s\n", yesno(rssconf & F_CHNUNDFLOW1));
	seq_printf(seq, "  ChnUndFlow0:   %3s\n", yesno(rssconf & F_CHNUNDFLOW0));
	seq_printf(seq, "  RstChn3:       %3s\n", yesno(rssconf & F_RSTCHN3));
	seq_printf(seq, "  RstChn2:       %3s\n", yesno(rssconf & F_RSTCHN2));
	seq_printf(seq, "  RstChn1:       %3s\n", yesno(rssconf & F_RSTCHN1));
	seq_printf(seq, "  RstChn0:       %3s\n", yesno(rssconf & F_RSTCHN0));
	seq_printf(seq, "  UpdVld:        %3s\n", yesno(rssconf & F_UPDVLD));
	seq_printf(seq, "  Xoff:          %3s\n", yesno(rssconf & F_XOFF));
	seq_printf(seq, "  UpdChn3:       %3s\n", yesno(rssconf & F_UPDCHN3));
	seq_printf(seq, "  UpdChn2:       %3s\n", yesno(rssconf & F_UPDCHN2));
	seq_printf(seq, "  UpdChn1:       %3s\n", yesno(rssconf & F_UPDCHN1));
	seq_printf(seq, "  UpdChn0:       %3s\n", yesno(rssconf & F_UPDCHN0));
	seq_printf(seq, "  Queue:         %3d\n", G_QUEUE(rssconf));

	return 0;
}

static int rss_config_open(struct inode *inode, struct file *file)
{
	return single_open(file, rss_config_show, PDE(inode)->data);
}

static const struct file_operations rss_config_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = rss_config_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

/*
 * RSS Secret Key.
 */

static int rss_key_show(struct seq_file *seq, void *v)
{
	u32 key[10];

	t4_read_rss_key(seq->private, key);
	seq_printf(seq, "%08x%08x%08x%08x%08x%08x%08x%08x%08x%08x\n",
		   key[9], key[8], key[7], key[6], key[5], key[4], key[3],
		   key[2], key[1], key[0]);
	return 0;
}

static int rss_key_open(struct inode *inode, struct file *file)
{
	return single_open(file, rss_key_show, PDE(inode)->data);
}

static inline unsigned int hex2val(char c)
{
	return isdigit(c) ? c - '0' : tolower(c) - 'a' + 10;
}

static ssize_t rss_key_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *pos)
{
	int i, j;
	u32 key[10];
	char s[100], *p;
	struct adapter *adap = PDE(file->f_path.dentry->d_inode)->data;

	if (count > sizeof(s) - 1)
		return -EINVAL;
	if (copy_from_user(s, buf, count))
		return -EFAULT;
	for (i = count; i > 0 && isspace(s[i - 1]); i--)
		;
	s[i] = '\0';

	for (p = s, i = 9; i >= 0; i--) {
		key[i] = 0;
		for (j = 0; j < 8; j++, p++) {
			if (!isxdigit(*p))
				return -EINVAL;
			key[i] = (key[i] << 4) | hex2val(*p);
		}
	}

	t4_write_rss_key(adap, key, -1);
	return count;
}

static const struct file_operations rss_key_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = rss_key_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
	.write   = rss_key_write
};

/*
 * PF RSS Configuration.
 */

struct rss_pf_conf {
	u32 rss_pf_map;
	u32 rss_pf_mask;
	u32 rss_pf_config;
};

static int rss_pf_config_show(struct seq_file *seq, void *v, int idx)
{
	struct rss_pf_conf *pfconf;

	if (v == SEQ_START_TOKEN) {
		/* use the 0th entry to dump the PF Map Index Size */
		pfconf = seq->private + offsetof(struct seq_tab, data);
		seq_printf(seq, "PF Map Index Size = %d\n\n",
			   G_LKPIDXSIZE(pfconf->rss_pf_map));

		seq_puts(seq, "     RSS              PF   VF    Hash Tuple Enable         Default\n");
		seq_puts(seq, "     Enable       IPF Mask Mask  IPv6      IPv4      UDP   Queue\n");
		seq_puts(seq, " PF  Map Chn Prt  Map Size Size  Four Two  Four Two  Four  Ch1  Ch0\n");
	} else {
		#define G_PFnLKPIDX(map, n) \
			(((map) >> S_PF1LKPIDX*(n)) & M_PF0LKPIDX)
		#define G_PFnMSKSIZE(mask, n) \
			(((mask) >> S_PF1MSKSIZE*(n)) & M_PF1MSKSIZE)

		pfconf = v;
		seq_printf(seq, "%3d  %3s %3s %3s  %3d  %3d  %3d   %3s %3s   %3s %3s   %3s  %3d  %3d\n",
			   idx,
			   yesno(pfconf->rss_pf_config & F_MAPENABLE),
			   yesno(pfconf->rss_pf_config & F_CHNENABLE),
			   yesno(pfconf->rss_pf_config & F_PRTENABLE),
			   G_PFnLKPIDX(pfconf->rss_pf_map, idx),
			   G_PFnMSKSIZE(pfconf->rss_pf_mask, idx),
			   G_IVFWIDTH(pfconf->rss_pf_config),
			   yesno(pfconf->rss_pf_config & F_IP6FOURTUPEN),
			   yesno(pfconf->rss_pf_config & F_IP6TWOTUPEN),
			   yesno(pfconf->rss_pf_config & F_IP4FOURTUPEN),
			   yesno(pfconf->rss_pf_config & F_IP4TWOTUPEN),
			   yesno(pfconf->rss_pf_config & F_UDPFOURTUPEN),
			   G_CH1DEFAULTQUEUE(pfconf->rss_pf_config),
			   G_CH0DEFAULTQUEUE(pfconf->rss_pf_config));

		#undef G_PFnLKPIDX
		#undef G_PFnMSKSIZE
	}
	return 0;
}

static int rss_pf_config_open(struct inode *inode, struct file *file)
{
	struct adapter *adapter = PDE(inode)->data;
	struct seq_tab *p;
	u32 rss_pf_map, rss_pf_mask;
	struct rss_pf_conf *pfconf;
	int pf;

	p = seq_open_tab(file, 8, sizeof(*pfconf), 1, rss_pf_config_show);
	if (!p)
		return -ENOMEM;

	pfconf = (struct rss_pf_conf *)p->data;
	rss_pf_map = t4_read_rss_pf_map(adapter);
	rss_pf_mask = t4_read_rss_pf_mask(adapter);
	for (pf = 0; pf < 8; pf++) {
		pfconf[pf].rss_pf_map = rss_pf_map;
		pfconf[pf].rss_pf_mask = rss_pf_mask;
		t4_read_rss_pf_config(adapter, pf, &pfconf[pf].rss_pf_config);
	}
	return 0;
}

static const struct file_operations rss_pf_config_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = rss_pf_config_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};

/*
 * VF RSS Configuration.
 */

struct rss_vf_conf {
	u32 rss_vf_vfl;
	u32 rss_vf_vfh;
};

static int rss_vf_config_show(struct seq_file *seq, void *v, int idx)
{
	if (v == SEQ_START_TOKEN) {
		seq_puts(seq, "     RSS                     Hash Tuple Enable\n");
		seq_puts(seq, "     Enable   IVF  Dis  Enb  IPv6      IPv4      UDP   Def  Secret Key\n");
		seq_puts(seq, " VF  Chn Prt  Map  VLAN  uP  Four Two  Four Two  Four  Que  Idx       Hash\n");
	} else {
		struct rss_vf_conf *vfconf = v;
		seq_printf(seq, "%3d  %3s %3s  %3d   %3s %3s   %3s %3s   %3s %3s   %3s  %3d  %3d %#10x\n",
			   idx,
			   yesno(vfconf->rss_vf_vfh & F_VFCHNEN),
			   yesno(vfconf->rss_vf_vfh & F_VFPRTEN),
			   G_VFLKPIDX(vfconf->rss_vf_vfh),
			   yesno(vfconf->rss_vf_vfh & F_VFVLNEX),
			   yesno(vfconf->rss_vf_vfh & F_VFUPEN),
			   yesno(vfconf->rss_vf_vfh & F_VFIP4FOURTUPEN),
			   yesno(vfconf->rss_vf_vfh & F_VFIP6TWOTUPEN),
			   yesno(vfconf->rss_vf_vfh & F_VFIP4FOURTUPEN),
			   yesno(vfconf->rss_vf_vfh & F_VFIP4TWOTUPEN),
			   yesno(vfconf->rss_vf_vfh & F_ENABLEUDPHASH),
			   G_DEFAULTQUEUE(vfconf->rss_vf_vfh),
			   G_KEYINDEX(vfconf->rss_vf_vfh),
			   vfconf->rss_vf_vfl);
	}
	return 0;
}

static int rss_vf_config_open(struct inode *inode, struct file *file)
{
	struct adapter *adapter = PDE(inode)->data;
	struct seq_tab *p;
	struct rss_vf_conf *vfconf;
	int vf;

	p = seq_open_tab(file, 128, sizeof(*vfconf), 1, rss_vf_config_show);
	if (!p)
		return -ENOMEM;

	vfconf = (struct rss_vf_conf *)p->data;
	for (vf = 0; vf < 128; vf++) {
		t4_read_rss_vf_config(adapter, vf, &vfconf[vf].rss_vf_vfl,
				      &vfconf[vf].rss_vf_vfh);
	}
	return 0;
}

static const struct file_operations rss_vf_config_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = rss_vf_config_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_private
};

static int mtutab_show(struct seq_file *seq, void *v)
{
	u16 mtus[NMTUS];
	struct adapter *adap = seq->private;

	spin_lock(&adap->stats_lock);
	t4_read_mtu_tbl(adap, mtus, NULL);
	spin_unlock(&adap->stats_lock);

	seq_printf(seq, "%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n",
		   mtus[0], mtus[1], mtus[2], mtus[3], mtus[4], mtus[5],
		   mtus[6], mtus[7], mtus[8], mtus[9], mtus[10], mtus[11],
		   mtus[12], mtus[13], mtus[14], mtus[15]);
	return 0;
}

static int mtutab_open(struct inode *inode, struct file *file)
{
	return single_open(file, mtutab_show, PDE(inode)->data);
}

static ssize_t mtutab_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *pos)
{
	int i;
	unsigned long mtus[NMTUS];
	struct adapter *adap = PDE(file->f_path.dentry->d_inode)->data;

	/* Require min MTU of 81 to accommodate SACK */
	i = rd_usr_int_vec(buf, count, NMTUS, mtus, 81, MAX_MTU, 10);
	if (i)
		return i;

	/* MTUs must be in ascending order */
	for (i = 1; i < NMTUS; ++i)
		if (mtus[i] < mtus[i - 1])
			return -EINVAL;

	/* can't change the MTU table if offload is in use */
	mutex_lock(&uld_mutex);
	for (i = 0; i < CXGB4_ULD_MAX; i++)
		if (adap->uld_handle[i]) {
			mutex_unlock(&uld_mutex);
			return -EBUSY;
		}

	for (i = 0; i < NMTUS; ++i)
		adap->params.mtus[i] = mtus[i];
	t4_load_mtus(adap, adap->params.mtus, adap->params.a_wnd,
		     adap->params.b_wnd);
	mutex_unlock(&uld_mutex);
	return count;
}

static const struct file_operations mtutab_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = mtutab_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
	.write   = mtutab_write
};

static int cctrl_tbl_show(struct seq_file *seq, void *v)
{
	static const char *dec_fac[] = {
		"0.5", "0.5625", "0.625", "0.6875", "0.75", "0.8125", "0.875",
		"0.9375" };

	int i;
	u16 incr[NMTUS][NCCTRL_WIN];
	struct adapter *adap = seq->private;

	t4_read_cong_tbl(adap, incr);

	for (i = 0; i < NCCTRL_WIN; ++i) {
		seq_printf(seq, "%2d: %4u %4u %4u %4u %4u %4u %4u %4u\n", i,
			   incr[0][i], incr[1][i], incr[2][i], incr[3][i],
			   incr[4][i], incr[5][i], incr[6][i], incr[7][i]);
		seq_printf(seq, "%8u %4u %4u %4u %4u %4u %4u %4u %5u %s\n",
			   incr[8][i], incr[9][i], incr[10][i], incr[11][i],
			   incr[12][i], incr[13][i], incr[14][i], incr[15][i],
			   adap->params.a_wnd[i],
			   dec_fac[adap->params.b_wnd[i]]);
	}
	return 0;
}

static int cctrl_tbl_open(struct inode *inode, struct file *file)
{
	return single_open(file, cctrl_tbl_show, PDE(inode)->data);
}

static const struct file_operations cctrl_tbl_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = cctrl_tbl_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

struct mem_desc {
	unsigned int base;
	unsigned int limit;
	unsigned int idx;
};

static int mem_desc_cmp(const void *a, const void *b)
{
	return ((const struct mem_desc *)a)->base -
	       ((const struct mem_desc *)b)->base;
}

static void mem_region_show(struct seq_file *seq, const char *name,
			    unsigned int from, unsigned int to)
{
	char buf[40];

	string_get_size((u64)to - from + 1, STRING_UNITS_2, buf, sizeof(buf));
	seq_printf(seq, "%-15s %#x-%#x [%s]\n", name, from, to, buf);
}

static int meminfo_show(struct seq_file *seq, void *v)
{
	static const char *memory[] = { "EDC0:", "EDC1:", "MC:" };
	static const char *region[] = {
		"DBQ contexts:", "IMSG contexts:", "FLM cache:", "TCBs:",
		"Pstructs:", "Timers:", "Rx FL:", "Tx FL:", "Pstruct FL:",
		"Tx payload:", "Rx payload:", "LE hash:", "iSCSI region:",
		"TDDP region:", "TPT region:", "STAG region:", "RQ region:",
		"RQUDP region:", "PBL region:", "TXPBL region:", "ULPRX state:",
		"ULPTX state:", "On-chip queues:"
	};

	int i, n;
	u32 lo, hi;
	struct mem_desc avail[3];
	struct mem_desc mem[ARRAY_SIZE(region) + 3];      /* up to 3 holes */
	struct mem_desc *md = mem;
	struct adapter *adap = seq->private;

	for (i = 0; i < ARRAY_SIZE(mem); i++) {
		mem[i].limit = 0;
		mem[i].idx = i;
	}

	/* Find and sort the populated memory ranges */
	i = 0;
	lo = t4_read_reg(adap, A_MA_TARGET_MEM_ENABLE);
	if (lo & F_EDRAM0_ENABLE) {
		hi = t4_read_reg(adap, A_MA_EDRAM0_BAR);
		avail[i].base = G_EDRAM0_BASE(hi) << 20;
		avail[i].limit = avail[i].base + (G_EDRAM0_SIZE(hi) << 20);
		avail[i].idx = 0;
		i++;
	}
	if (lo & F_EDRAM1_ENABLE) {
		hi = t4_read_reg(adap, A_MA_EDRAM1_BAR);
		avail[i].base = G_EDRAM1_BASE(hi) << 20;
		avail[i].limit = avail[i].base + (G_EDRAM1_SIZE(hi) << 20);
		avail[i].idx = 1;
		i++;
	}
	if (lo & F_EXT_MEM_ENABLE) {
		hi = t4_read_reg(adap, A_MA_EXT_MEMORY_BAR);
		avail[i].base = G_EXT_MEM_BASE(hi) << 20;
		avail[i].limit = avail[i].base + (G_EXT_MEM_SIZE(hi) << 20);
		avail[i].idx = 2;
		i++;
	}
	if (!i)                                    /* no memory available */
		return 0;
	sort(avail, i, sizeof(struct mem_desc), mem_desc_cmp, NULL);

	(md++)->base = t4_read_reg(adap, A_SGE_DBQ_CTXT_BADDR);
	(md++)->base = t4_read_reg(adap, A_SGE_IMSG_CTXT_BADDR);
	(md++)->base = t4_read_reg(adap, A_SGE_FLM_CACHE_BADDR);
	(md++)->base = t4_read_reg(adap, A_TP_CMM_TCB_BASE);
	(md++)->base = t4_read_reg(adap, A_TP_CMM_MM_BASE);
	(md++)->base = t4_read_reg(adap, A_TP_CMM_TIMER_BASE);
	(md++)->base = t4_read_reg(adap, A_TP_CMM_MM_RX_FLST_BASE);
	(md++)->base = t4_read_reg(adap, A_TP_CMM_MM_TX_FLST_BASE);
	(md++)->base = t4_read_reg(adap, A_TP_CMM_MM_PS_FLST_BASE);

	/* the next few have explicit upper bounds */
	md->base = t4_read_reg(adap, A_TP_PMM_TX_BASE);
	md->limit = md->base - 1 +
		    t4_read_reg(adap, A_TP_PMM_TX_PAGE_SIZE) *
		    G_PMTXMAXPAGE(t4_read_reg(adap, A_TP_PMM_TX_MAX_PAGE));
	md++;

	md->base = t4_read_reg(adap, A_TP_PMM_RX_BASE);
	md->limit = md->base - 1 +
		    t4_read_reg(adap, A_TP_PMM_RX_PAGE_SIZE) *
		    G_PMRXMAXPAGE(t4_read_reg(adap, A_TP_PMM_RX_MAX_PAGE));
	md++;

	if (t4_read_reg(adap, A_LE_DB_CONFIG) & F_HASHEN) {
		hi = t4_read_reg(adap, A_LE_DB_TID_HASHBASE) / 4;
		md->base = t4_read_reg(adap, A_LE_DB_HASH_TID_BASE);
		md->limit = (adap->tids.ntids - hi) * 16 + md->base - 1;
	} else {
		md->base = 0;
		md->idx = ARRAY_SIZE(region);  /* hide it */
	}
	md++;

#define ulp_region(reg) \
	md->base = t4_read_reg(adap, A_ULP_ ## reg ## _LLIMIT);\
	(md++)->limit = t4_read_reg(adap, A_ULP_ ## reg ## _ULIMIT)

	ulp_region(RX_ISCSI);
	ulp_region(RX_TDDP);
	ulp_region(TX_TPT);
	ulp_region(RX_STAG);
	ulp_region(RX_RQ);
	ulp_region(RX_RQUDP);
	ulp_region(RX_PBL);
	ulp_region(TX_PBL);
#undef ulp_region

	md->base = t4_read_reg(adap, A_ULP_RX_CTX_BASE);
	md->limit = md->base + adap->tids.ntids - 1;
	md++;
	md->base = t4_read_reg(adap, A_ULP_TX_ERR_TABLE_BASE);
	md->limit = md->base + adap->tids.ntids - 1;
	md++;

	md->base = adap->vres.ocq.start;
	if (adap->vres.ocq.size)
		md->limit = md->base + adap->vres.ocq.size - 1;
	else
		md->idx = ARRAY_SIZE(region);  /* hide it */
	md++;

	/* add any address-space holes, there can be up to 3 */
	for (n = 0; n < i - 1; n++)
		if (avail[n].limit < avail[n + 1].base)
			(md++)->base = avail[n].limit;
	if (avail[n].limit)
		(md++)->base = avail[n].limit;

	n = md - mem;
	sort(mem, n, sizeof(struct mem_desc), mem_desc_cmp, NULL);

	for (lo = 0; lo < i; lo++)
		mem_region_show(seq, memory[avail[lo].idx], avail[lo].base,
				avail[lo].limit - 1);

	seq_putc(seq, '\n');
	for (i = 0; i < n; i++) {
		if (mem[i].idx >= ARRAY_SIZE(region))
			continue;                        /* skip holes */
		if (!mem[i].limit)
			mem[i].limit = i < n - 1 ? mem[i + 1].base - 1 : ~0;
		mem_region_show(seq, region[mem[i].idx], mem[i].base,
				mem[i].limit);
	}

	seq_putc(seq, '\n');
	lo = t4_read_reg(adap, A_CIM_SDRAM_BASE_ADDR);
	hi = t4_read_reg(adap, A_CIM_SDRAM_ADDR_SIZE) + lo - 1;
	mem_region_show(seq, "uP RAM:", lo, hi);

	lo = t4_read_reg(adap, A_CIM_EXTMEM2_BASE_ADDR);
	hi = t4_read_reg(adap, A_CIM_EXTMEM2_ADDR_SIZE) + lo - 1;
	mem_region_show(seq, "uP Extmem2:", lo, hi);

	lo = t4_read_reg(adap, A_TP_PMM_RX_MAX_PAGE);
	seq_printf(seq, "\n%u Rx pages of size %uKiB for %u channels\n",
		   G_PMRXMAXPAGE(lo),
		   t4_read_reg(adap, A_TP_PMM_RX_PAGE_SIZE) >> 10,
		   (lo & F_PMRXNUMCHN) ? 2 : 1);

	lo = t4_read_reg(adap, A_TP_PMM_TX_MAX_PAGE);
	hi = t4_read_reg(adap, A_TP_PMM_TX_PAGE_SIZE);
	seq_printf(seq, "%u Tx pages of size %u%ciB for %u channels\n",
		   G_PMTXMAXPAGE(lo),
		   hi >= (1 << 20) ? (hi >> 20) : (hi >> 10),
		   hi >= (1 << 20) ? 'M' : 'K', 1 << G_PMTXNUMCHN(lo));
	seq_printf(seq, "%u p-structs\n\n",
		   t4_read_reg(adap, A_TP_CMM_MM_MAX_PSTRUCT));

	for (i = 0; i < 4; i++) {
		lo = t4_read_reg(adap, A_MPS_RX_PG_RSV0 + i * 4);
		seq_printf(seq, "Port %d using %u pages out of %u allocated\n",
			   i, G_USED(lo), G_ALLOC(lo));
	}
	for (i = 0; i < 4; i++) {
		lo = t4_read_reg(adap, A_MPS_RX_PG_RSV4 + i * 4);
		seq_printf(seq,
			   "Loopback %d using %u pages out of %u allocated\n",
			   i, G_USED(lo), G_ALLOC(lo));
	}
	return 0;
}

static int meminfo_open(struct inode *inode, struct file *file)
{
	return single_open(file, meminfo_show, PDE(inode)->data);
}

static const struct file_operations meminfo_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = meminfo_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int mps_trc_show(struct seq_file *seq, void *v)
{
	int enabled, i;
	struct trace_params tp;
	struct adapter *adap = seq->private;
	unsigned int trcidx = (uintptr_t)adap & 3;

	adap = (void *)adap - trcidx;
	t4_get_trace_filter(adap, &tp, trcidx, &enabled);
	if (!enabled) {
		seq_puts(seq, "tracer is disabled\n");
		return 0;
	}

	if (tp.skip_ofst * 8 >= TRACE_LEN) {
		dev_err(adap->pdev_dev, "illegal trace pattern skip offset\n");
		return -EINVAL;
	}
	if (tp.port < 8) {
		i = adap->chan_map[tp.port & 3];
		if (i >= MAX_NPORTS) {
			dev_err(adap->pdev_dev, "tracer %u is assigned "
				"to non-existing port\n", trcidx);
			return -EINVAL;
		}
		seq_printf(seq, "tracer is capturing %s %s, ",
			   adap->port[i]->name, tp.port < 4 ? "Rx" : "Tx");
	} else
		seq_printf(seq, "tracer is capturing loopback %d, ",
			   tp.port - 8);
	seq_printf(seq, "snap length: %u, min length: %u\n", tp.snap_len,
		   tp.min_len);
	seq_printf(seq, "packets captured %smatch filter\n",
		   tp.invert ? "do not " : "");

	if (tp.skip_ofst) {
		seq_puts(seq, "filter pattern: ");
		for (i = 0; i < tp.skip_ofst * 2; i += 2)
			seq_printf(seq, "%08x%08x", tp.data[i], tp.data[i + 1]);
		seq_putc(seq, '/');
		for (i = 0; i < tp.skip_ofst * 2; i += 2)
			seq_printf(seq, "%08x%08x", tp.mask[i], tp.mask[i + 1]);
		seq_puts(seq, "@0\n");
	}

	seq_puts(seq, "filter pattern: ");
	for (i = tp.skip_ofst * 2; i < TRACE_LEN / 4; i += 2)
		seq_printf(seq, "%08x%08x", tp.data[i], tp.data[i + 1]);
	seq_putc(seq, '/');
	for (i = tp.skip_ofst * 2; i < TRACE_LEN / 4; i += 2)
		seq_printf(seq, "%08x%08x", tp.mask[i], tp.mask[i + 1]);
	seq_printf(seq, "@%u\n", (tp.skip_ofst + tp.skip_len) * 8);
	return 0;
}

static int mps_trc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mps_trc_show, PDE(inode)->data);
}

static unsigned int xdigit2int(unsigned char c)
{
	return isdigit(c) ? c - '0' : tolower(c) - 'a' + 10;
}

#define TRC_PORT_NONE 0xff

/*
 * Set an MPS trace filter.  Syntax is:
 *
 * disable
 *
 * to disable tracing, or
 *
 * interface [snaplen=<val>] [minlen=<val>] [not] [<pattern>]...
 *
 * where interface is one of rxN, txN, or loopbackN, N = 0..3, and pattern
 * has the form
 *
 * <pattern data>[/<pattern mask>][@<anchor>]
 *
 * Up to 2 filter patterns can be specified.  If 2 are supplied the first one
 * must be anchored at 0.  An omited mask is taken as a mask of 1s, an omitted
 * anchor is taken as 0.
 */
static ssize_t mps_trc_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *pos)
{
	int i, j, enable;
	u32 *data, *mask;
	struct trace_params tp;
	char *s, *p, *word, *end;
	struct adapter *adap = PDE(file->f_path.dentry->d_inode)->data;
	unsigned int trcidx = (uintptr_t)adap & 3;

	adap = (void *)adap - trcidx;

	/*
	 * Don't accept input more than 1K, can't be anything valid except lots
	 * of whitespace.  Well, use less.
	 */
	if (count > 1024)
		return -EFBIG;
	p = s = kzalloc(count + 1, GFP_USER);
	if (!s)
		return -ENOMEM;
	if (copy_from_user(s, buf, count)) {
		count = -EFAULT;
		goto out;
	}

	if (s[count - 1] == '\n')
		s[count - 1] = '\0';

	enable = strcmp("disable", s) != 0;
	if (!enable)
		goto apply;

	memset(&tp, 0, sizeof(tp));
	tp.port = TRC_PORT_NONE;
	i = 0;                                      /* counts pattern nibbles */

	while (p) {
		while (isspace(*p))
			p++;
		word = strsep(&p, " ");
		if (!*word)
			break;

		if (!strncmp(word, "snaplen=", 8)) {
			j = simple_strtoul(word + 8, &end, 10);
			if (*end || j > 9600) {
inval:				count = -EINVAL;
				goto out;
			}
			tp.snap_len = j;
			continue;
		}
		if (!strncmp(word, "minlen=", 7)) {
			j = simple_strtoul(word + 7, &end, 10);
			if (*end || j > M_TFMINPKTSIZE)
				goto inval;
			tp.min_len = j;
			continue;
		}
		if (!strcmp(word, "not")) {
			tp.invert = !tp.invert;
			continue;
		}
		if (!strncmp(word, "loopback", 8) && tp.port == TRC_PORT_NONE) {
			if (word[8] < '0' || word[8] > '3' || word[9])
				goto inval;
			tp.port = word[8] - '0' + 8;
			continue;
		}
		if (!strncmp(word, "tx", 2) && tp.port == TRC_PORT_NONE) {
			if (word[2] < '0' || word[2] > '3' || word[3])
				goto inval;
			tp.port = word[2] - '0' + 4;
			if (adap->chan_map[tp.port & 3] >= MAX_NPORTS)
				goto inval;
			continue;
		}
		if (!strncmp(word, "rx", 2) && tp.port == TRC_PORT_NONE) {
			if (word[2] < '0' || word[2] > '3' || word[3])
				goto inval;
			tp.port = word[2] - '0';
			if (adap->chan_map[tp.port] >= MAX_NPORTS)
				goto inval;
			continue;
		}
		if (!isxdigit(*word))
			goto inval;

		/* we have found a trace pattern */
		if (i) {                            /* split pattern */
			if (tp.skip_len)            /* too many splits */
				goto inval;
			tp.skip_ofst = i / 16;
		}

		data = &tp.data[i / 8];
		mask = &tp.mask[i / 8];
		j = i;

		while (isxdigit(*word)) {
			if (i >= TRACE_LEN * 2) {
				count = -EFBIG;
				goto out;
			}
			*data = (*data << 4) + xdigit2int(*word++);
			if (++i % 8 == 0)
				data++;
		}
		if (*word == '/') {
			word++;
			while (isxdigit(*word)) {
				if (j >= i)         /* mask longer than data */
					goto inval;
				*mask = (*mask << 4) + xdigit2int(*word++);
				if (++j % 8 == 0)
					mask++;
			}
			if (i != j)                 /* mask shorter than data */
				goto inval;
		} else {                            /* no mask, use all 1s */
			for ( ; i - j >= 8; j += 8)
				*mask++ = 0xffffffff;
			if (i % 8)
				*mask = (1 << (i % 8) * 4) - 1;
		}
		if (*word == '@') {
			j = simple_strtoul(word + 1, &end, 10);
			if (*end && *end != '\n')
				goto inval;
			if (j & 7)          /* doesn't start at multiple of 8 */
				goto inval;
			j /= 8;
			if (j < tp.skip_ofst)     /* overlaps earlier pattern */
				goto inval;
			if (j - tp.skip_ofst > 31)            /* skip too big */
				goto inval;
			tp.skip_len = j - tp.skip_ofst;
		}
		if (i % 8) {
			*data <<= (8 - i % 8) * 4;
			*mask <<= (8 - i % 8) * 4;
			i = (i + 15) & ~15;         /* 8-byte align */
		}
	}

	if (tp.port == TRC_PORT_NONE)
		goto inval;

#if 0
	if (tp.port < 8)
		printk("tracer is capturing %s %s, ",
			adap->port[adap->chan_map[tp.port & 3]]->name,
			tp.port < 4 ? "Rx" : "Tx");
	else
		printk("tracer is capturing loopback %u, ", tp.port - 8);
	printk("snap length: %u, min length: %u\n", tp.snap_len, tp.min_len);
	printk("packets captured %smatch filter\n", tp.invert ? "do not " : "");

	if (tp.skip_ofst) {
		printk("filter pattern: ");
		for (i = 0; i < tp.skip_ofst * 2; i += 2)
			printk("%08x%08x", tp.data[i], tp.data[i + 1]);
		printk("/");
		for (i = 0; i < tp.skip_ofst * 2; i += 2)
			printk("%08x%08x", tp.mask[i], tp.mask[i + 1]);
		printk("@0\n");
	}

	printk("filter pattern: ");
	for (i = tp.skip_ofst * 2; i < TRACE_LEN / 4; i += 2)
		printk("%08x%08x", tp.data[i], tp.data[i + 1]);
	printk("/");
	for (i = tp.skip_ofst * 2; i < TRACE_LEN / 4; i += 2)
		printk("%08x%08x", tp.mask[i], tp.mask[i + 1]);
	printk("@%u\n", (tp.skip_ofst + tp.skip_len) * 8);
#endif

apply:
	i = t4_set_trace_filter(adap, &tp, trcidx, enable);
	if (i)
		count = i;
out:
	kfree(s);
	return count;
}

static const struct file_operations mps_trc_proc_fops = {
	.owner   = THIS_MODULE,
	.open    = mps_trc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
	.write   = mps_trc_write
};

static int tid_info_show(struct seq_file *seq, void *v)
{
	struct adapter *adap = seq->private;
	const struct tid_info *t = &adap->tids;

	if (t4_read_reg(adap, A_LE_DB_CONFIG) & F_HASHEN) {
		unsigned int sb = t4_read_reg(adap, A_LE_DB_SERVER_INDEX) / 4;

		if (sb)
			seq_printf(seq, "TID range: 0..%u/%u..%u", sb - 1,
				   t4_read_reg(adap, A_LE_DB_TID_HASHBASE) / 4,
				   t->ntids - 1);
		else
			seq_printf(seq, "TID range: %u..%u",
				   t4_read_reg(adap, A_LE_DB_TID_HASHBASE) / 4,
				   t->ntids - 1);
	} else
		seq_printf(seq, "TID range: 0..%u", t->ntids - 1);
	seq_printf(seq, ", in use: %u\n", atomic_read(&t->tids_in_use));
	seq_printf(seq, "STID range: %u..%u, in use: %u\n", t->stid_base,
		   t->stid_base + t->nstids - 1, t->stids_in_use);
	seq_printf(seq, "ATID range: 0..%u, in use: %u\n", t->natids - 1,
		   t->atids_in_use);
	seq_printf(seq, "FTID range: %u..%u\n", t->ftid_base, 
		   t->ftid_base + t->nftids - 1);
	seq_printf(seq, "HW TID usage: %u IP users, %u IPv6 users\n",
		   t4_read_reg(adap, A_LE_DB_ACT_CNT_IPV4),
		   t4_read_reg(adap, A_LE_DB_ACT_CNT_IPV6));
	return 0;
}

DEFINE_SIMPLE_PROC_FILE(tid_info);

static int uld_show(struct seq_file *seq, void *v)
{
	int i;
	struct adapter *adap = seq->private;

	for (i = 0; i < CXGB4_ULD_MAX; i++)
		if (adap->uld_handle[i])
			seq_printf(seq, "%s: %s\n", uld_str[i], ulds[i].name);
	return 0;
}

DEFINE_SIMPLE_PROC_FILE(uld);

enum {
	ADAP_NEED_L2T  = 1 << 0,
	ADAP_NEED_OFLD = 1 << 1,
	ADAP_NEED_FILT = 1 << 2,
};

struct cxgb4_proc_entry {
	const char *name;
	mode_t mode;
	unsigned int req;      /* adapter requirements to create this file */
	unsigned char data;    /* data passed in low bits of adapter pointer */
	const struct file_operations *fops;
};

static struct cxgb4_proc_entry proc_files[] = {
	{ "cctrl", 0444, 0, 0, &cctrl_tbl_proc_fops },
	{ "cpl_stats", 0444, 0, 0, &cpl_stats_proc_fops },
	{ "ddp_stats", 0444, ADAP_NEED_OFLD, 0, &ddp_stats_proc_fops },
	{ "fcoe_stats", 0444, 0, 0, &fcoe_stats_proc_fops },
#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	{ "filters", 0444, 0, 0, &filters_proc_fops },
#endif
	{ "hw_sched", 0444, 0, 0, &sched_proc_fops },
#if defined(CONFIG_PROC_FS) && defined(CONFIG_CHELSIO_T4_OFFLOAD)
	{ "l2t", 0444, ADAP_NEED_L2T, 0, &t4_l2t_proc_fops },
#endif
	{ "lb_stats", 0444, 0, 0, &lb_stats_proc_fops },
	{ "meminfo", 0444, 0, 0, &meminfo_proc_fops },
	{ "path_mtus", 0644, 0, 0, &mtutab_proc_fops },
	{ "pm_stats", 0644, 0, 0, &pm_stats_proc_fops },
	{ "qstats", 0444, 0, 0, &sge_stats_proc_fops },
	{ "rdma_stats", 0444, ADAP_NEED_OFLD, 0, &rdma_stats_proc_fops },
	{ "rss", 0444, 0, 0, &rss_proc_fops },
	{ "rss_config", 0444, 0, 0, &rss_config_proc_fops },
	{ "rss_key", 0600, 0, 0, &rss_key_proc_fops },
	{ "rss_pf_config", 0444, 0, 0, &rss_pf_config_proc_fops },
	{ "rss_vf_config", 0444, 0, 0, &rss_vf_config_proc_fops },
	{ "tcp_stats", 0444, 0, 0, &tcp_stats_proc_fops },
	{ "tids", 0444, ADAP_NEED_OFLD, 0, &tid_info_proc_fops },
	{ "tp_err_stats", 0444, 0, 0, &tp_err_stats_proc_fops },
	{ "trace0", 0644, 0, 0, &mps_trc_proc_fops },
	{ "trace1", 0644, 0, 1, &mps_trc_proc_fops },
	{ "trace2", 0644, 0, 2, &mps_trc_proc_fops },
	{ "trace3", 0644, 0, 3, &mps_trc_proc_fops },
	{ "tx_rate", 0444, 0, 0, &tx_rate_proc_fops },
	{ "uld", 0444, 0, 0, &uld_proc_fops },
};

static int __devinit setup_proc(struct adapter *adap,
				struct proc_dir_entry *dir)
{
	int i, created;
	struct proc_dir_entry *p;
	int ofld = is_offload(adap);

	if (!dir)
		return -EINVAL;

	/* If we can create any of the entries we do. */
	for (created = i = 0; i < ARRAY_SIZE(proc_files); ++i) {
		unsigned int req = proc_files[i].req;

		if ((req & ADAP_NEED_OFLD) && !ofld)
			continue;
		if ((req & ADAP_NEED_L2T) && !adap->l2t)
			continue;

		p = proc_create_data(proc_files[i].name, proc_files[i].mode,
				     dir, proc_files[i].fops,
				     (void *)adap + proc_files[i].data);
		if (p)
			created++;
	}

	return created;
}

static void cleanup_proc(struct adapter *adap, struct proc_dir_entry *dir)
{
	int i;
	int ofld = is_offload(adap);

	for (i = 0; i < ARRAY_SIZE(proc_files); ++i) {
		unsigned int req = proc_files[i].req;

		if ((req & ADAP_NEED_OFLD) && !ofld)
			continue;
		if ((req & ADAP_NEED_L2T) && !adap->l2t)
			continue;

		remove_proc_entry(proc_files[i].name, dir);
	}
}

/*
 * offload upper-layer driver support
 */

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
#include <net/offload.h>
#include "cxgb4_ctl_defs.h"

/*
 * Allocate an active-open TID and set it to the supplied value.
 */
int cxgb4_alloc_atid(struct tid_info *t, void *data)
{
	int atid = -1;

	spin_lock_bh(&t->atid_lock);
	if (t->afree) {
		union aopen_entry *p = t->afree;

		atid = p - t->atid_tab;
		t->afree = p->next;
		p->data = data;
		t->atids_in_use++;
	}
	spin_unlock_bh(&t->atid_lock);
	return atid;
}
EXPORT_SYMBOL(cxgb4_alloc_atid);

/*
 * Release an active-open TID.
 */
void cxgb4_free_atid(struct tid_info *t, unsigned int atid)
{
	union aopen_entry *p = &t->atid_tab[atid];

	spin_lock_bh(&t->atid_lock);
	p->next = t->afree;
	t->afree = p;
	t->atids_in_use--;
	spin_unlock_bh(&t->atid_lock);
}
EXPORT_SYMBOL(cxgb4_free_atid);

/*
 * Allocate a server TID and set it to the supplied value.
 */
int cxgb4_alloc_stid(struct tid_info *t, int family, void *data)
{
	int stid;

	spin_lock_bh(&t->stid_lock);
	if (family == PF_INET) {
		stid = find_first_zero_bit(t->stid_bmap, t->nstids);
		if (stid < t->nstids)
			__set_bit(stid, t->stid_bmap);
		else
			stid = -1;
	} else {
		stid = bitmap_find_free_region(t->stid_bmap, t->nstids, 2);
		if (stid < 0)
			stid = -1;
	}
	if (stid >= 0) {
		t->stid_tab[stid].data = data;
		stid += t->stid_base;
		t->stids_in_use++;
	}
	spin_unlock_bh(&t->stid_lock);
	return stid;
}
EXPORT_SYMBOL(cxgb4_alloc_stid);

/*
 * Release a server TID.
 */
void cxgb4_free_stid(struct tid_info *t, unsigned int stid, int family)
{
	stid -= t->stid_base;
	spin_lock_bh(&t->stid_lock);
	if (family == PF_INET)
		__clear_bit(stid, t->stid_bmap);
	else
		bitmap_release_region(t->stid_bmap, stid, 2);
	t->stid_tab[stid].data = NULL;
	t->stids_in_use--;
	spin_unlock_bh(&t->stid_lock);
}
EXPORT_SYMBOL(cxgb4_free_stid);

/*
 * Populate a TID_RELEASE WR.  Caller must properly size the skb.
 */
static void mk_tid_release(struct sk_buff *skb, unsigned int chan,
			   unsigned int tid)
{
	struct cpl_tid_release *req;

	set_wr_txq(skb, CPL_PRIORITY_SETUP, chan);
	req = (struct cpl_tid_release *)__skb_put(skb, sizeof(*req));
	INIT_TP_WR_MIT_CPL(req, CPL_TID_RELEASE, tid);
}

/*
 * Queue a TID release request and if necessary schedule a work queue to
 * process it.
 */
static void cxgb4_queue_tid_release(struct tid_info *t, unsigned int chan,
			     unsigned int tid)
{
	void **p = &t->tid_tab[tid];
	struct adapter *adap = container_of(t, struct adapter, tids);

	spin_lock_bh(&adap->tid_release_lock);
	*p = adap->tid_release_head;
	/* Low 2 bits encode the Tx channel number */
	adap->tid_release_head = (void **)((uintptr_t)p | chan);
	if (!*p)
		schedule_work(&adap->tid_release_task);
	spin_unlock_bh(&adap->tid_release_lock);
}

/*
 * Process the list of pending TID release requests.
 */
static void process_tid_release_list(struct work_struct *work)
{
	struct sk_buff *skb;
	struct adapter *adap;

	adap = container_of(work, struct adapter, tid_release_task);

	spin_lock_bh(&adap->tid_release_lock);
	while (adap->tid_release_head) {
		void **p = adap->tid_release_head;
		unsigned int chan = (uintptr_t)p & 3;
		p = (void *)p - chan;

		adap->tid_release_head = *p;
		*p = NULL;
		spin_unlock_bh(&adap->tid_release_lock);

		while (!(skb = alloc_skb(sizeof(struct cpl_tid_release),
					 GFP_KERNEL)))
			yield();

		mk_tid_release(skb, chan, p - adap->tids.tid_tab);
		t4_ofld_send(adap, skb);
		spin_lock_bh(&adap->tid_release_lock);
	}
	spin_unlock_bh(&adap->tid_release_lock);
}

/*
 * Release a TID and inform HW.  If we are unable to allocate the release
 * message we defer to a work queue.
 */
void cxgb4_remove_tid(struct tid_info *t, unsigned int chan, unsigned int tid)
{
	struct sk_buff *skb;
	struct adapter *adap = container_of(t, struct adapter, tids);

	WARN_ON(tid >= t->ntids);

	if (t->tid_tab[tid]) {
		t->tid_tab[tid] = NULL;
		atomic_dec(&t->tids_in_use);
	}

	skb = alloc_skb(sizeof(struct cpl_tid_release), GFP_ATOMIC);
	if (likely(skb)) {
		mk_tid_release(skb, chan, tid);
		t4_ofld_send(adap, skb);
	} else
		cxgb4_queue_tid_release(t, chan, tid);
}
EXPORT_SYMBOL(cxgb4_remove_tid);

/*
 * Allocate and initialize the TID tables.  Returns 0 on success.
 */
static int tid_init(struct tid_info *t)
{
	size_t size;
	unsigned int stid_bmap_size;
	unsigned int natids = t->natids;

	stid_bmap_size = BITS_TO_LONGS(t->nstids);
	size = t->ntids * sizeof(*t->tid_tab) +
	       natids * sizeof(*t->atid_tab) +
	       t->nstids * sizeof(*t->stid_tab) +
	       stid_bmap_size * sizeof(long) +
	       t->nftids * sizeof(*t->ftid_tab);
	t->tid_tab = t4_alloc_mem(size);
	if (!t->tid_tab)
		return -ENOMEM;

	t->atid_tab = (union aopen_entry *)&t->tid_tab[t->ntids];
	t->stid_tab = (struct serv_entry *)&t->atid_tab[natids];
	t->stid_bmap = (unsigned long *)&t->stid_tab[t->nstids];
	t->ftid_tab = (struct filter_entry *)&t->stid_bmap[stid_bmap_size];
	spin_lock_init(&t->stid_lock);
	spin_lock_init(&t->atid_lock);

	t->stids_in_use = 0;
	t->afree = NULL;
	t->atids_in_use = 0;
	atomic_set(&t->tids_in_use, 0);

	/* Setup the free list for atid_tab and clear the stid bitmap. */
	if (natids) {
		while (--natids)
			t->atid_tab[natids - 1].next = &t->atid_tab[natids];
		t->afree = t->atid_tab;
	}
	bitmap_zero(t->stid_bmap, t->nstids);
	return 0;
}

/**
 *	cxgb4_create_server - create an IP server
 *	@dev: the device
 *	@stid: the server TID
 *	@sip: local IP address to bind server to
 *	@sport: the server's TCP port
 *	@queue: queue to direct messages from this server to
 *
 *	Create an IP server for the given port and address.
 *	Returns <0 on error and one of the %NET_XMIT_* values on success.
 */
int cxgb4_create_server(const struct net_device *dev, unsigned int stid,
			__be32 sip, __be16 sport, unsigned int queue)
{
	unsigned int chan;
	struct sk_buff *skb;
	struct adapter *adap;
	struct cpl_pass_open_req *req;

	skb = alloc_skb(sizeof(*req), GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	adap = netdev2adap(dev);
	req = (struct cpl_pass_open_req *)__skb_put(skb, sizeof(*req));
	INIT_TP_WR(req, 0);
	OPCODE_TID(req) = htonl(MK_OPCODE_TID(CPL_PASS_OPEN_REQ, stid));
	req->local_port = sport;
	req->peer_port = htons(0);
	req->local_ip = sip;
	req->peer_ip = htonl(0);
	chan = rxq_to_chan(&adap->sge, queue);
	req->opt0 = cpu_to_be64(V_TX_CHAN(chan));
	req->opt1 = cpu_to_be64(V_CONN_POLICY(CPL_CONN_POLICY_ASK) |
				F_SYN_RSS_ENABLE | V_SYN_RSS_QUEUE(queue));
	return t4_mgmt_tx(adap, skb);
}
EXPORT_SYMBOL(cxgb4_create_server);

/**
 *	cxgb4_create_server6 - create an IPv6 server
 *	@dev: the device
 *	@stid: the server TID
 *	@sip: local IPv6 address to bind server to
 *	@sport: the server's TCP port
 *	@queue: queue to direct messages from this server to
 *
 *	Create an IPv6 server for the given port and address.
 *	Returns <0 on error and one of the %NET_XMIT_* values on success.
 */
int cxgb4_create_server6(const struct net_device *dev, unsigned int stid,
			 const struct in6_addr *sip, __be16 sport,
			 unsigned int queue)
{
	unsigned int chan;
	struct sk_buff *skb;
	struct adapter *adap;
	struct cpl_pass_open_req6 *req;

	skb = alloc_skb(sizeof(*req), GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	adap = netdev2adap(dev);
	req = (struct cpl_pass_open_req6 *)__skb_put(skb, sizeof(*req));
	INIT_TP_WR(req, 0);
	OPCODE_TID(req) = htonl(MK_OPCODE_TID(CPL_PASS_OPEN_REQ6, stid));
	req->local_port = sport;
	req->peer_port = htons(0);
	req->local_ip_hi = *(__be64 *)(sip->s6_addr);
	req->local_ip_lo = *(__be64 *)(sip->s6_addr + 8);
	req->peer_ip_hi = cpu_to_be64(0);
	req->peer_ip_lo = cpu_to_be64(0);
	chan = rxq_to_chan(&adap->sge, queue);
	req->opt0 = cpu_to_be64(V_TX_CHAN(chan));
	req->opt1 = cpu_to_be64(V_CONN_POLICY(CPL_CONN_POLICY_ASK) |
				F_SYN_RSS_ENABLE | V_SYN_RSS_QUEUE(queue));
	return t4_mgmt_tx(adap, skb);
}
EXPORT_SYMBOL(cxgb4_create_server6);

/**
 *	cxgb4_setup_ddpbuf - set up a DDP buffer
 *	@pdev: the PCI device
 *	@bus_addr: bus addresses of pages making up the buffer
 *	@naddr: number of entries in @bus_addr
 *	@tid: connection id associated with the buffer
 *	@tag: HW buffer tag
 *	@len: buffer length
 *	@pg_ofst: buffer start offset into first page
 *	@color: buffer generation
 *
 *	Sets up a buffer for direct data placement.
 */
int cxgb4_setup_ddpbuf(struct pci_dev *pdev, const dma_addr_t *bus_addr,
		       unsigned int naddr, unsigned int tid, unsigned int tag,
		       unsigned int len, unsigned int pg_ofst,
		       unsigned int color)
{
	__be64 w0, w1;
	unsigned int n;
	struct adapter *adap = pci_get_drvdata(pdev);
	volatile void __iomem *addr = adap->regs + MEMWIN0_BASE;

	w0 = cpu_to_be64(F_PPOD_VALID | V_PPOD_TID(tid) | V_PPOD_TAG(tag) |
			 V_PPOD_COLOR(color));
	w1 = cpu_to_be64(V_PPOD_LEN(len) | V_PPOD_OFST(pg_ofst));
	n = tag * sizeof(struct pagepod) + adap->vres.ddp.start;

	spin_lock(&adap->win0_lock);

	/* move window to first pagepod */
	t4_write_reg(adap, PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_OFFSET, 0), n);
	t4_read_reg(adap, PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_OFFSET, 0));

	for ( ; naddr; naddr -= PPOD_PAGES) {
		writeq(w0, addr);
		writeq(w1, addr + 8);
		writeq(0, addr + 16);
		addr += 24;

		n = min(naddr, PPOD_PAGES + 1);
		for ( ; n; n--, addr += 8, bus_addr++)
			writeq(cpu_to_be64(*bus_addr), addr);
		if (naddr <= PPOD_PAGES) {
			for ( ; naddr <= PPOD_PAGES; naddr++, addr += 8)
				writeq(0, addr);
			break;
		}
		bus_addr--;
	}
	t4_read_reg(adap, MEMWIN0_BASE);   /* flush */
	spin_unlock(&adap->win0_lock);
	return 0;
}
EXPORT_SYMBOL(cxgb4_setup_ddpbuf);

#define __DBG_MEMWIN_ISCSI_PAGEPOD__
#define MAX_PPOD_PER_PCIE_MEMWIN   (MEMWIN0_APERTURE/sizeof(struct pagepod))
/**
 *	cxgb4_setup_iscsi_pagepod - set up iscsi DDP pagepods
 *	@pdev: the PCI device
 *	@ppod_hdr: the pagepod header settings
 *	@bus_addr: bus addresses of pages making up the buffer
 *	@naddr: number of entries in @bus_addr
 *	@idx: pagepod index
 *	@max: # of pagepod to be written
 *
 *	Sets up iscsi pagepods for iscsi direct data placement.
 */
int cxgb4_setup_iscsi_pagepod(struct pci_dev *pdev, void *ppod_hdr,
				dma_addr_t *bus_addr, unsigned int naddr,
				unsigned int idx, unsigned int max)
{
	struct pagepod *ppod = (struct pagepod *)ppod_hdr;
	struct adapter *adap = pci_get_drvdata(pdev);
	unsigned int pidx = 0;
	int memwin_loop = (max + MAX_PPOD_PER_PCIE_MEMWIN - 1) /
				MAX_PPOD_PER_PCIE_MEMWIN;
	int i, j, k;
	int err = 0;

	if (!bus_addr || !naddr) {
		printk(KERN_ERR "%s: bus addr 0x%p, naddr %u INVALID.\n",
			__func__, bus_addr, naddr);
		return -EINVAL;
	}

	if ((max * 4) < naddr) {
		printk(KERN_ERR "%s: max ppod %u * 4 < # of bus_addr %u.\n",
			__func__, max, naddr);
		return -EINVAL;
	}

	spin_lock(&adap->win0_lock);

	for (i = 0; i < memwin_loop; i++) {
		unsigned int addr = MEMWIN0_BASE;
                unsigned int ppod_start = idx * sizeof(struct pagepod) +
					adap->vres.iscsi.start;
		unsigned int nppod = max < MAX_PPOD_PER_PCIE_MEMWIN ?
					max : MAX_PPOD_PER_PCIE_MEMWIN;
#ifdef __DBG_MEMWIN_ISCSI_PAGEPOD__
		unsigned int pidx_s = pidx;
		unsigned int addr_s = addr;
#endif

		t4_write_reg(adap,
			PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_OFFSET, 0),
			ppod_start);

		t4_read_reg(adap,
			PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_OFFSET, 0));

		max -= nppod;
                idx += nppod;

		for (j = 0; j < nppod; j++) {
			/* ppod header */
			t4_write_reg64(adap, addr,
					ppod->vld_tid_pgsz_tag_color);
			addr += 8;
			t4_write_reg64(adap, addr, ppod->len_offset);
			addr += 8;
			t4_write_reg64(adap, addr, 0ULL);
			addr += 8;

			for (k = 0; k < (PPOD_PAGES + 1);
				k++, pidx++, addr+=8) {
				if (pidx < naddr) {
					if (!(bus_addr[pidx])) {
						printk(KERN_ERR
						"%s: bus addr %u INVALID.\n",
						__func__, pidx);
						err = -EINVAL;
						goto done;
					}
					t4_write_reg64(adap, addr,
						cpu_to_be64(bus_addr[pidx]));
				} else 
					t4_write_reg64(adap, addr, 0ULL);
			}	
			pidx--;
		}

		/* flush */
		t4_read_reg(adap, MEMWIN0_BASE);

#ifdef __DBG_MEMWIN_ISCSI_PAGEPOD__
		/* read it back */
		for (j = 0; j < nppod; j++) {
			u64 val;

			/* ppod header */
			val = t4_read_reg64(adap, addr_s);
			if (val != ppod->vld_tid_pgsz_tag_color) {
				printk(KERN_ERR
				"%s: loop %d, nppod %d, vld 0x%llx/0x%llx.\n",
				__func__, i, j, val,
				ppod->vld_tid_pgsz_tag_color);
				err = -EIO;
			}
			addr_s += 8;

			val = t4_read_reg64(adap, addr_s);
			if (val != ppod->len_offset) {
				printk(KERN_ERR
				"%s: loop %d, nppod %d, len 0x%llx/0x%llx.\n",
				__func__, i, j, val, ppod->len_offset);
				err = -EIO;
			}
			addr_s += 8;

			val = t4_read_reg64(adap, addr_s);
			if (val) {
				printk(KERN_ERR
				"%s: loop %d, nppod %d, rsvd 0x%llx.\n",
				__func__, i, j, val);
				err = -EIO;
			}
			addr_s += 8;

			for (k = 0; k < (PPOD_PAGES + 1);
				k++, pidx_s++, addr_s += 8) {
				val = t4_read_reg64(adap, addr_s);
				if (pidx_s < naddr) {
					val = be64_to_cpu(val);
					if (val != bus_addr[pidx_s]) {
						printk(KERN_ERR
						"%s: loop %d, nppod %d, "
						"page %d, pidx %u, 0x%llx"
						" != 0x%llx.\n",
						__func__, i, j, k, pidx_s,
						val, bus_addr[pidx_s]);
						err = -EIO;
					}
				} else if (val) {
					printk(KERN_ERR
					"%s: loop %d, nppod %d, page %d, "
					"pidx %u/%u, 0x%llx non-zero.\n",
					__func__, i, j, k, pidx_s, naddr, val);
					err = -EIO;
				}
			}
			pidx_s--;

			if (err < 0)
				goto done;
		}
#endif
	}

done:
	spin_unlock(&adap->win0_lock);
	return err;
}
EXPORT_SYMBOL(cxgb4_setup_iscsi_pagepod);

/**
 *	cxgb4_clear_iscsi_pagepod - clear iscsi DDP pagepods
 *	@pdev: the PCI device
 *	@idx: pagepod index
 *	@max: # of pagepod to be cleared
 *
 *	clear settings for iscsi direct data placement.
 */
int cxgb4_clear_iscsi_pagepod(struct pci_dev *pdev,
				unsigned int idx, unsigned int max)
{
	struct adapter *adap = pci_get_drvdata(pdev);
	int memwin_loop = (max + MAX_PPOD_PER_PCIE_MEMWIN - 1) /
				MAX_PPOD_PER_PCIE_MEMWIN;
	int i, j;
	int err = 0;

	spin_lock(&adap->win0_lock);

	for (i = 0; i < memwin_loop; i++) {
		unsigned int addr = MEMWIN0_BASE;
                unsigned int ppod_start = idx * sizeof(struct pagepod) +
					adap->vres.iscsi.start;
		unsigned int nppod = max < MAX_PPOD_PER_PCIE_MEMWIN ?
					max : MAX_PPOD_PER_PCIE_MEMWIN;
#ifdef __DBG_MEMWIN_ISCSI_PAGEPOD__
		unsigned int addr_s = addr;
#endif

		t4_write_reg(adap,
			PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_OFFSET, 0),
			ppod_start);

		t4_read_reg(adap,
			PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_OFFSET, 0));

		max -= nppod;
                idx += nppod;

		for (j = 0; j < nppod; j++, addr += 64)
			t4_write_reg64(adap, addr, 0ULL);

		/* flush */
		t4_read_reg(adap, MEMWIN0_BASE);

#ifdef __DBG_MEMWIN_ISCSI_PAGEPOD__
		/* read it back */
		for (j = 0; j < nppod; j++, addr_s += 64) {
			u64 val;

			val = t4_read_reg64(adap, addr_s);
			if (val) {
				printk(KERN_ERR
				"%s: loop %d, nppod %d, clr 0x%llx non-zero.\n",
				__func__, i, j, val);
				err = -EIO;
				goto done;
			}
		}
#endif
	}

done:
	spin_unlock(&adap->win0_lock);
	return err;
}
EXPORT_SYMBOL(cxgb4_clear_iscsi_pagepod);

static ssize_t reg_attr_show(struct device *d, char *buf, int reg, int shift,
			     unsigned int mask)
{
	ssize_t len;
	unsigned int v;
	struct adapter *adap = netdev2adap(to_net_dev(d));

	/* Synchronize with ioctls that may shut down the device */
	mutex_lock(&adap->user_mutex);
	v = t4_read_reg(adap, reg);
	len = sprintf(buf, "%u\n", (v >> shift) & mask);
	mutex_unlock(&adap->user_mutex);
	return len;
}

static ssize_t reg_attr_store(struct device *d, const char *buf, size_t len,
			      int reg, int shift, unsigned int mask,
			      unsigned int min_val, unsigned int max_val)
{
	char *endp;
	unsigned int val;
	struct adapter *adap;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	val = simple_strtoul(buf, &endp, 0);
	if (endp == buf || val < min_val || val > max_val)
		return -EINVAL;

	adap = netdev2adap(to_net_dev(d));
	mutex_lock(&adap->user_mutex);
	t4_set_reg_field(adap, reg, mask << shift, val << shift);
	mutex_unlock(&adap->user_mutex);
	return len;
}

#define T4_REG_SHOW(name, reg, shift, mask) \
static ssize_t show_##name(struct device *d, struct device_attribute *attr, \
			   char *buf) \
{ \
	return reg_attr_show(d, buf, reg, shift, mask); \
}

#define T4_REG_STORE(name, reg, shift, mask, min_val, max_val) \
static ssize_t store_##name(struct device *d, struct device_attribute *attr, \
			    const char *buf, size_t len) \
{ \
	return reg_attr_store(d, buf, len, reg, shift, mask, min_val, max_val); \
}

#define T4_ATTR(name, reg, shift, mask, min_val, max_val) \
T4_REG_SHOW(name, reg, shift, mask) \
T4_REG_STORE(name, reg, shift, mask, min_val, max_val) \
static DEVICE_ATTR(name, S_IRUGO | S_IWUSR, show_##name, store_##name)

T4_ATTR(tcp_retries1, A_TP_SHIFT_CNT, S_RXTSHIFTMAXR1, M_RXTSHIFTMAXR1, 3, 15);
T4_ATTR(tcp_retries2, A_TP_SHIFT_CNT, S_RXTSHIFTMAXR2, M_RXTSHIFTMAXR2, 0, 15);
T4_ATTR(tcp_syn_retries, A_TP_SHIFT_CNT, S_SYNSHIFTMAX, M_SYNSHIFTMAX, 0, 15);
T4_ATTR(tcp_keepalive_probes, A_TP_SHIFT_CNT, S_KEEPALIVEMAXR2,
	M_KEEPALIVEMAXR2, 1, 15);

static ssize_t timer_attr_show(struct device *d, char *buf, int reg)
{
	ssize_t len;
	unsigned int v, tps;
	struct adapter *adap = netdev2adap(to_net_dev(d));

	/* Synchronize with ioctls that may shut down the device */
	mutex_lock(&adap->user_mutex);
	v = t4_read_reg(adap, reg);
	tps = (adap->params.vpd.cclk * 1000) >> adap->params.tp.tre;
	len = sprintf(buf, "%u\n", v / tps);
	mutex_unlock(&adap->user_mutex);
	return len;
}

static ssize_t timer_attr_store(struct device *d, const char *buf, size_t len,
				int reg, unsigned int min_val,
				unsigned int max_val)
{
	char *endp;
	unsigned int val, tps;
	struct adapter *adap;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	adap = netdev2adap(to_net_dev(d));
	tps = (adap->params.vpd.cclk * 1000) >> adap->params.tp.tre;
	val = simple_strtoul(buf, &endp, 0);
	if (endp == buf || val * tps < min_val || val * tps > max_val)
		return -EINVAL;

	mutex_lock(&adap->user_mutex);
	t4_write_reg(adap, reg, val * tps);
	mutex_unlock(&adap->user_mutex);
	return len;
}

#define T4_TIMER_REG_SHOW(name, reg) \
static ssize_t show_##name(struct device *d, struct device_attribute *attr, \
			   char *buf) \
{ \
	return timer_attr_show(d, buf, reg); \
}

#define T4_TIMER_REG_STORE(name, reg, min_val, max_val) \
static ssize_t store_##name(struct device *d, struct device_attribute *attr, \
			    const char *buf, size_t len) \
{ \
	return timer_attr_store(d, buf, len, reg, min_val, max_val); \
}

#define T4_TIMER_ATTR(name, reg, min_val, max_val) \
T4_TIMER_REG_SHOW(name, reg) \
T4_TIMER_REG_STORE(name, reg, min_val, max_val) \
static DEVICE_ATTR(name, S_IRUGO | S_IWUSR, show_##name, store_##name)

T4_TIMER_ATTR(tcp_keepalive_time, A_TP_KEEP_IDLE, 0, M_KEEPALIVEIDLE);
T4_TIMER_ATTR(tcp_keepalive_intvl, A_TP_KEEP_INTVL, 0, M_KEEPALIVEINTVL);
T4_TIMER_ATTR(tcp_finwait2_timeout, A_TP_FINWAIT2_TIMER, 0, M_FINWAIT2TIME);

static struct attribute *offload_attrs[] = {
	&dev_attr_tcp_retries1.attr,
	&dev_attr_tcp_retries2.attr,
	&dev_attr_tcp_syn_retries.attr,
	&dev_attr_tcp_keepalive_probes.attr,
	&dev_attr_tcp_keepalive_time.attr,
	&dev_attr_tcp_keepalive_intvl.attr,
	&dev_attr_tcp_finwait2_timeout.attr,
	NULL
};

static struct attribute_group offload_attr_group = { .attrs = offload_attrs };

/*
 * Dummy handler for Rx offload packets in case an offload module hasn't
 * registered yet.  We can get such packets in response to setting up filters.
 * We just drop them.
 */
static int rx_offload_blackhole(struct toedev *dev, struct sk_buff **skbs,
				int n)
{
	while (n--)
		dev_kfree_skb_any(skbs[n]);
	return 0;
}

/*
 * Other dummy ops usually overwritten by offload modules when they attach.
 */
static int dummy_can_offload(struct toedev *dev, struct sock *sk)
{
	return 0;
}

static int dummy_connect(struct toedev *dev, struct sock *sk,
			 struct net_device *edev)
{
	return -1;
}

static void dummy_neigh_update(struct toedev *dev, struct neighbour *neigh)
{
}

static void set_dummy_ops(struct toedev *dev)
{
	dev->can_offload  = dummy_can_offload;
	dev->connect      = dummy_connect;
	dev->neigh_update = dummy_neigh_update;
	dev->recv         = rx_offload_blackhole;
}

#endif /* CONFIG_CHELSIO_T4_OFFLOAD */

/**
 *	cxgb4_best_mtu - find the entry in the MTU table closest to an MTU
 *	@mtus: the HW MTU table
 *	@mtu: the target MTU
 *	@idx: index of selected entry in the MTU table
 *
 *	Returns the index and the value in the HW MTU table that is closest to
 *	but does not exceed @mtu, unless @mtu is smaller than any value in the
 *	table, in which case that smallest available value is selected.
 */
unsigned int cxgb4_best_mtu(const unsigned short *mtus, unsigned short mtu,
			    unsigned int *idx)
{
	unsigned int i = 0;

	while (i < NMTUS - 1 && mtus[i + 1] <= mtu)
		++i;
	if (idx)
		*idx = i;
	return mtus[i];
}
EXPORT_SYMBOL(cxgb4_best_mtu);

/**
 *	cxgb4_port_chan - get the HW channel of a port
 *	@dev: the net device for the port
 *
 *	Return the HW Tx channel of the given port.
 */
unsigned int cxgb4_port_chan(const struct net_device *dev)
{
	return netdev2pinfo(dev)->tx_chan;
}
EXPORT_SYMBOL(cxgb4_port_chan);

/**
 *	cxgb4_port_viid - get the VI id of a port
 *	@dev: the net device for the port
 *
 *	Return the VI id of the given port.
 */
unsigned int cxgb4_port_viid(const struct net_device *dev)
{
	return netdev2pinfo(dev)->viid;
}
EXPORT_SYMBOL(cxgb4_port_viid);

/**
 *	cxgb4_port_idx - get the index of a port
 *	@dev: the net device for the port
 *
 *	Return the index of the given port.
 */
unsigned cxgb4_port_idx(const struct net_device *dev)
{
	return netdev2pinfo(dev)->port_id;
}
EXPORT_SYMBOL(cxgb4_port_idx);

/**
 *	cxgb4_netdev_by_hwid - return the net device of a HW port
 *	@pdev: identifies the adapter
 *	@id: the HW port id
 *
 *	Return the net device associated with the interface with the given HW
 *	id.
 */
struct net_device *cxgb4_netdev_by_hwid(struct pci_dev *pdev, unsigned int id)
{
	const struct adapter *adap = pci_get_drvdata(pdev);

	if (!adap || id >= NCHAN)
		return NULL;
	id = adap->chan_map[id];
	return id < MAX_NPORTS ? adap->port[id] : NULL;
}
EXPORT_SYMBOL(cxgb4_netdev_by_hwid);

/**
 *	cxgb4_root_dev - get the root net_device of a physical net_device
 *	@dev: the net device
 *	@vlan: the VLAN id or -1 if we shouldn't traverse VLAN devices
 *
 *	Return the root bonding or VLAN device for the given device.
 *	It assumes that VLAN devices are layered on top of bonding devices.
 */
struct net_device *cxgb4_root_dev(struct net_device *dev, int vlan)
{
	struct vlan_group *grp = netdev2pinfo(dev)->vlan_grp;

	while (dev->master) {
		dev = dev->master;
#if 1
		grp = NULL;
#else
		grp = ((const struct bonding *)netdev_priv(dev))->vlgrp;
#endif
	}
	if (vlan >= 0)
		dev = grp ? vlan_group_get_device(grp, vlan) : NULL;
	return dev;
}
EXPORT_SYMBOL(cxgb4_root_dev);

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
void cxgb4_iscsi_init(struct net_device *dev, unsigned int tag_mask,
		      const unsigned int *pgsz_order)
{
	struct adapter *adap = netdev2adap(dev);

	t4_write_reg(adap, A_ULP_RX_ISCSI_TAGMASK, tag_mask);
	t4_write_reg(adap, A_ULP_RX_ISCSI_PSZ, V_HPZ0(pgsz_order[0]) |
		     V_HPZ1(pgsz_order[1]) | V_HPZ2(pgsz_order[2]) |
		     V_HPZ3(pgsz_order[3]));
}
EXPORT_SYMBOL(cxgb4_iscsi_init);

int cxgb4_wr_mbox(struct net_device *dev, const void *cmd,
		  int size, void *rpl)
{
	struct adapter *adap = netdev2adap(dev);

	return t4_wr_mbox(adap, adap->mbox, cmd, size, rpl);
}
EXPORT_SYMBOL(cxgb4_wr_mbox);

static struct pci_driver cxgb4_driver;

static void check_neigh_update(struct neighbour *neigh)
{
	const struct device *parent;
	const struct net_device *netdev = neigh->dev;

	if (netdev->priv_flags & IFF_802_1Q_VLAN)
		netdev = vlan_dev_real_dev(netdev);
	parent = netdev->dev.parent;
	if (parent && parent->driver == &cxgb4_driver.driver)
		t4_l2t_update(dev_get_drvdata(parent), neigh);
}

static int netevent_cb(struct notifier_block *nb, unsigned long event,
		       void *data)
{
	switch (event) {
	case NETEVENT_NEIGH_UPDATE :
		check_neigh_update(data);
		break;
	case NETEVENT_PMTU_UPDATE:
	case NETEVENT_REDIRECT:
	default:
		break;
	}
	return 0;
}

static bool netevent_registered;
static struct notifier_block cxgb4_netevent_nb = {
	.notifier_call = netevent_cb
};

static void uld_attach(struct adapter *adap, unsigned int uld)
{
	void *handle;
	struct cxgb4_lld_info lli;
	unsigned int s_qpp;
	unsigned short i;

	if (!is_offload(adap))
		return;

	lli.pdev = adap->pdev;
	lli.l2t = adap->l2t;
	lli.tids = &adap->tids;
	lli.ports = adap->port;
	lli.vr = &adap->vres;
	lli.mtus = adap->params.mtus;
	if (uld == CXGB4_ULD_RDMA) {
		lli.rxq_ids = adap->sge.rdma_rxq;
		lli.nrxq = adap->sge.rdmaqs;
	} else if (uld == CXGB4_ULD_ISCSI) {
		lli.rxq_ids = adap->sge.iscsi_rxq;
		lli.nrxq = adap->sge.niscsiq;
	} else if (uld == CXGB4_ULD_TOE) {
		lli.rxq_ids = adap->sge.ofld_rxq;
		lli.nrxq = adap->sge.ofldqsets;
	}
	lli.ntxq = adap->sge.ofldqsets;
	lli.nchan = adap->params.nports;
	lli.nports = adap->params.nports;
	lli.wr_cred = adap->params.ofldq_wr_cred;
	lli.adapter_type = adap->params.rev;
	lli.iscsi_iolen = G_MAXRXDATA(t4_read_reg(adap, A_TP_PARA_REG2));
	s_qpp = (S_QUEUESPERPAGEPF0 +
		 (S_QUEUESPERPAGEPF1 - S_QUEUESPERPAGEPF0) * adap->pf);
	lli.udb_density =
		1 << ((t4_read_reg(adap, A_SGE_EGRESS_QUEUES_PER_PAGE_PF)
		       >> s_qpp) & M_QUEUESPERPAGEPF0);
	lli.ucq_density =
		1 << ((t4_read_reg(adap, A_SGE_INGRESS_QUEUES_PER_PAGE_PF)
		       >> s_qpp) & M_QUEUESPERPAGEPF0);
	lli.filt_mode = tp_vlan_pri_map;
	
	for (i = 0; i < NCHAN; i++)
		lli.tx_modq[i] = adap->params.tp.tx_modq[i];
	lli.gts_reg = adap->regs + MYPF_REG(A_SGE_PF_GTS);
	lli.db_reg = adap->regs + MYPF_REG(A_SGE_PF_KDOORBELL);
	lli.fw_vers = adap->params.fw_vers;

	handle = ulds[uld].add(&lli);
	if (IS_ERR(handle)) {
		CH_WARN(adap, "could not attach to the %s driver, error %ld\n",
			uld_str[uld], PTR_ERR(handle));
		return;
	}

	adap->uld_handle[uld] = handle;

	if (!netevent_registered) {
		register_netevent_notifier(&cxgb4_netevent_nb);
		netevent_registered = true;
	}

	if (adap->flags & FULL_INIT_DONE)
		ulds[uld].state_change(handle, CXGB4_STATE_UP);
}

static void attach_ulds(struct adapter *adap)
{
	unsigned int i;

	mutex_lock(&uld_mutex);
	list_add_tail(&adap->list_node, &adapter_list);
	for (i = 0; i < CXGB4_ULD_MAX; i++)
		if (ulds[i].add)
			uld_attach(adap, i);
	mutex_unlock(&uld_mutex);
}

static void detach_ulds(struct adapter *adap)
{
	unsigned int i;

	mutex_lock(&uld_mutex);
	list_del(&adap->list_node);
	for (i = 0; i < CXGB4_ULD_MAX; i++)
		if (adap->uld_handle[i]) {
			ulds[i].state_change(adap->uld_handle[i],
					     CXGB4_STATE_DETACH);
			adap->uld_handle[i] = NULL;
		}
	if (netevent_registered && list_empty(&adapter_list)) {
		unregister_netevent_notifier(&cxgb4_netevent_nb);
		netevent_registered = false;
	}
	mutex_unlock(&uld_mutex);
}

static void notify_ulds(struct adapter *adap, enum cxgb4_state new_state)
{
	unsigned int i;

	mutex_lock(&uld_mutex);
	for (i = 0; i < CXGB4_ULD_MAX; i++)
		if (adap->uld_handle[i])
			ulds[i].state_change(adap->uld_handle[i], new_state);
	mutex_unlock(&uld_mutex);
}

/**
 *	cxgb4_register_uld - register an upper-layer driver
 *	@type: the ULD type
 *	@p: the ULD methods
 *
 *	Registers an upper-layer driver with this driver and notifies the ULD
 *	about any presently available devices that support its type.  Returns
 *	%-EBUSY if a ULD of the same type is already registered.
 */
int cxgb4_register_uld(enum cxgb4_uld type, const struct cxgb4_uld_info *p)
{
	int ret = 0;
	struct adapter *adap;

	if (type >= CXGB4_ULD_MAX)
		return -EINVAL;
	mutex_lock(&uld_mutex);
	if (ulds[type].add) {
		ret = -EBUSY;
		goto out;
	}
	ulds[type] = *p;
	list_for_each_entry(adap, &adapter_list, list_node)
		uld_attach(adap, type);
out:	mutex_unlock(&uld_mutex);
	return ret;
}
EXPORT_SYMBOL(cxgb4_register_uld);

/**
 *	cxgb4_unregister_uld - unregister an upper-layer driver
 *	@type: the ULD type
 *
 *	Unregisters an existing upper-layer driver.
 */
int cxgb4_unregister_uld(enum cxgb4_uld type)
{
	struct adapter *adap;

	if (type >= CXGB4_ULD_MAX)
		return -EINVAL;
	mutex_lock(&uld_mutex);
	list_for_each_entry(adap, &adapter_list, list_node)
		adap->uld_handle[type] = NULL;
	ulds[type].add = NULL;
	mutex_unlock(&uld_mutex);
	return 0;
}
EXPORT_SYMBOL(cxgb4_unregister_uld);

#endif /* CONFIG_CHELSIO_T4_OFFLOAD */

/**
 *	cxgb_up - enable the adapter
 *	@adap: adapter being enabled
 *
 *	Called when the first port is enabled, this function performs the
 *	actions necessary to make an adapter operational, such as completing
 *	the initialization of HW modules, and enabling interrupts.
 *
 *	Must be called with the rtnl lock held.
 */
static int cxgb_up(struct adapter *adap)
{
	int err;

	err = setup_sge_queues(adap);
	if (err)
		goto out;
	err = setup_rss(adap);
	if (err)
		goto freeq;
	if (is_offload(adap))
		setup_loopback(adap);

	if (adap->flags & USING_MSIX) {
		name_msix_vecs(adap);
		err = request_irq(adap->msix_info[0].vec, t4_nondata_intr, 0,
				  adap->msix_info[0].desc, adap);
		if (err)
			goto irq_err;

		err = request_msix_queue_irqs(adap);
		if (err) {
			free_irq(adap->msix_info[0].vec, adap);
			goto irq_err;
		}
	} else {
		err = request_irq(adap->pdev->irq, t4_intr_handler(adap),
				  (adap->flags & USING_MSI) ? 0 : IRQF_SHARED,
				  adap->name, adap);
		if (err)
			goto irq_err;
	}

	enable_rx(adap);
	t4_sge_start(adap);
	t4_intr_enable(adap);
	adap->flags |= FULL_INIT_DONE;
#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	notify_ulds(adap, CXGB4_STATE_UP);
#endif
 out:
	return err;
 irq_err:
	CH_ERR(adap, "request_irq failed, err %d\n", err);
 freeq:
	t4_free_sge_resources(adap);
	goto out;
}

/*
 * Release resources when all the ports and offloading have been stopped.
 */
static void cxgb_down(struct adapter *adapter)
{
	t4_intr_disable(adapter);
#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	cancel_work_sync(&adapter->tid_release_task);
#endif

	if (adapter->flags & USING_MSIX) {
		free_msix_queue_irqs(adapter);
		free_irq(adapter->msix_info[0].vec, adapter);
	} else
		free_irq(adapter->pdev->irq, adapter);
	quiesce_rx(adapter);
	t4_sge_stop(adapter);
	t4_free_sge_resources(adapter);
	adapter->flags &= ~FULL_INIT_DONE;
}

static int cxgb_open(struct net_device *dev)
{
	int err;
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	if (!(adapter->flags & FULL_INIT_DONE)) {
		err = cxgb_up(adapter);
		if (err < 0)
			return err;
	}

	err = link_start(dev);
	if (err)
		return err;

	netif_tx_start_all_queues(dev);

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	if (test_bit(OFFLOAD_DEVMAP_BIT, &adapter->registered_device_map))
		netdev_set_offload(dev);
#endif
	return 0;
}

static int cxgb_close(struct net_device *dev)
{
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	netif_tx_stop_all_queues(dev);
	netif_carrier_off(dev);
	return t4_enable_vi(adapter, adapter->mbox, pi->viid, false, false);
}

#ifdef CONFIG_CHELSIO_T4_OFFLOAD

static int toe_ctl(struct toedev *tdev, unsigned int req, void *data)
{
	return -EOPNOTSUPP;
}

static void __devinit setup_offload(struct adapter *adapter)
{
	struct toedev *tdev = &adapter->tdev;

	init_offload_dev(tdev);
	set_dummy_ops(tdev);
	tdev->nlldev = adapter->params.nports;
	tdev->lldev = adapter->port;
	tdev->ctl = toe_ctl;
	tdev->ttid = TOE_ID_CHELSIO_T4;
}

#define ERR(fmt, ...) do {\
	printk(KERN_ERR "%s: " fmt "\n", dev->name, ## __VA_ARGS__); \
	return -EINVAL; \
} while (0)

/*
 * Perform device independent validation of offload policy.
 */
static int validate_offload_policy(const struct net_device *dev,
				   const struct ofld_policy_file *f,
				   size_t len)
{
	int i, inst;
	const u32 *p;
	const struct ofld_prog_inst *pi;

	/*
	 * We validate the following:
	 * - Program sizes match what's in the header
	 * - Branch targets are within the program
	 * - Offsets do not step outside struct offload_req
	 * - Outputs are valid
	 */
	printk(KERN_DEBUG "version %u, program length %zu bytes, alternate "
	       "program length %zu bytes\n", f->vers,
	       f->prog_size * sizeof(*pi), f->opt_prog_size * sizeof(*p));

	if (sizeof(*f) + (f->nrules + 1) * sizeof(struct offload_settings) +
	    f->prog_size * sizeof(*pi) + f->opt_prog_size * sizeof(*p) != len)
		ERR("bad offload policy length %zu", len);

	if (f->output_everything >= 0 && f->output_everything > f->nrules)
		ERR("illegal output_everything %d in header",
		    f->output_everything);

	pi = f->prog;

	for (i = 0; i < f->prog_size; i++, pi++) {
		if (pi->offset < 0 ||
		    pi->offset >= sizeof(struct offload_req) / 4)
			ERR("illegal offset %d at instruction %d", pi->offset,
			    i);
		if (pi->next[0] < 0 && -pi->next[0] > f->nrules)
			ERR("illegal output %d at instruction %d",
			    -pi->next[0], i);
		if (pi->next[1] < 0 && -pi->next[1] > f->nrules)
			ERR("illegal output %d at instruction %d",
			    -pi->next[1], i);
		if (pi->next[0] > 0 && pi->next[0] >= f->prog_size)
			ERR("illegal branch target %d at instruction %d",
			    pi->next[0], i);
		if (pi->next[1] > 0 && pi->next[1] >= f->prog_size)
			ERR("illegal branch target %d at instruction %d",
			    pi->next[1], i);
	}

	p = (const u32 *)pi;

	for (inst = i = 0; i < f->opt_prog_size; inst++) {
		unsigned int off = *p & 0xffff, nvals = *p >> 16;

		if (off >= sizeof(struct offload_req) / 4)
			ERR("illegal offset %u at opt instruction %d",
			    off, inst);
		if ((int32_t)p[1] < 0 && -p[1] > f->nrules)
			ERR("illegal output %d at opt instruction %d",
			    -p[1], inst);
		if ((int32_t)p[2] < 0 && -p[2] > f->nrules)
			ERR("illegal output %d at opt instruction %d",
			    -p[2], inst);
		if ((int32_t)p[1] > 0 && p[1] >= f->opt_prog_size)
			ERR("illegal branch target %d at opt instruction %d",
			    p[1], inst);
		if ((int32_t)p[2] > 0 && p[2] >= f->opt_prog_size)
			ERR("illegal branch target %d at opt instruction %d",
			    p[2], inst);
		p += 4 + nvals;
		i += 4 + nvals;
		if (i > f->opt_prog_size)
			ERR("too many values %u for opt instruction %d",
			    nvals, inst);
	}

	return 0;
}

#undef ERR

static int validate_policy_settings(const struct net_device *dev,
				    struct adapter *adap,
				    const struct ofld_policy_file *f)
{
	int i, nchan = adap->params.nports;
	const u32 *op = (const u32 *)&f->prog[f->prog_size];
	const struct offload_settings *s = (void *)&op[f->opt_prog_size];

	for (i = 0; i <= f->nrules; i++, s++) {
		if (s->cong_algo > 3) {
			printk(KERN_ERR "%s: illegal congestion algorithm %d\n",
			       dev->name, s->cong_algo);
			return -EINVAL;
		}
		if (s->rssq >= adap->sge.ofldqsets / nchan) {
			printk(KERN_ERR "%s: illegal RSS queue %d\n", dev->name,
			       s->rssq);
			return -EINVAL;
		}
		if (s->sched_class >= NTX_SCHED / nchan) {
			printk(KERN_ERR "%s: illegal scheduling class %d\n",
			       dev->name, s->sched_class);
			return -EINVAL;
		}
	}
	return 0;
}

#endif /* CONFIG_CHELSIO_T4_OFFLOAD */

/*
 * driver-specific ioctl support
 */

/* clear statistics for the given Ethernet Tx and Rx queues */
static void clear_ethq_stats(struct sge *p, unsigned int idx)
{
	struct sge_eth_rxq *rxq = &p->ethrxq[idx];
	struct sge_eth_txq *txq = &p->ethtxq[idx];

	memset(&rxq->stats, 0, sizeof(rxq->stats));
	rxq->fl.alloc_failed = rxq->fl.large_alloc_failed = 0;
	rxq->fl.starving = 0;

	txq->tso = txq->tx_cso = txq->vlan_ins = 0;
	txq->q.stops = txq->q.restarts = 0;
	txq->mapping_err = 0;
}

/* clear statistics for the Ethernet queues associated with the given port */
static void clear_port_qstats(struct adapter *adap, const struct port_info *pi)
{
	int i;

	for (i = 0; i < pi->nqsets; i++)
		clear_ethq_stats(&adap->sge, pi->first_qset + i);
}

/**
 *	t4_get_desc - dump an SGE descriptor for debugging purposes
 *	@p: points to the sge structure for the adapter
 *	@category: the type of queue
 *	@qid: the absolute SGE QID of the specific queue within the category
 *	@idx: the descriptor index in the queue
 *	@data: where to dump the descriptor contents
 *
 *	Dumps the contents of a HW descriptor of an SGE queue.  Returns the
 *	size of the descriptor or a negative error.
 */
static int get_qdesc(const struct sge *p, int category, unsigned int qid,
		     unsigned int idx, unsigned char *data)
{
	int i, len = sizeof(struct tx_desc);

	/*
	 * For Tx queues allow reading the status entry too.
	 */
	if (category == SGE_QTYPE_TX_ETH) {
		const struct sge_eth_txq *q = p->ethtxq;

		for (i = 0; i < ARRAY_SIZE(p->ethtxq); i++, q++)
			if (q->q.cntxt_id == qid && q->q.desc &&
			    idx <= q->q.size) {
				memcpy(data, &q->q.desc[idx], len);
				return len;
			}
	}
	if (category == SGE_QTYPE_TX_OFLD) {
		const struct sge_ofld_txq *q = p->ofldtxq;

		for (i = 0; i < ARRAY_SIZE(p->ofldtxq); i++, q++)
			if (q->q.cntxt_id == qid && q->q.desc &&
			    idx <= q->q.size) {
				memcpy(data, &q->q.desc[idx], len);
				return len;
			}
	}
	if (category == SGE_QTYPE_TX_CTRL) {
		const struct sge_ctrl_txq *q = p->ctrlq;

		for (i = 0; i < ARRAY_SIZE(p->ctrlq); i++, q++)
			if (q->q.cntxt_id == qid && q->q.desc &&
			    idx <= q->q.size) {
				memcpy(data, &q->q.desc[idx], len);
				return len;
			}
	}
	if (category == SGE_QTYPE_FL) {
		const struct sge_fl *q;

		qid -= p->egr_start;
		q = qid >= ARRAY_SIZE(p->egr_map) ? NULL : p->egr_map[qid];
		if (q && q >= &p->ethrxq[0].fl && idx < q->size) {
			*(__be64 *)data = q->desc[idx];
			return sizeof(u64);
		}
	}
	if (category == SGE_QTYPE_RSP) {
		const struct sge_rspq *q;

		qid -= p->ingr_start;
		q = qid >= ARRAY_SIZE(p->ingr_map) ? NULL : p->ingr_map[qid];
		if (q && idx < q->size) {
			len = q->iqe_len;
			idx *= len / sizeof(u64);
			memcpy(data, &q->desc[idx], len);
			return len;
		}
	}
	return -EINVAL;
}

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
/*
 * Return an error number if the indicated filter isn't writable ...
 */
static int writable_filter(struct filter_entry *f)
{
	if (f->locked)
		return -EPERM;
	if (f->pending)
		return -EBUSY;

	return 0;
}

/*
 * Delete the filter at the specified index (if valid).  The checks for all
 * the common problems with doing this like the filter being locked, currently
 * pending in another operation, etc.
 */
static int delete_filter(struct adapter *adapter, unsigned int fidx)
{
	struct filter_entry *f;
	int ret;

	if (fidx >= adapter->tids.nftids)
		return -EINVAL;

	f = &adapter->tids.ftid_tab[fidx];
	ret = writable_filter(f);
	if (ret)
		return ret;
	if (f->valid)
		return del_filter_wr(adapter, fidx);

	return 0;
}
#endif

/*
 * Simple predicate to vet incoming Chelsio ioctl() parameters to make sure
 * they are either not set (value < 0) or within the indicated range.
 */
static int in_range(int val, int lo, int hi)
{
	return val < 0 || (val <= hi && val >= lo);
}

static int cxgb_extension_ioctl(struct net_device *dev, void __user *useraddr)
{
	int ret;
	u32 cmd;
	struct adapter *adapter = netdev2adap(dev);

	if (copy_from_user(&cmd, useraddr, sizeof(cmd)))
		return -EFAULT;

	switch (cmd) {
	case CHELSIO_SETREG: {
		struct ch_reg edata;

		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (copy_from_user(&edata, useraddr, sizeof(edata)))
			return -EFAULT;
		if ((edata.addr & 3) != 0 ||
		    edata.addr >= pci_resource_len(adapter->pdev, 0))
			return -EINVAL;
		writel(edata.val, adapter->regs + edata.addr);
		break;
	}
	case CHELSIO_GETREG: {
		struct ch_reg edata;

		if (copy_from_user(&edata, useraddr, sizeof(edata)))
			return -EFAULT;
		if ((edata.addr & 3) != 0 ||
		    edata.addr >= pci_resource_len(adapter->pdev, 0))
			return -EINVAL;
		edata.val = readl(adapter->regs + edata.addr);
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		break;
	}
	case CHELSIO_GET_SGE_CTXT: {
		struct ch_mem_range t;
		u32 buf[SGE_CTXT_SIZE / 4];

		if (copy_from_user(&t, useraddr, sizeof(t)))
			return -EFAULT;
		if (t.len < SGE_CTXT_SIZE || t.addr > M_CTXTQID)
			return -EINVAL;

		if (t.mem_id == CNTXT_TYPE_RSP || t.mem_id == CNTXT_TYPE_CQ)
			ret = CTXT_INGRESS;
		else if (t.mem_id == CNTXT_TYPE_EGRESS)
			ret = CTXT_EGRESS;
		else if (t.mem_id == CNTXT_TYPE_FL)
			ret = CTXT_FLM;
		else if (t.mem_id == CNTXT_TYPE_CONG)
			ret = CTXT_CNM;
		else
			return -EINVAL;

		if (adapter->flags & FW_OK)
			ret = t4_sge_ctxt_rd(adapter, adapter->mbox, t.addr,
					     ret, buf);
		else
			ret = t4_sge_ctxt_rd_bd(adapter, t.addr, ret, buf);
		if (ret)
			return ret;

		t.version = mk_adap_vers(adapter);
		if (copy_to_user(useraddr + sizeof(t), buf, SGE_CTXT_SIZE) ||
		    copy_to_user(useraddr, &t, sizeof(t)))
			return -EFAULT;
		break;
	}
	case CHELSIO_GET_SGE_DESC2: {
		unsigned char buf[128];
		struct ch_mem_range edesc;

		if (copy_from_user(&edesc, useraddr, sizeof(edesc)))
			return -EFAULT;
		/*
		 * Upper 8 bits of mem_id is the queue type, the rest the qid.
		 */
		ret = get_qdesc(&adapter->sge, edesc.mem_id >> 24,
				edesc.mem_id & 0xffffff, edesc.addr, buf);
		if (ret < 0)
			return ret;
		if (edesc.len < ret)
			return -EINVAL;

		edesc.len = ret;
		edesc.version = mk_adap_vers(adapter);
		if (copy_to_user(useraddr + sizeof(edesc), buf, edesc.len) ||
		    copy_to_user(useraddr, &edesc, sizeof(edesc)))
			return -EFAULT;
		break;
	}
	case CHELSIO_SET_QSET_PARAMS: {
		struct sge_eth_rxq *rq;
		struct sge_eth_txq *tq;
		struct ch_qset_params t;
		const struct port_info *pi = netdev_priv(dev);

		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (copy_from_user(&t, useraddr, sizeof(t)))
			return -EFAULT;
		if (t.qset_idx >= pi->nqsets)
			return -EINVAL;
		if (t.txq_size[1] >= 0 || t.txq_size[2] >= 0 ||
		    t.fl_size[1] >= 0 || t.cong_thres >= 0 || t.polling >= 0)
			return -EINVAL;
		if (//!in_range(t.intr_lat, 0, M_NEWTIMER) ||
		    //!in_range(t.cong_thres, 0, 255) ||
		    !in_range(t.txq_size[0], MIN_TXQ_ENTRIES,
			      MAX_TXQ_ENTRIES) ||
		    !in_range(t.fl_size[0], MIN_FL_ENTRIES, MAX_RX_BUFFERS) ||
		    !in_range(t.rspq_size, MIN_RSPQ_ENTRIES, MAX_RSPQ_ENTRIES))
			return -EINVAL;

		if (t.lro > 0)
			return -EINVAL;

		if ((adapter->flags & FULL_INIT_DONE) &&
		    (t.rspq_size >= 0 || t.fl_size[0] >= 0 ||
		     t.txq_size[0] >= 0))
			return -EBUSY;

		tq = &adapter->sge.ethtxq[t.qset_idx + pi->first_qset];
		rq = &adapter->sge.ethrxq[t.qset_idx + pi->first_qset];

		if (t.rspq_size >= 0)
			rq->rspq.size = t.rspq_size;
		if (t.fl_size[0] >= 0)
			rq->fl.size = t.fl_size[0] + 8; /* need an empty desc */
		if (t.txq_size[0] >= 0)
			tq->q.size = t.txq_size[0];
		if (t.intr_lat >= 0)
			rq->rspq.intr_params =
				V_QINTR_TIMER_IDX(closest_timer(&adapter->sge, t.intr_lat));
		break;
	}
	case CHELSIO_GET_QSET_PARAMS: {
		struct sge_eth_rxq *rq;
		struct sge_eth_txq *tq;
		struct ch_qset_params t;
		const struct port_info *pi = netdev_priv(dev);

		if (copy_from_user(&t, useraddr, sizeof(t)))
			return -EFAULT;
		if (t.qset_idx >= pi->nqsets)
			return -EINVAL;

		tq = &adapter->sge.ethtxq[t.qset_idx + pi->first_qset];
		rq = &adapter->sge.ethrxq[t.qset_idx + pi->first_qset];
		t.rspq_size   = rq->rspq.size;
		t.txq_size[0] = tq->q.size;
		t.txq_size[1] = 0;
		t.txq_size[2] = 0;
		t.fl_size[0]  = rq->fl.size - 8; /* sub unused descriptor */
		t.fl_size[1]  = 0;
		t.polling     = 1;
		t.lro         = ((dev->features & NETIF_F_GRO) != 0);
		t.intr_lat    = qtimer_val(adapter, &rq->rspq);
		t.cong_thres  = 0;


		if (adapter->flags & USING_MSIX)
			t.vector = adapter->msix_info[pi->first_qset +
						      t.qset_idx + 2].vec;
		else
			t.vector = adapter->pdev->irq;

		if (copy_to_user(useraddr, &t, sizeof(t)))
			return -EFAULT;
		break;
	}
	case CHELSIO_SET_QSET_NUM: {
		struct ch_reg edata;
		struct port_info *pi = netdev_priv(dev);
		unsigned int i, first_qset = 0, other_qsets;

		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (adapter->flags & FULL_INIT_DONE)
			return -EBUSY;
		if (copy_from_user(&edata, useraddr, sizeof(edata)))
			return -EFAULT;

		/*
		 * Check legitimate range for number of Queue Sets.  We need
		 * at least one Queue Set and we can't have more that
		 * MAX_ETH_QSETS.  (Note that the incoming value from User
		 * Space is an unsigned 32-bit value.  Since that includes
		 * 0xffff == (u32)-1, if we depend solely on the test below
		 * for "edata.val + other_qsets > adapter->sge.max_ethqsets",
		 * then we'll miss such bad values because of wrap-around
		 * arithmetic.)
		 */
		if (edata.val < 1 || edata.val > MAX_ETH_QSETS)
			return -EINVAL;

		other_qsets = adapter->sge.ethqsets - pi->nqsets;
		if (edata.val + other_qsets > adapter->sge.max_ethqsets)
			return -EINVAL;
		pi->nqsets = edata.val;
		adapter->sge.ethqsets = other_qsets + pi->nqsets;

		for_each_port(adapter, i)
			if (adapter->port[i]) {
				pi = adap2pinfo(adapter, i);
				pi->first_qset = first_qset;
				first_qset += pi->nqsets;
			}
		break;
	}
	case CHELSIO_GET_QSET_NUM: {
		struct ch_reg edata;
		struct port_info *pi = netdev_priv(dev);

		edata.cmd = CHELSIO_GET_QSET_NUM;
		edata.val = pi->nqsets;
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		break;
	}
	case CHELSIO_LOAD_FW: {
		u8 *fw_data;
		struct ch_mem_range t;

		if (!capable(CAP_SYS_RAWIO))
			return -EPERM;
		if (copy_from_user(&t, useraddr, sizeof(t)))
			return -EFAULT;
		if (!t.len)
			return -EINVAL;

		fw_data = t4_alloc_mem(t.len);
		if (!fw_data)
			return -ENOMEM;

		if (copy_from_user(fw_data, useraddr + sizeof(t), t.len)) {
			t4_free_mem(fw_data);
			return -EFAULT;
		}

		ret = t4_load_fw(adapter, fw_data, t.len);
		t4_free_mem(fw_data);
		if (ret)
			return ret;
		break;
	}
	case CHELSIO_LOAD_BOOT: {
		u8 *boot_data;
		struct ch_mem_range t;
		unsigned int pcie_pf_exprom_ofst, offset;

		if (!capable(CAP_SYS_RAWIO))
			return -EPERM;
		if (copy_from_user(&t, useraddr, sizeof(t)))
			return -EFAULT;

		boot_data = t4_alloc_mem(t.len);
		if (!boot_data)
			return -ENOMEM;

		if (copy_from_user(boot_data, useraddr + sizeof(t), t.len)) {
			t4_free_mem(boot_data);
			return -EFAULT;
		}

		if (t.mem_id == 0) {
			/*
			 * Flash boot image to the offset defined by the PFs
			 * EXPROM_OFST defined in the serial configuration file.
			 * Read PCIE_PF_EXPROM_OFST register
		 	 */
			pcie_pf_exprom_ofst = t4_read_reg(adapter,
					PF_REG(t.addr, A_PCIE_PF_EXPROM_OFST));
			offset = G_OFFSET(pcie_pf_exprom_ofst);

		} else if (t.mem_id == 1) {
			/*
			 * Flash boot image to offset specified by the user.
			 */
			offset = G_OFFSET(t.addr);

		} else {
			t4_free_mem(boot_data);
			return -EINVAL;
		}

		ret = t4_load_boot(adapter, boot_data, offset, t.len);
		t4_free_mem(boot_data);
		if (ret)
			return ret;
		break;
	}
        case CHELSIO_LOAD_CFG: {
                u8 *cfg_data;
		struct struct_load_cfg t;
		

		if (!capable(CAP_SYS_RAWIO))
			return -EPERM;
                if (copy_from_user(&t, useraddr, sizeof(t)))
			return -EFAULT;

		cfg_data = t4_alloc_mem(t.len);
		if (!cfg_data)
			return -ENOMEM;

		if (copy_from_user(cfg_data, useraddr + sizeof(t), t.len)) {
			t4_free_mem(cfg_data);
			return -EFAULT;
		}
                ret = t4_load_cfg(adapter, cfg_data, t.len);
		t4_free_mem(cfg_data);
		if (ret)
			return ret;
		break;
        }

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	case CHELSIO_SET_FILTER: {
		u32 fconf, iconf;
		struct ch_filter t;
		unsigned int fidx, iq;
		struct filter_entry *f;

		/*
		 * Vet the filter specification against our hardware filter
		 * configuration and capabilities.
		 */

		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (adapter->tids.nftids == 0 ||
		    adapter->tids.ftid_tab == NULL)
			return -EOPNOTSUPP;
		if (!(adapter->flags & FULL_INIT_DONE))
			return -EAGAIN;  /* can still change nfilters */
		if (copy_from_user(&t, useraddr, sizeof(t)))
			return -EFAULT;

		if (t.filter_id >= adapter->tids.nftids)
			return -E2BIG;

		/*
		 * Check for unconfigured fields being used.
		 */
		t4_read_indirect(adapter, A_TP_PIO_ADDR, A_TP_PIO_DATA,
				 &fconf, 1, A_TP_VLAN_PRI_MAP);
		t4_read_indirect(adapter, A_TP_PIO_ADDR, A_TP_PIO_DATA,
				 &iconf, 1, A_TP_INGRESS_CONFIG);

		#define S(_field) \
			(t.fs.val._field || t.fs.mask._field)
		#define U(_mask, _field) \
			(!(fconf & (_mask)) && S(_field))

		if (U(F_FCOE, fcoe) || U(F_PORT, iport) || U(F_TOS, tos) ||
		    U(F_ETHERTYPE, ethtype) || U(F_MACMATCH, macidx) ||
		    U(F_MPSHITTYPE, matchtype) || U(F_FRAGMENTATION, frag) ||
		    U(F_PROTOCOL, proto) ||
		    U(F_VNIC_ID, pfvf_vld) ||
		    U(F_VNIC_ID, ovlan_vld) ||
		    U(F_VLAN, ivlan_vld))
			return -EOPNOTSUPP;

		/*
		 * T4 inconveniently uses the same 17 bits for both the Outer
		 * VLAN Tag and PF/VF/VFvld fields based on F_VNIC being set
		 * in TP_INGRESS_CONFIG.  Hense the somewhat crazy checks
		 * below.  Additionally, since the T4 firmware interface also
		 * carries that overlap, we need to translate any PF/VF
		 * specification into that internal format below.
		 */
		if (S(pfvf_vld) && S(ovlan_vld))
			return -EOPNOTSUPP;
		if ((S(pfvf_vld) && !(iconf & F_VNIC)) ||
		    (S(ovlan_vld) && (iconf & F_VNIC)))
			return -EOPNOTSUPP;
		if (t.fs.val.pf > 0x7 || t.fs.val.vf > 0x7f)
			return -ERANGE;
		t.fs.mask.pf &= 0x7;
		t.fs.mask.vf &= 0x7f;

		#undef S
		#undef U

		/*
		 * If the user is requesting that the filter action loop
		 * matching packets back out one of our ports, make sure that
		 * the egress port is in range.
		 */
		if (t.fs.action == FILTER_SWITCH &&
		    t.fs.eport >= adapter->params.nports)
			return -ERANGE;

		/*
		 * Don't allow various trivially obvious bogus out-of-range
		 * values ...
		 */
		if (t.fs.val.iport >= adapter->params.nports)
			return -ERANGE;

		/*
		 * If the user has requested steering matching Ingress Packets
		 * to a specific Queue Set, we need to make sure it's in range
		 * for the port and map that into the Absolute Queue ID of the
		 * Queue Set's Response Queue.
		 */
		if (!t.fs.dirsteer) {
			if (t.fs.iq)
				return -EINVAL;
			iq = 0;
		} else {
			struct port_info *pi = netdev_priv(dev);

			/*
			 * If the iq id is greater than the number of qsets,
			 * then assume it is an absolute qid.
			 */
			if (t.fs.iq < pi->nqsets)
				iq = adapter->sge.ethrxq[pi->first_qset +
							 t.fs.iq].rspq.abs_id;
			else
				iq = t.fs.iq;
		}

		/*
		 * IPv6 filters occupy four slots and must be aligned on
		 * four-slot boundaries.  IPv4 filters only occupy a single
		 * slot and have no alignment requirements but writing a new
		 * IPv4 filter into the middle of an existing IPv6 filter
		 * requires clearing the old IPv6 filter.
		 */
		if (t.fs.type == 0) { /* IPv4 */
			/*
			 * If our IPv4 filter isn't being written to a
			 * multiple of four filter index and there's an IPv6
			 * filter at the multiple of 4 base slot, then we need
			 * to delete that IPv6 filter ...
			 */
			fidx = t.filter_id & ~0x3;
			if (fidx != t.filter_id &&
			    adapter->tids.ftid_tab[fidx].fs.type) {
				ret = delete_filter(adapter, fidx);
				if (ret)
					return ret;
			}
		} else { /* IPv6 */
			/*
			 * Ensure that the IPv6 filter is aligned on a
			 * multiple of 4 boundary.
			 */
			if (t.filter_id & 0x3)
				return -EINVAL;

			/*
			 * Check all except the base overlapping IPv4 filter
			 * slots.
			 */
			for (fidx = t.filter_id+1; fidx < t.filter_id+4; fidx++) {
				ret = delete_filter(adapter, fidx);
				if (ret)
					return ret;
			}
		}

		/*
		 * Check to make sure the filter requested is writable ...
		 */
		f = &adapter->tids.ftid_tab[t.filter_id];
		ret = writable_filter(f);
		if (ret)
			return ret;

		/*
		 * Clear out any old resources being used by the filter before
		 * we start constructing the new filter.
		 */
		if (f->valid)
			clear_filter(adapter, f);

		/*
		 * Convert the filter specification into our internal format.
		 * We copy the PF/VF specification into the Outer VLAN field
		 * here so the rest of the code -- including the interface to
		 * the firmware -- doesn't have to constantly do these checks.
		 */
		f->fs = t.fs;
		f->fs.iq = iq;
		if (iconf & F_VNIC) {
			f->fs.val.ovlan = (t.fs.val.pf << 7) | t.fs.val.vf;
			f->fs.mask.ovlan = (t.fs.mask.pf << 7) | t.fs.mask.vf;
			f->fs.val.ovlan_vld = t.fs.val.pfvf_vld;
			f->fs.mask.ovlan_vld = t.fs.mask.pfvf_vld;
		}

		/*
		 * Attempt to set the filter.  If we don't succeed, we clear
		 * it and return the failure.
		 */
		ret = set_filter_wr(adapter, t.filter_id);
		if (ret) {
			clear_filter(adapter, f);
			return ret;
		}

		break;
	}
	case CHELSIO_DEL_FILTER: {
		struct ch_filter fs;

		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (adapter->tids.nftids == 0 ||
		    adapter->tids.ftid_tab == NULL)
			return -EOPNOTSUPP;
		if (!(adapter->flags & FULL_INIT_DONE))
			return -EAGAIN;  /* can still change nfilters */
		if (copy_from_user(&fs, useraddr, sizeof(fs)))
			return -EFAULT;

		/*
		 * Make sure this is a valid filter and that we can delete it.
		 */
		if (fs.filter_id >= adapter->tids.nftids)
			return -E2BIG;
printk ("ioctl() delete_filter %d\n", fs.filter_id);
		ret = delete_filter(adapter, fs.filter_id);
		break;
	}
#endif
	case CHELSIO_CLEAR_STATS: {
		struct ch_reg edata;
		struct port_info *pi = netdev_priv(dev);

		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (copy_from_user(&edata, useraddr, sizeof(edata)))
			return -EFAULT;
		if ((edata.val & STATS_QUEUE) && edata.addr != -1 &&
		    edata.addr >= pi->nqsets)
			return -EINVAL;
		if (edata.val & STATS_PORT) {
			t4_clr_port_stats(adapter, pi->tx_chan);
			clear_sge_port_stats(adapter, pi);
		}
		if (edata.val & STATS_QUEUE) {
			if (edata.addr == -1)
				clear_port_qstats(adapter, pi);
			else
				clear_ethq_stats(&adapter->sge,
						 pi->first_qset + edata.addr);
		}
		break;
	}
#ifdef CONFIG_CHELSIO_T4_OFFLOAD
#if 0
	case CHELSIO_DEVUP:
		if (!is_offload(adapter))
			return -EOPNOTSUPP;
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		return activate_offload(&adapter->tdev);
#endif
	case CHELSIO_SET_HW_SCHED: {
		struct ch_hw_sched t;
		unsigned int ticks_per_usec = core_ticks_per_usec(adapter);

		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (copy_from_user(&t, useraddr, sizeof(t)))
			return -EFAULT;
		if (t.sched >= NTX_SCHED || !in_range(t.mode, 0, 1) ||
		    !in_range(t.channel, 0, NCHAN-1) ||
		    !in_range(t.kbps, 0, 10000000) ||
		    !in_range(t.class_ipg, 0, 10000 * 65535 / ticks_per_usec) ||
		    !in_range(t.flow_ipg, 0,
			      dack_ticks_to_usec(adapter, 0x7ff)))
			return -EINVAL;

		if (t.kbps >= 0) {
			ret = t4_set_sched_bps(adapter, t.sched, t.kbps);
			if (ret < 0)
				return ret;
		}
		if (t.class_ipg >= 0)
			t4_set_sched_ipg(adapter, t.sched, t.class_ipg);
		if (t.flow_ipg >= 0) {
			ret = t4_set_pace_tbl(adapter, &t.flow_ipg, t.sched, 1);
			if (ret < 0)
				return ret;
		}
		if (t.mode >= 0) {
			int bit = 1 << (S_TIMERMODE + t.sched);

			t4_set_reg_field(adapter, A_TP_MOD_CONFIG,
					 bit, t.mode ? bit : 0);
		}
		if (t.channel >= 0) {
			t4_set_reg_field(adapter, A_TP_TX_MOD_QUEUE_REQ_MAP,
					 3 << (2 * t.sched),
					 t.channel << (2 * t.sched));
			adapter->params.tp.tx_modq[t.channel] = t.sched;
		}
		if (t.weight >= 0) {
			if (t.sched <= 3)
				t4_set_reg_field(adapter, A_TP_TX_MOD_QUEUE_WEIGHT0,
					0xFF << (8 * t.sched),
					t.weight << (8 * t.sched));
			if(t.sched > 3)
				t4_set_reg_field(adapter, A_TP_TX_MOD_QUEUE_WEIGHT1,
					0xFF << (8 * (t.sched - 4)),
					t.weight << (8 * (t.sched - 4)));
		}
		break;
	}
	case CHELSIO_SET_OFLD_POLICY: {
		struct ch_mem_range t;
		struct ofld_policy_file *opf;
		struct cxgb4_uld_info *toe_uld = &ulds[CXGB4_ULD_TOE];
		void *toe_handle = adapter->uld_handle[CXGB4_ULD_TOE];

		if (!test_bit(OFFLOAD_DEVMAP_BIT,
			      &adapter->registered_device_map))
			return -EOPNOTSUPP;
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (copy_from_user(&t, useraddr, sizeof(t)))
			return -EFAULT;
		if (!toe_uld->control || !TOEDEV(dev))
			return -EOPNOTSUPP;

		/* len == 0 removes any existing policy */
		if (t.len == 0) {
			toe_uld->control(toe_handle,
					 CXGB4_CONTROL_SET_OFFLOAD_POLICY,
					 NULL);
			break;
		}

		opf = t4_alloc_mem(t.len);
		if (!opf)
			return -ENOMEM;

		if (copy_from_user(opf, useraddr + sizeof(t), t.len)) {
			t4_free_mem(opf);
			return -EFAULT;
		}

		ret = validate_offload_policy(dev, opf, t.len);
		if (!ret)
			ret = validate_policy_settings(dev, adapter, opf);
		if (!ret)
			ret = toe_uld->control(toe_handle,
					       CXGB4_CONTROL_SET_OFFLOAD_POLICY,
					       opf);
		t4_free_mem(opf);
		return ret;
	}
#endif
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

/*
 * net_device operations
 */

/* IEEE 802.3 specified MDIO devices */
enum {
	MDIO_DEV_PMA_PMD = 1,
	MDIO_DEV_VEND2   = 31
};

static int cxgb_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
	int ret = 0, mmd;
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;
	struct mii_ioctl_data *data = (struct mii_ioctl_data *)&req->ifr_data;

	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = pi->mdio_addr;
		break;
	case SIOCGMIIREG: {
		u32 val;

		if (pi->link_cfg.supported & FW_PORT_CAP_SPEED_10G) {
			mmd = data->phy_id >> 8;
			if (!mmd)
				mmd = MDIO_DEV_PMA_PMD;
			else if (mmd > MDIO_DEV_VEND2)
				return -EINVAL;

			ret = t4_mdio_rd(adapter, adapter->mbox,
					 data->phy_id & 0x1f, mmd,
					 data->reg_num, &val);
		} else
			ret = t4_mdio_rd(adapter, adapter->mbox,
					 data->phy_id & 0x1f, 0,
					 data->reg_num & 0x1f, &val);
		if (!ret)
			data->val_out = val;
		break;
	}
	case SIOCSMIIREG:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (pi->link_cfg.supported & FW_PORT_CAP_SPEED_10G) {
			mmd = data->phy_id >> 8;
			if (!mmd)
				mmd = MDIO_DEV_PMA_PMD;
			else if (mmd > MDIO_DEV_VEND2)
				return -EINVAL;

			ret = t4_mdio_wr(adapter, adapter->mbox,
					 data->phy_id & 0x1f, mmd,
					 data->reg_num, data->val_in);
		} else
			ret = t4_mdio_wr(adapter, adapter->mbox,
					 data->phy_id & 0x1f, 0,
					 data->reg_num & 0x1f, data->val_in);
		break;

	case SIOCCHIOCTL:
		return cxgb_extension_ioctl(dev, (void *)req->ifr_data);
	default:
		return -EOPNOTSUPP;
	}
	return ret;
}

static int cxgb_change_mtu(struct net_device *dev, int new_mtu)
{
	int ret;
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	if (new_mtu < 81)         /* accommodate SACK */
		return -EINVAL;
	ret = t4_set_rxmode(adapter, adapter->mbox, pi->viid, new_mtu, -1, -1,
			    -1, -1, true);
	if (!ret)
		dev->mtu = new_mtu;
	return ret;
}

static int cxgb_set_mac_addr(struct net_device *dev, void *p)
{
	int ret;
	struct sockaddr *addr = p;
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EINVAL;

	ret = t4_change_mac(adapter, adapter->mbox, pi->viid,
			    pi->xact_addr_filt, addr->sa_data, true, true);
	if (ret < 0)
		return ret;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
	pi->xact_addr_filt = ret;
	return 0;
}

static void vlan_rx_register(struct net_device *dev, struct vlan_group *grp)
{
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adapter = pi->adapter;

	pi->vlan_grp = grp;
	t4_set_rxmode(adapter, adapter->mbox, pi->viid, -1, -1, -1, -1,
		      grp != NULL, 0);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void cxgb_netpoll(struct net_device *dev)
{
	struct port_info *pi = netdev_priv(dev);
	struct adapter *adap = pi->adapter;

	if (adap->flags & USING_MSIX) {
		int i;
		struct sge_eth_rxq *rx = &adap->sge.ethrxq[pi->first_qset];

		for (i = pi->nqsets; i; i--, rx++)
			t4_sge_intr_msix(0, &rx->rspq);
	} else
		t4_intr_handler(adap)(0, adap);
}
#endif

void t4_fatal_err(struct adapter *adap)
{
	t4_set_reg_field(adap, A_SGE_CONTROL, F_GLOBALENABLE, 0);
	t4_intr_disable(adap);
	dev_alert(adap->pdev_dev, "encountered fatal error, adapter stopped\n");
}

static void setup_memwin(struct adapter *adap)
{
	u32 bar0;

	bar0 = pci_resource_start(adap->pdev, 0);  /* truncation intentional */
	t4_write_reg(adap, PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_BASE_WIN, 0),
		     (bar0 + MEMWIN0_BASE) | V_BIR(0) |
		     V_WINDOW(ilog2(MEMWIN0_APERTURE) - 10));
	t4_write_reg(adap, PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_BASE_WIN, 1),
		     (bar0 + MEMWIN1_BASE) | V_BIR(0) |
		     V_WINDOW(ilog2(MEMWIN1_APERTURE) - 10));
	t4_write_reg(adap, PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_BASE_WIN, 2),
		     (bar0 + MEMWIN2_BASE) | V_BIR(0) |
		     V_WINDOW(ilog2(MEMWIN2_APERTURE) - 10));
	if (adap->vres.ocq.size) {
		unsigned int start, sz_kb;

		start = pci_resource_start(adap->pdev, 2) +
			OCQ_WIN_OFFSET(adap->pdev, &adap->vres);
		sz_kb = roundup_pow_of_two(adap->vres.ocq.size) >> 10;
		t4_write_reg(adap,
			     PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_BASE_WIN, 3),
			     start | V_BIR(1) | V_WINDOW(ilog2(sz_kb)));
		t4_write_reg(adap,
			     PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_OFFSET, 3),
			     adap->vres.ocq.start);
		t4_read_reg(adap,
			    PCIE_MEM_ACCESS_REG(A_PCIE_MEM_ACCESS_OFFSET, 3));
	}
}

/*
 * Max # of ATIDs.  The absolute HW max is 16K but we keep it lower.
 */
#define MAX_ATIDS 8192U

/*
 * Phase 0 of initialization: contact FW, obtain config, perform basic init.
 */
static int adap_init0(struct adapter *adap)
{
	struct sge *s = &adap->sge;
	int ret;
	u32 v, port_vec;
	enum dev_state state;
	u32 params[7], val[7];
	struct fw_caps_config_cmd caps_cmd;
	struct fw_devlog_cmd devlog_cmd;
	u32 devlog_meminfo;
	int j;

	ret = t4_check_fw_version(adap);
	if (ret == -EINVAL || ret > 0) {
		if (upgrade_fw(adap) >= 0)             /* recache FW version */
			ret = t4_check_fw_version(adap);
	}
	if (ret < 0)
		return ret;

	if (!fw_attach)
		return 0;

	/* contact FW, request master */
	ret = t4_fw_hello(adap, adap->mbox, adap->mbox, MASTER_MUST, &state);
	if (ret < 0) {
		CH_ERR(adap, "could not connect to FW, error %d\n", ret);
		return ret;
	}

	/* reset device */
	ret = t4_fw_reset(adap, adap->mbox, F_PIORSTMODE | F_PIORST);
	if (ret < 0)
		goto bye;

	/*
	 * Grab VPD parameters.  This should be done after we establish a
	 * connection to the firmware since some of the VPD parameters
	 * (notably the Core Clock frequency) are retrieved via requests to
	 * the firmware.  Note that we need the VPD parameters before we call
	 * sge_init() since it uses the Core Clock value.
	 */
	ret = t4_get_vpd_params(adap, &adap->params.vpd);
	if (ret < 0)
		goto bye;

	/* query parameters */
	v =
	    V_FW_PARAMS_MNEM(FW_PARAMS_MNEM_DEV) |
	    V_FW_PARAMS_PARAM_X(FW_PARAMS_PARAM_DEV_PORTVEC);
	ret = t4_query_params(adap, adap->mbox, adap->pf, 0, 1, &v, &port_vec);
	if (ret < 0)
		goto bye;

	/*
	 * Read firmware device log parameters.  We really need to find a way
	 * to get these parameters initialized with some default values (which
	 * are likely to be correct) for the case where we either don't
	 * attache to the firmware or it's crashed when we probe the adapter.
	 * That way we'll still be able to perform early firmware startup
	 * debugging ...  If the request to get the Firmware's Device Log
	 * parameters fails, we'll live so we don't make that a fatal error.
	 */
	memset(&devlog_cmd, 0, sizeof devlog_cmd);
	devlog_cmd.op_to_write = htonl(V_FW_CMD_OP(FW_DEVLOG_CMD) |
				       F_FW_CMD_REQUEST | F_FW_CMD_READ);
	devlog_cmd.retval_len16 = htonl(FW_LEN16(devlog_cmd));
	ret = t4_wr_mbox(adap, adap->mbox, &devlog_cmd, sizeof(devlog_cmd),
			 &devlog_cmd);
	if (ret == 0) {
		devlog_meminfo = ntohl(devlog_cmd.memtype_devlog_memaddr16_devlog);
		adap->params.devlog.memtype = G_FW_DEVLOG_CMD_MEMTYPE_DEVLOG(devlog_meminfo);
		adap->params.devlog.start = G_FW_DEVLOG_CMD_MEMADDR16_DEVLOG(devlog_meminfo) << 4;
		adap->params.devlog.size = ntohl(devlog_cmd.memsize_devlog);
	}

	/* get device capabilities */
	memset(&caps_cmd, 0, sizeof(caps_cmd));
	caps_cmd.op_to_write = htonl(V_FW_CMD_OP(FW_CAPS_CONFIG_CMD) |
				     F_FW_CMD_REQUEST | F_FW_CMD_READ);
	caps_cmd.retval_len16 = htonl(V_FW_CMD_LEN16(sizeof(caps_cmd) / 16));
	ret = t4_wr_mbox(adap, adap->mbox, &caps_cmd, sizeof(caps_cmd),
			 &caps_cmd);
	if (ret < 0)
		goto bye;

	/* select capabilities we'll be using */
	if (caps_cmd.niccaps & htons(FW_CAPS_CONFIG_NIC_VM)) {
		if (!vf_acls)
			caps_cmd.niccaps ^= htons(FW_CAPS_CONFIG_NIC_VM);
		else
			caps_cmd.niccaps = htons(FW_CAPS_CONFIG_NIC_VM);
	} else if (vf_acls) {
		CH_ERR(adap, "virtualization ACLs not supported");
		goto bye;
	}
	caps_cmd.op_to_write = htonl(V_FW_CMD_OP(FW_CAPS_CONFIG_CMD) |
			      F_FW_CMD_REQUEST | F_FW_CMD_WRITE);
	ret = t4_wr_mbox(adap, adap->mbox, &caps_cmd, sizeof(caps_cmd), NULL);
	if (ret < 0)
		goto bye;

	/*
	 * Select RSS Global Mode we want to use.  We use "Basic Virtual"
	 * mode which maps each Virtual Interface to its own section of
	 * the RSS Table and we turn on all map and hash enables ...
	 */
	adap->flags |= RSS_TNLALLLOOKUP;
	ret = t4_config_glbl_rss(adap, adap->mbox,
				 FW_RSS_GLB_CONFIG_CMD_MODE_BASICVIRTUAL,
				 F_FW_RSS_GLB_CONFIG_CMD_TNLMAPEN |
				 F_FW_RSS_GLB_CONFIG_CMD_HASHTOEPLITZ |
				 ((adap->flags & RSS_TNLALLLOOKUP) ?
					F_FW_RSS_GLB_CONFIG_CMD_TNLALLLKP : 0));
	if (ret < 0)
		goto bye;

	ret = t4_cfg_pfvf(adap, adap->mbox, adap->pf, 0,
			  PFRES_NEQ, PFRES_NETHCTRL,
			  PFRES_NIQFLINT, PFRES_NIQ,
			  PFRES_TC, PFRES_NVI,
			  M_FW_PFVF_CMD_CMASK,
			  pfvfres_pmask(adap, adap->pf, 0),
			  PFRES_NEXACTF,
			  PFRES_R_CAPS, PFRES_WX_CAPS);
	if (ret < 0)
		goto bye;

	/*
	 * Perform low level SGE initialization _before_ enabling any SR-IOV
	 * because the Virtual Function Driver depends on this initialization.
	 * Also FW reads some of the settings.
	 */
	for (v = 0; v < SGE_NTIMERS - 1; v++)
		s->timer_val[v] = min(intr_holdoff[v], MAX_SGE_TIMERVAL);
	s->timer_val[SGE_NTIMERS - 1] = MAX_SGE_TIMERVAL;
	s->counter_val[0] = 1;
	for (v = 1; v < SGE_NCOUNTERS; v++)
		s->counter_val[v] = min(intr_cnt[v - 1], M_THRESHOLD_0);
	t4_sge_init(adap);

	adap->params.nports = hweight32(port_vec);
	adap->params.portvec = port_vec;

#ifdef CONFIG_PCI_IOV
	/*
	 * Provision resource limits for Virtual Functions.  We currently
	 * grant them all the same static resource limits except for the Port
	 * Access Rights Mask which we're assigning based on the PF.  All of
	 * the static provisioning stuff for both the PF and VF really needs
	 * to be managed in a persistent manner for each device which the
	 * firmware controls.
	 */
	{
		int pf, vf;

		for (pf = 0; pf < ARRAY_SIZE(num_vf); pf++) {
			if (num_vf[pf] <= 0)
				continue;

			/* VF numbering starts at 1! */
			for (vf = 1; vf <= num_vf[pf]; vf++) {
				ret = t4_cfg_pfvf(adap, adap->mbox,
						  pf, vf,
						  VFRES_NEQ, VFRES_NETHCTRL,
						  VFRES_NIQFLINT, VFRES_NIQ,
						  VFRES_TC, VFRES_NVI,
						  M_FW_PFVF_CMD_CMASK,
						  pfvfres_pmask(adap, pf, vf),
						  VFRES_NEXACTF,
						  VFRES_R_CAPS, VFRES_WX_CAPS);
				if (ret < 0)
					dev_warn(adap->pdev_dev, "failed to "
						 "provision pf/vf=%d/%d; "
						 "err=%d\n", pf, vf, ret);
			}
		}
	}
#endif

	/*
	 * Set up the default filter mode.  Later we'll want to implement this
	 * via a firmware command, etc. ...  This needs to be done before the
	 * firmare initialization command in t4_early_init() ...  If the
	 * selected set of fields isn't equal to the default value, we'll need
	 * to make sure that the field selections will fit in the 36-bit
	 * budget.
	 */
	if (tp_vlan_pri_map != TP_VLAN_PRI_MAP_DEFAULT) {
		int i, bits = 0;

		for (i = TP_VLAN_PRI_MAP_FIRST; i <= TP_VLAN_PRI_MAP_LAST; i++)
			switch (tp_vlan_pri_map & (1 << i)) {
			    case 0:
				/* compressed filter field not enabled */
				break;

			    case F_FCOE:          bits +=  1; break;
			    case F_PORT:          bits +=  3; break;
			    case F_VNIC_ID:       bits += 17; break;
			    case F_VLAN:          bits += 17; break;
			    case F_TOS:           bits +=  8; break;
			    case F_PROTOCOL:      bits +=  8; break;
			    case F_ETHERTYPE:     bits += 16; break;
			    case F_MACMATCH:      bits +=  9; break;
			    case F_MPSHITTYPE:    bits +=  3; break;
			    case F_FRAGMENTATION: bits +=  1; break;
			}

		if (bits > 36) {
			CH_ERR(adap, "tp_vlan_pri_map=%#x needs %d bits > 36;"
			       " using %#x\n", tp_vlan_pri_map, bits,
			       TP_VLAN_PRI_MAP_DEFAULT);
			tp_vlan_pri_map = TP_VLAN_PRI_MAP_DEFAULT;
		}
	}
	v = tp_vlan_pri_map;
	t4_write_indirect(adap, A_TP_PIO_ADDR, A_TP_PIO_DATA,
			  &v, 1, A_TP_VLAN_PRI_MAP);

	/*
	 * We need FiveTupleLookup mode to be set in order to support
	 * any of the compressed filter fields above.  meanwhile, we need
	 * LookupEveryPacket set in order to support matching non-TCP
	 * packets.
	 */
	if (tp_vlan_pri_map)
		t4_set_reg_field(adap, A_TP_GLOBAL_CONFIG,
				 V_FIVETUPLELOOKUP(M_FIVETUPLELOOKUP),
				 V_FIVETUPLELOOKUP(M_FIVETUPLELOOKUP));
	t4_tp_wr_bits_indirect(adap, A_TP_INGRESS_CONFIG, F_CSUM_HAS_PSEUDO_HDR,
			       F_LOOKUPEVERYPKT);

	/*
	 * Get basic stuff going.  Note that this _must_ be after all PFVF
	 * commands ...
	 */
	ret = t4_early_init(adap, adap->mbox);
	if (ret < 0)
		goto bye;

#define FW_PARAM_DEV(param) \
	V_FW_PARAMS_MNEM(FW_PARAMS_MNEM_DEV) | \
	V_FW_PARAMS_PARAM_X(FW_PARAMS_PARAM_DEV_##param)

#define FW_PARAM_PFVF(param) \
	V_FW_PARAMS_MNEM(FW_PARAMS_MNEM_PFVF) | \
	V_FW_PARAMS_PARAM_X(FW_PARAMS_PARAM_PFVF_##param)|  \
	V_FW_PARAMS_PARAM_Y(0) | \
	V_FW_PARAMS_PARAM_Z(0)

	params[0] = FW_PARAM_DEV(PORTVEC);
	params[1] = FW_PARAM_PFVF(L2T_START);
	params[2] = FW_PARAM_PFVF(L2T_END);
	params[3] = FW_PARAM_PFVF(FILTER_START);
	params[4] = FW_PARAM_PFVF(FILTER_END);
	params[5] = FW_PARAM_PFVF(IQFLINT_START);
	params[6] = FW_PARAM_PFVF(EQ_START);
	ret = t4_query_params(adap, adap->mbox, adap->pf, 0, 7, params, val);
	if (ret < 0)
		goto bye;
	port_vec = val[0];
	adap->tids.ftid_base = val[3];
	adap->tids.nftids = val[4] - val[3] + 1;
	adap->sge.ingr_start = val[5];
	adap->sge.egr_start = val[6];

	if (caps_cmd.toecaps) {
		/* query offload-related parameters */
		params[0] = FW_PARAM_DEV(NTID);
		params[1] = FW_PARAM_PFVF(SERVER_START);
		params[2] = FW_PARAM_PFVF(SERVER_END);
		params[3] = FW_PARAM_PFVF(TDDP_START);
		params[4] = FW_PARAM_PFVF(TDDP_END);
		params[5] = FW_PARAM_DEV(FLOWC_BUFFIFO_SZ);
		ret = t4_query_params(adap, adap->mbox, adap->pf, 0, 6,
				      params, val);
		if (ret < 0)
			goto bye;
		adap->tids.ntids = val[0];
		adap->tids.natids = min(adap->tids.ntids / 2, MAX_ATIDS);
		adap->tids.stid_base = val[1];
		adap->tids.nstids = val[2] - val[1] + 1;
		adap->vres.ddp.start = val[3];
		adap->vres.ddp.size = val[4] - val[3] + 1;
		adap->params.ofldq_wr_cred = val[5];
		adap->params.offload = 1;
	}
	if (caps_cmd.rdmacaps) {
		params[0] = FW_PARAM_PFVF(STAG_START);
		params[1] = FW_PARAM_PFVF(STAG_END);
		params[2] = FW_PARAM_PFVF(RQ_START);
		params[3] = FW_PARAM_PFVF(RQ_END);
		params[4] = FW_PARAM_PFVF(PBL_START);
		params[5] = FW_PARAM_PFVF(PBL_END);
		ret = t4_query_params(adap, adap->mbox, adap->pf, 0, 6,
				      params, val);
		if (ret < 0)
			goto bye;
		adap->vres.stag.start = val[0];
		adap->vres.stag.size = val[1] - val[0] + 1;
		adap->vres.rq.start = val[2];
		adap->vres.rq.size = val[3] - val[2] + 1;
		adap->vres.pbl.start = val[4];
		adap->vres.pbl.size = val[5] - val[4] + 1;

		params[0] = FW_PARAM_PFVF(SQRQ_START);
		params[1] = FW_PARAM_PFVF(SQRQ_END);
		params[2] = FW_PARAM_PFVF(CQ_START);
		params[3] = FW_PARAM_PFVF(CQ_END);
		params[4] = FW_PARAM_PFVF(OCQ_START);
		params[5] = FW_PARAM_PFVF(OCQ_END);
		ret = t4_query_params(adap, 0, 0, 0, 6, params, val);
		if (ret < 0)
			goto bye;
		adap->vres.qp.start = val[0];
		adap->vres.qp.size = val[1] - val[0] + 1;
		adap->vres.cq.start = val[2];
		adap->vres.cq.size = val[3] - val[2] + 1;
		adap->vres.ocq.start = val[4];
		adap->vres.ocq.size = val[5] - val[4] + 1;
	}
	if (caps_cmd.iscsicaps) {
		params[0] = FW_PARAM_PFVF(ISCSI_START);
		params[1] = FW_PARAM_PFVF(ISCSI_END);
		ret = t4_query_params(adap, adap->mbox, adap->pf, 0, 2,
				      params, val);
		if (ret < 0)
			goto bye;
		adap->vres.iscsi.start = val[0];
		adap->vres.iscsi.size = val[1] - val[0] + 1;
	}
#undef FW_PARAM_PFVF
#undef FW_PARAM_DEV

	/* These are finalized by FW initialization, load their values now */
	v = t4_read_reg(adap, A_TP_TIMER_RESOLUTION);
	adap->params.tp.tre = G_TIMERRESOLUTION(v);
	adap->params.tp.dack_re = G_DELAYEDACKRESOLUTION(v);
	t4_read_mtu_tbl(adap, adap->params.mtus, NULL);
	t4_load_mtus(adap, adap->params.mtus, adap->params.a_wnd,
		     adap->params.b_wnd);

	/* tweak some settings */
	t4_write_reg(adap, A_TP_SHIFT_CNT, V_SYNSHIFTMAX(6) |
		     V_RXTSHIFTMAXR1(4) | V_RXTSHIFTMAXR2(15) |
		     V_PERSHIFTBACKOFFMAX(8) | V_PERSHIFTMAX(8) |
		     V_KEEPALIVEMAXR1(4) | V_KEEPALIVEMAXR2(9));
	t4_write_reg(adap, A_ULP_RX_TDDP_PSZ, V_HPZ0(PAGE_SHIFT - 12));

	/* MODQ_REQ_MAP defaults to setting queues 0-3 to chan 0-3 */
	for (j=0; j< NCHAN; j++)
		adap->params.tp.tx_modq[j] = j;
	
	adap->flags |= FW_OK;
	return 0;

	/*
	 * If a command timed out or failed with EIO FW does not operate within
	 * its spec or something catastrophic happened to HW/FW, stop issuing
	 * commands.
	 */
bye:	if (ret != -ETIMEDOUT && ret != -EIO)
		t4_fw_bye(adap, adap->mbox);
	return ret;
}

static inline bool is_10g_port(const struct link_config *lc)
{
	return (lc->supported & FW_PORT_CAP_SPEED_10G) != 0;
}

static inline void init_rspq(struct adapter *adap, struct sge_rspq *q,
			     unsigned int us, unsigned int cnt,
			     unsigned int size, unsigned int iqe_size)
{
	q->adapter = adap;
	set_rspq_intr_params(q, us, cnt);
	q->iqe_len = iqe_size;
	q->size = size;
}

/*
 * Perform default configuration of DMA queues depending on the number and type
 * of ports we found and the number of available CPUs.  Most settings can be
 * modified by the admin prior to actual use.
 */
static void __devinit cfg_queues(struct adapter *adap)
{
	struct sge *s = &adap->sge;
	int i, q10g = 0, n10g = 0, qidx = 0;

	for_each_port(adap, i)
		n10g += is_10g_port(&adap2pinfo(adap, i)->link_cfg);

	/*
	 * We default to 1 queue per non-10G port and up to # of cores queues
	 * per 10G port.
	 */
	if (n10g)
		q10g = (MAX_ETH_QSETS - (adap->params.nports - n10g)) / n10g;
	if (q10g > num_online_cpus())
		q10g = num_online_cpus();

	for_each_port(adap, i) {
		struct port_info *pi = adap2pinfo(adap, i);

		pi->first_qset = qidx;
		pi->nqsets = is_10g_port(&pi->link_cfg) ? q10g : 1;
		if (pi->nqsets > pi->rss_size)
			pi->nqsets = pi->rss_size;
		qidx += pi->nqsets;
	}

	s->ethqsets = qidx;
	s->max_ethqsets = qidx;   /* MSI-X may lower it later */

	if (is_offload(adap)) {
		/*
		 * For offload we use 1 queue/channel if all ports are up to 1G,
		 * otherwise we divide all available queues amongst the channels
		 * capped by the number of available cores.
		 */
		if (n10g) {
			i = min_t(int, ARRAY_SIZE(s->ofldrxq),
				  num_online_cpus());
			s->ofldqsets = roundup(i, adap->params.nports);
		} else
			s->ofldqsets = adap->params.nports;
		/* For RDMA one Rx queue per channel suffices */
		s->rdmaqs = adap->params.nports;
#ifdef SCSI_CXGB4_ISCSI
		s->niscsiq = adap->params.nports;
#endif
	}

	for (i = 0; i < ARRAY_SIZE(s->ethrxq); i++) {
		struct sge_eth_rxq *r = &s->ethrxq[i];

		init_rspq(adap, &r->rspq, 5, 10, 1024, 64);
		r->useskbs = rx_useskbs;
		r->fl.size = (r->useskbs ? 1024 : 72);
	}

	for (i = 0; i < ARRAY_SIZE(s->ethtxq); i++)
		s->ethtxq[i].q.size = 1024;

	for (i = 0; i < ARRAY_SIZE(s->ctrlq); i++)
		s->ctrlq[i].q.size = 512;

	for (i = 0; i < ARRAY_SIZE(s->ofldtxq); i++)
		s->ofldtxq[i].q.size = 1024;

	for (i = 0; i < ARRAY_SIZE(s->ofldrxq); i++) {
		struct sge_ofld_rxq *r = &s->ofldrxq[i];

		init_rspq(adap, &r->rspq, 5, 1, 1024, 64);
		r->rspq.uld = CXGB4_ULD_TOE;
		r->useskbs = 0;
		r->fl.size = 72;
	}

	for (i = 0; i < ARRAY_SIZE(s->rdmarxq); i++) {
		struct sge_ofld_rxq *r = &s->rdmarxq[i];

		init_rspq(adap, &r->rspq, 5, 1, 511, 64);
		r->rspq.uld = CXGB4_ULD_RDMA;
		r->useskbs = 0;
		r->fl.size = 72;
	}

	for (i = 0; i < ARRAY_SIZE(s->iscsirxq); i++) {
		struct sge_ofld_rxq *r = &s->iscsirxq[i];

		init_rspq(adap, &r->rspq, 5, 1, 1024, 64);
		r->rspq.uld = CXGB4_ULD_ISCSI;
		r->useskbs = 0;
		r->fl.size = 72;
	}

	init_rspq(adap, &s->fw_evtq, 0, 1, 1024, 64);
	init_rspq(adap, &s->intrq, 0, 1, 2 * MAX_INGQ, 64);
}

/*
 * Interrupt handler used to check if MSI/MSI-X works on this platform.
 */
static irqreturn_t __devinit check_intr_handler(int irq, void *data)
{
	struct adapter *adap = data;

	adap->swintr = 1;
	t4_write_reg(adap, MYPF_REG(A_PL_PF_INT_CAUSE), F_PFSW);
	t4_read_reg(adap, MYPF_REG(A_PL_PF_INT_CAUSE));          /* flush */
	return IRQ_HANDLED;
}

static void __devinit check_msi(struct adapter *adap)
{
	int vec;

	vec = (adap->flags & USING_MSI) ? adap->pdev->irq :
					  adap->msix_info[0].vec;

	if (request_irq(vec, check_intr_handler, 0, adap->name, adap))
		return;

	adap->swintr = 0;
	t4_write_reg(adap, MYPF_REG(A_PL_PF_INT_ENABLE), F_PFSW);
	t4_write_reg(adap, MYPF_REG(A_PL_PF_CTL), F_SWINT);
	msleep(10);
	t4_write_reg(adap, MYPF_REG(A_PL_PF_INT_ENABLE), 0);
	free_irq(vec, adap);

	if (!adap->swintr) {
		const char *s = (adap->flags & USING_MSI) ? "MSI" : "MSI-X";

		cxgb_disable_msi(adap);
		dev_info(adap->pdev_dev,
			 "the kernel believes that %s is available on this "
			 "platform\nbut the driver's %s test has failed.  "
			 "Proceeding with INTx interrupts.\n", s, s);
	}
}

/*
 * Reduce the number of Ethernet queues across all ports to at most n.
 * n provides at least one queue per port.
 */
static void __devinit reduce_ethqs(struct adapter *adap, int n)
{
	int i;
	struct port_info *pi;

	while (n < adap->sge.ethqsets)
		for_each_port(adap, i) {
			pi = adap2pinfo(adap, i);
			if (pi->nqsets > 1) {
				pi->nqsets--;
				adap->sge.ethqsets--;
				if (adap->sge.ethqsets <= n)
					break;
			}
		}

	n = 0;
	for_each_port(adap, i) {
		pi = adap2pinfo(adap, i);
		pi->first_qset = n;
		n += pi->nqsets;
	}
}

/* 2 MSI-X vectors needed for the FW queue and non-data interrupts */
#define EXTRA_VECS 2

static int __devinit cxgb_enable_msix(struct adapter *adap)
{
	int ofld_need = 0;
	int i, err, want, need;
	struct sge *s = &adap->sge;
	unsigned int nchan = adap->params.nports;
	struct msix_entry entries[MAX_INGQ + 1];

	for (i = 0; i < ARRAY_SIZE(entries); ++i)
		entries[i].entry = i;

	want = s->max_ethqsets + EXTRA_VECS;
	if (is_offload(adap)) {
		want += s->rdmaqs + s->ofldqsets + s->niscsiq;
		/* need nchan for each possible ULD */
		ofld_need = 2 * nchan;
#ifdef SCSI_CXGB4_ISCSI
		ofld_need += nchan;
#endif
	}
	need = adap->params.nports + EXTRA_VECS + ofld_need;

	while ((err = pci_enable_msix(adap->pdev, entries, want)) >= need)
		want = err;

	if (!err) {
		/*
		 * Distribute available vectors to the various queue groups.
		 * Every group gets its minimum requirement and NIC gets top
		 * priority for leftovers.
		 */
		i = want - EXTRA_VECS - ofld_need;
		if (i < s->max_ethqsets) {
			s->max_ethqsets = i;
			if (i < s->ethqsets)
				reduce_ethqs(adap, i);
		}
		if (is_offload(adap)) {
			/* after NIC leftovers go to TOE */
			i = want - EXTRA_VECS - s->max_ethqsets;
			i -= ofld_need - nchan;
			s->ofldqsets = (i / nchan) * nchan;  /* round down */
		}
		for (i = 0; i < want; ++i)
			adap->msix_info[i].vec = entries[i].vector;
	} else if (err > 0) {
		pci_disable_msix(adap->pdev);
		dev_info(adap->pdev_dev, 
			 "only %d MSI-X vectors left, not using MSI-X\n", err);
	}
	return err;
}

#undef EXTRA_VECS

static void __devinit print_port_info(adapter_t *adap)
{
	static const char *base[] = {
		"Fiber_XFI",
		"Fiber_XAUI",
		"BT_SGMII",
		"BT_XFI",
		"BT_XAUI",
		"KX4",
		"CX4",
		"KX",
		"KR",
		"SFP",
		"BP_AP",
		"BP4_AP",
	};

	int i;
	char buf[80];

	for_each_port(adap, i) {
		struct net_device *dev = adap->port[i];
		const struct port_info *pi = netdev_priv(dev);
		char *bufp = buf;

		if (!test_bit(i, &adap->registered_device_map))
			continue;

		if (pi->link_cfg.supported & FW_PORT_CAP_SPEED_100M)
			bufp += sprintf(bufp, "100/");
		if (pi->link_cfg.supported & FW_PORT_CAP_SPEED_1G)
			bufp += sprintf(bufp, "1000/");
		if (pi->link_cfg.supported & FW_PORT_CAP_SPEED_10G)
			bufp += sprintf(bufp, "10G/");
		if (bufp != buf)
			--bufp;
		if (pi->port_type < ARRAY_SIZE(base))
			sprintf(bufp, "BASE-%s", base[pi->port_type]);
		else
			sprintf(bufp, "BASE-UNKNOWN[%d]", pi->port_type);

		printk(KERN_INFO "%s: Chelsio %s rev %d %s %sNIC PCIe x%d%s\n",
		       dev->name, adap->params.vpd.id, adap->params.rev,
		       buf, is_offload(adap) ? "R" : "", adap->params.pci.width,
		       (adap->flags & USING_MSIX) ? " MSI-X" :
		       (adap->flags & USING_MSI) ? " MSI" : "");

		printk(KERN_INFO "%s: S/N: %s, E/C: %s\n", adap->name,
		       adap->params.vpd.sn, adap->params.vpd.ec);
	}
}

#ifdef CONFIG_PCI_IOV
/**
 *	vf_monitor - monitor VFs for potential problems
 *	@work: the adapter's vf_monitor_task
 *
 *	VFs can get into trouble in various ways so we monitor them to see if
 *	they need to be kicked, reset, etc.
 */
static void vf_monitor(struct work_struct *work)
{
	struct adapter *adapter = container_of(work, struct adapter,
					       vf_monitor_task.work);
	struct pci_dev *pdev;
	u32 pcie_cdebug;
	unsigned int reqfn;
	const unsigned int vf_offset = 8;
	const unsigned int vf_stride = 4;
	unsigned int vfdevfn, pf, vf;
	struct pci_dev *vfdev;
	int pos, i;
	u16 control;

	/*
	 * Read the PCI-E Debug Register to see if it's hanging with a
	 * Request Valid condition.  But we check it several times to be
	 * Absolutely Sure since we can see the PCI-E block being busy
	 * transiently during normal operation.
	 */
	for (i = 0; i < 4; i++) {
		t4_write_reg(adapter, A_PCIE_CDEBUG_INDEX, 0x3c003c);
		pcie_cdebug = t4_read_reg(adapter, A_PCIE_CDEBUG_DATA_HIGH);
		if ((pcie_cdebug & 0x100) == 0)
			goto reschedule_vf_monitor;
	}

	/*
	 * We're not prepared to deal with anything other than a VF.
	 */
	pdev = adapter->pdev;
	reqfn = (pcie_cdebug >> 24) & 0xff;
	if (reqfn < vf_offset) {
		dev_info(&pdev->dev, "vf_monitor: hung ReqFn %d is a PF!\n",
			 reqfn);
		goto reschedule_vf_monitor;
	}

	/*
	 * Grab a handle on the VF's PCI State.
	 */
	pf = (reqfn - vf_offset) & (vf_stride - 1);
	vf = ((reqfn - vf_offset) & ~(vf_stride - 1))/vf_stride + 1;
	vfdevfn = PCI_SLOT(pdev->devfn) + reqfn;
	vfdev = pci_get_slot(pdev->bus, vfdevfn);
	if (vfdev == NULL) {
		dev_info(&pdev->dev, "vf_monitor: can't find PF%d/VF%d",
			 pf, vf);
		goto reschedule_vf_monitor;
	}

	/*
	 * Now that we have a handle on the VF which is hung, we need to
	 * mask and re-enable its interrupts, reset it and then disable its
	 * interrupts again.
	 */
	pos = pci_find_capability(vfdev, PCI_CAP_ID_MSIX);
	if (!pos) {
		dev_err(&pdev->dev, "vf_monitor: can't find MSI-X PF%d/VF%d\n",
			pf, vf);
		goto drop_vfdev_reference;
	}
	pci_read_config_word(vfdev, pos+PCI_MSIX_FLAGS, &control);
	if (control & PCI_MSIX_FLAGS_ENABLE) {
		dev_info(&pdev->dev, "vf_monitor: MSI-X already enabled PF%d/VF%d\n",
			 pf, vf);
		goto drop_vfdev_reference;
	}
	pci_write_config_word(vfdev, pos+PCI_MSIX_FLAGS,
			      control |
			      PCI_MSIX_FLAGS_ENABLE |
			      PCI_MSIX_FLAGS_MASKALL);
	pci_reset_function(vfdev);
	pci_write_config_word(vfdev, pos+PCI_MSIX_FLAGS, control);
	dev_warn(&pdev->dev, "vf_monitor: reset hung PF%d/VF%d\n", pf, vf);

drop_vfdev_reference:
	/*
	 * Drop reference to the VF's CI State.
	 */
	pci_dev_put(vfdev);

reschedule_vf_monitor:
	/*
	 * Set up for the next time we need to check things ...
	 */
	schedule_delayed_work(&adapter->vf_monitor_task, VF_MONITOR_PERIOD);
}
#endif

#ifdef HAVE_NET_DEVICE_OPS
static const struct net_device_ops cxgb4_netdev_ops = {
	.ndo_open             = cxgb_open,
	.ndo_stop             = cxgb_close,
	.ndo_start_xmit       = t4_eth_xmit,
	.ndo_get_stats        = cxgb_get_stats,
	.ndo_set_rx_mode      = cxgb_set_rxmode,
	.ndo_set_mac_address  = cxgb_set_mac_addr,
	.ndo_validate_addr    = eth_validate_addr,
	.ndo_do_ioctl         = cxgb_ioctl,
	.ndo_change_mtu       = cxgb_change_mtu,
	.ndo_vlan_rx_register = vlan_rx_register,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller  = cxgb_netpoll,
#endif
};
#endif

#define VLAN_FEAT (NETIF_F_SG | NETIF_F_IP_CSUM | TSO_FLAGS | \
		   NETIF_F_IPV6_CSUM | NETIF_F_HIGHDMA)

static int __devinit init_one(struct pci_dev *pdev,
			      const struct pci_device_id *ent)
{
	static int version_printed;

	int func;
	int i, err, pci_using_dac = 0;
	struct adapter *adapter = NULL;
	struct port_info *pi;

	if (!version_printed) {
		printk(KERN_INFO "%s - version %s\n", DRV_DESC, DRV_VERSION);
		++version_printed;
	}

	err = pci_request_regions(pdev, KBUILD_MODNAME);
	if (err) {
		/* Just info, some other driver may have claimed the device. */
		dev_info(&pdev->dev, "cannot obtain PCI resources\n");
		return err;
	}

	/*
	 * We control everything via a single PF (which we refer to as the
	 * "Master PF").  This Master PF is identifed with a special PCI
	 * Device ID separate from the "normal" NIC Device IDs so for the most
	 * part we just advertise that we want to be hooked up with the
	 * Unified PF and everything works out.  However, for the "PE10K" FPGA
	 * device, all [two] of the PFs have the same Device ID so we need to
	 * explcitly skip working with any PF other than Master PF.
	 */
	func = PCI_FUNC(pdev->devfn);
	if (pdev->device == 0xa000 && func != master_pf)
		goto out_success;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "cannot enable PCI device\n");
		goto out_release_regions;
	}

	pci_enable_pcie_error_reporting(pdev);

	if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
		pci_using_dac = 1;
		err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
		if (err) {
			dev_err(&pdev->dev, "unable to obtain 64-bit DMA for "
				"coherent allocations\n");
			goto out_disable_device;
		}
	} else if ((err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) != 0) {
		dev_err(&pdev->dev, "no usable DMA configuration\n");
		goto out_disable_device;
	}

	pci_set_master(pdev);

	adapter = kzalloc(sizeof(*adapter), GFP_KERNEL);
	if (!adapter) {
		err = -ENOMEM;
		goto out_disable_device;
	}
	adapter->regs = pci_ioremap_bar(pdev, 0);
	if (!adapter->regs) {
		dev_err(&pdev->dev, "cannot map device registers\n");
		err = -ENOMEM;
		goto out_free_adapter;
	}

	adapter->pdev = pdev;
	adapter->pdev_dev = &pdev->dev;
	adapter->name = pci_name(pdev);
	adapter->mbox = func;
	adapter->pf = func;
	adapter->msg_enable = dflt_msg_enable;
	memset(adapter->chan_map, 0xff, sizeof(adapter->chan_map));
	bitmap_zero(adapter->sge.blocked_fl, MAX_EGRQ);

	spin_lock_init(&adapter->mdio_lock);
	spin_lock_init(&adapter->win0_lock);
	spin_lock_init(&adapter->work_lock);
	spin_lock_init(&adapter->stats_lock);
	spin_lock_init(&adapter->tid_release_lock);
	t4_os_lock_init(&adapter->mbox_lock);
	mutex_init(&adapter->user_mutex);
	INIT_LIST_HEAD(&adapter->mbox_list.list);

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	INIT_WORK(&adapter->tid_release_task, process_tid_release_list);
#endif

	err = t4_prep_adapter(adapter, false);
	if (err)
		goto out_unmap_bar;
	err = adap_init0(adapter);
	if (err)
		dev_err(&pdev->dev, "Failed to contact FW, error %d.  "
			"Continuing in debug mode\n", err);
	setup_memwin(adapter);

	adapter->tx_coal = tx_coal;

	for_each_port(adapter, i) {
		struct net_device *netdev;

		netdev = alloc_etherdev_mq(sizeof(struct port_info),
					   MAX_ETH_QSETS);
		if (!netdev) {
			err = -ENOMEM;
			goto out_free_dev;
		}

		SET_NETDEV_DEV(netdev, &pdev->dev);

		adapter->port[i] = netdev;
		pi = netdev_priv(netdev);
		pi->adapter = adapter;
		pi->xact_addr_filt = -1;
		pi->rx_offload = RX_CSO;
		pi->port_id = i;
		netif_carrier_off(netdev);
		netdev->irq = pdev->irq;

		netdev->features |= NETIF_F_SG | TSO_FLAGS;
		netdev->features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
		if (pci_using_dac)
			netdev->features |= NETIF_F_HIGHDMA;

#ifdef CONFIG_CXGB4_GRO
		netdev->features |= NETIF_F_GRO;
#endif
		netdev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
		netdev->vlan_features = netdev->features & VLAN_FEAT;
#ifdef HAVE_NET_DEVICE_OPS
		netdev->netdev_ops = &cxgb4_netdev_ops;
#else
		netdev->vlan_rx_register = vlan_rx_register;

		netdev->open = cxgb_open;
		netdev->stop = cxgb_close;
		netdev->hard_start_xmit = t4_eth_xmit;
		netdev->get_stats = cxgb_get_stats;
		netdev->set_rx_mode = cxgb_set_rxmode;
		netdev->do_ioctl = cxgb_ioctl;
		netdev->change_mtu = cxgb_change_mtu;
		netdev->set_mac_address = cxgb_set_mac_addr;
#ifdef CONFIG_NET_POLL_CONTROLLER
		netdev->poll_controller = cxgb_netpoll;
#endif
#endif
		SET_ETHTOOL_OPS(netdev, &cxgb_ethtool_ops);
	}

	pci_set_drvdata(pdev, adapter);

	if (adapter->flags & FW_OK) {
		err = t4_port_init(adapter, adapter->mbox, adapter->pf, 0);
		if (err)
			goto out_free_dev;
	}

	cfg_queues(adapter);  // XXX move after we know interrupt type

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	adapter->l2t = t4_init_l2t();
	if (!adapter->l2t) {
		/* We tolerate a lack of L2T, giving up some functionality */
		dev_warn(&pdev->dev, "could not allocate L2T, continuing\n");
		adapter->params.offload = 0;
	}

	if (is_offload(adapter) && tid_init(&adapter->tids) < 0) {
		dev_warn(&pdev->dev, "could not allocate TID table, "
			 "continuing\n");
		adapter->params.offload = 0;
	}

	if (is_offload(adapter)) {
		__set_bit(OFFLOAD_DEVMAP_BIT,
			&adapter->registered_device_map);
	}
#endif

	/*
	 * The card is now ready to go.  If any errors occur during device
	 * registration we do not fail the whole card but rather proceed only
	 * with the ports we manage to register successfully.  However we must
	 * register at least one net device.
	 */
	for_each_port(adapter, i) {
		pi = adap2pinfo(adapter, i);
		adapter->port[i]->dev_id = pi->tx_chan;
		adapter->port[i]->real_num_tx_queues = pi->nqsets;

		err = register_netdev(adapter->port[i]);
		if (err)
			dev_warn(&pdev->dev,
				 "cannot register net device %s, skipping\n",
				 adapter->port[i]->name);
		else {
			/*
			 * Change the name we use for messages to the name of
			 * the first successfully registered interface.
			 */
			if (!adapter->registered_device_map)
				adapter->name = adapter->port[i]->name;

			__set_bit(i, &adapter->registered_device_map);
			adapter->chan_map[pi->tx_chan] = i;
		}
	}
	if (!adapter->registered_device_map) {
		dev_err(&pdev->dev, "could not register any net devices\n");
		goto out_free_dev;
	}

	if (cxgb4_proc_root) {
		adapter->proc_root = proc_mkdir(pci_name(pdev),
						cxgb4_proc_root);
		if (!adapter->proc_root)
			dev_warn(&pdev->dev,
				 "could not create /proc directory");
		else
			setup_proc(adapter, adapter->proc_root);
	}

	if (cxgb4_debugfs_root) {
		adapter->debugfs_root = debugfs_create_dir(pci_name(pdev),
							   cxgb4_debugfs_root);
		setup_debugfs(adapter);
	}

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	setup_offload(adapter);
#endif

	/* See what interrupts we'll be using */
	if (msi > 1 && cxgb_enable_msix(adapter) == 0)
		adapter->flags |= USING_MSIX;
	else if (msi > 0 && pci_enable_msi(pdev) == 0)
		adapter->flags |= USING_MSI;
	if (adapter->flags & (USING_MSIX | USING_MSI))
		check_msi(adapter);

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
	if (is_offload(adapter)) {
		attach_ulds(adapter);
	}
#endif

	print_port_info(adapter);

#ifdef CONFIG_PCI_IOV
	/*
	 * Loop accross PF0-3 to see if any VFs need to be instantiated.
	 */
	for (func = 0; func < ARRAY_SIZE(num_vf); func++) {
		struct pci_dev *pf;

		if (num_vf[func] <= 0)
			continue;

		pf = pci_get_slot(pdev->bus,
				  PCI_DEVFN(PCI_SLOT(pdev->devfn),
					    func));
		if (pf == NULL) {
			dev_warn(&pdev->dev, "failed to find PF%d; not"
				 " enabling %d virtual functions\n",
				 func, num_vf[func]);
			continue;
		}
		err = pci_enable_sriov(pf, num_vf[func]);
		if (err < 0)
			dev_warn(&pf->dev, "failed to instantiate %d"
				 " virtual functions; err=%d\n", num_vf[func],
				 err);
		else {
			dev_info(&pf->dev,
				 "instantiated %u virtual functions\n",
				 num_vf[func]);
			adapter->vf_monitor_mask |= 1U << func;
		}
		pci_dev_put(pf);
	}

	/*
	 * If we instantiated any VFs, set up and start recurrant task to
	 * monitor the state of the VFs.
	 */
	if (adapter->vf_monitor_mask) {
		INIT_DELAYED_WORK(&adapter->vf_monitor_task, vf_monitor);
		schedule_delayed_work(&adapter->vf_monitor_task,
				      VF_MONITOR_PERIOD);
	}
#endif

 out_success:
	return 0;

 out_free_dev:
	t4_free_mem(adapter->l2t);
	for_each_port(adapter, i)
		if (adapter->port[i]) {
			pi = netdev_priv(adapter->port[i]);
			if (pi->viid != 0)
				t4_free_vi(adapter, adapter->mbox, adapter->pf,
					   0, pi->viid);
			free_netdev(adapter->port[i]);
		}
	if (adapter->flags & FW_OK)
		t4_fw_bye(adapter, adapter->mbox);
 out_unmap_bar:
	iounmap(adapter->regs);
 out_free_adapter:
	kfree(adapter);
 out_disable_device:
	pci_disable_device(pdev);
 out_release_regions:
	pci_release_regions(pdev);
	pci_set_drvdata(pdev, NULL);
	return err;
}

static void __devexit remove_one(struct pci_dev *pdev)
{
	struct adapter *adapter = pci_get_drvdata(pdev);

#ifdef CONFIG_PCI_IOV
	/*
	 * Tear down VF Monitoring.
	 * Note: Check to see if adapter is defined, because the not all
	 * PFs go through the full init process (i.e. adapter is NULL).
	 */
	if (adapter && adapter->vf_monitor_mask)
		cancel_delayed_work_sync(&adapter->vf_monitor_task);

	/*
	 * Loop accross PF0-3 to see if any VFs need to be uninstantiated.
	 */
	{
		int func;

		for (func = 0; func < 4; func++) {
			struct pci_dev *pf;

			if (num_vf[func] <= 0)
				continue;

			pf = pci_get_slot(pdev->bus,
					  PCI_DEVFN(PCI_SLOT(pdev->devfn),
						    func));
			if (pf == NULL) {
				dev_warn(&pdev->dev, "failed to find PF%d; not"
					 " disabling %d virtual functions\n",
					 func, num_vf[func]);
				continue;
			}
			pci_disable_sriov(pf);
			pci_dev_put(pf);
		}
	}
#endif

	if (adapter) {
		int i;

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
		if (is_offload(adapter))
			detach_ulds(adapter);
#endif

		for_each_port(adapter, i)
			if (test_bit(i, &adapter->registered_device_map))
				unregister_netdev(adapter->port[i]);

		if (adapter->proc_root) {
			cleanup_proc(adapter, adapter->proc_root);
			remove_proc_entry(pci_name(pdev), cxgb4_proc_root);
		}

#ifdef CONFIG_CHELSIO_T4_OFFLOAD
		/*
		 * If we allocated filters, free up state associated with any
		 * valid filters ...
		 */
		if (adapter->tids.ftid_tab) {
			struct filter_entry *f = &adapter->tids.ftid_tab[0];
			for (i = 0; i < adapter->tids.nftids; i++, f++)
				if (f->valid)
					clear_filter(adapter, f);
		}
#endif

		if (adapter->flags & FULL_INIT_DONE)
			cxgb_down(adapter);
		t4_free_mem(adapter->l2t);
		t4_free_mem(adapter->tids.tid_tab);
		t4_free_mem(adapter->filters);
		cxgb_disable_msi(adapter);

		for_each_port(adapter, i)
			if (adapter->port[i]) {
				struct port_info *pi = adap2pinfo(adapter, i);
				if (pi->viid != 0)
					t4_free_vi(adapter, adapter->mbox,
						   adapter->pf, 0, pi->viid);
				free_netdev(adapter->port[i]);
			}

		if (adapter->flags & FW_OK)
			t4_fw_bye(adapter, adapter->mbox);

		if (adapter->debugfs_root) {
			free_trace_bufs(adapter);
			debugfs_remove_recursive(adapter->debugfs_root);
		}

		iounmap(adapter->regs);
		kfree(adapter);
		pci_disable_device(pdev);
		pci_release_regions(pdev);
		pci_set_drvdata(pdev, NULL);
	} else if (PCI_FUNC(pdev->devfn) > 0)
		pci_release_regions(pdev);
}

static struct pci_driver cxgb4_driver = {
	.name     = KBUILD_MODNAME,
	.id_table = cxgb4_pci_tbl,
	.probe    = init_one,
	.remove   = __devexit_p(remove_one),
};

#define DRV_PROC_NAME "driver/" KBUILD_MODNAME

static int __init cxgb4_init_module(void)
{
	int ret;

	/* Debugfs support is optional, just warn if this fails */
	cxgb4_debugfs_root = debugfs_create_dir(KBUILD_MODNAME, NULL);
	if (!cxgb4_debugfs_root)
		pr_warning("could not create debugfs entry, continuing\n");

#ifdef CONFIG_PROC_FS
	cxgb4_proc_root = proc_mkdir(DRV_PROC_NAME, NULL);
	if (!cxgb4_proc_root)
		pr_warning("could not create /proc driver directory, "
			   "continuing\n");
#endif

	ret = pci_register_driver(&cxgb4_driver);
	if (ret < 0) {
		remove_proc_entry(DRV_PROC_NAME, NULL);
		debugfs_remove(cxgb4_debugfs_root);
	}
	return ret;
}

static void __exit cxgb4_cleanup_module(void)
{
	pci_unregister_driver(&cxgb4_driver);
	remove_proc_entry(DRV_PROC_NAME, NULL);
	debugfs_remove(cxgb4_debugfs_root);  /* NULL ok */
}

module_init(cxgb4_init_module);
module_exit(cxgb4_cleanup_module);
