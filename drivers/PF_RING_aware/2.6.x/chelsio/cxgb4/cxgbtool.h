/*
 * This file is part of the Chelsio NIC management interface.
 *
 * Copyright (C) 2003-2011 Chelsio Communications.  All rights reserved.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the LICENSE file included in this
 * release for licensing terms and conditions.
 */
#ifndef __CXGBTOOL_H__
#define __CXGBTOOL_H__

#define SIOCCHIOCTL SIOCDEVPRIVATE

enum {
	CHELSIO_SETREG			= 1024,
	CHELSIO_GETREG			= 1025,
	CHELSIO_SETTPI                  = 1026,
	CHELSIO_GETTPI                  = 1027,
	CHELSIO_DEVUP			= 1028,
	CHELSIO_GETMTUTAB		= 1029,
	CHELSIO_SETMTUTAB		= 1030,
	CHELSIO_GETMTU			= 1031,
	CHELSIO_SET_PM			= 1032,
	CHELSIO_GET_PM			= 1033,
	CHELSIO_SET_TCAM		= 1035,
	CHELSIO_READ_TCAM_WORD		= 1037,
	CHELSIO_GET_MEM			= 1038,
	CHELSIO_GET_SGE_CONTEXT		= 1039,
	CHELSIO_GET_SGE_DESC		= 1040,
	CHELSIO_LOAD_FW			= 1041,
	CHELSIO_SET_TRACE_FILTER        = 1044,
	CHELSIO_SET_QSET_PARAMS		= 1045,
	CHELSIO_GET_QSET_PARAMS		= 1046,
	CHELSIO_SET_QSET_NUM		= 1047,
	CHELSIO_GET_QSET_NUM		= 1048,
	CHELSIO_SET_PKTSCHED		= 1049,
	CHELSIO_SET_HW_SCHED		= 1051,
	CHELSIO_LOAD_BOOT		= 1054,
	CHELSIO_CLEAR_STATS             = 1055,
	CHELSIO_GET_UP_LA		= 1056,
	CHELSIO_GET_UP_IOQS		= 1057,
	CHELSIO_GET_TRACE_FILTER	= 1058,
	CHELSIO_GET_SGE_CTXT            = 1059,
	CHELSIO_GET_SGE_DESC2		= 1060,

	CHELSIO_SET_OFLD_POLICY         = 1062,

	CHELSIO_SET_FILTER		= 1063,
	CHELSIO_DEL_FILTER		= 1064,
	CHELSIO_GET_PKTSCHED            = 1065,
	CHELSIO_LOAD_CFG                = 1066,

#if 0 /* Unsupported */
	CHELSIO_SETTPI			= 1026,
	CHELSIO_GETTPI			= 1027,
	CHELSIO_GET_TCAM		= 1034,
	CHELSIO_GET_TCB			= 1036,
	CHELSIO_GET_PROTO		= 1042,
	CHELSIO_SET_PROTO		= 1043,
#endif
};

/* statistics categories */
enum {
	STATS_PORT  = 1 << 1,
	STATS_QUEUE = 1 << 2,
};

/* queue types for "qdesc" command */
enum {
	SGE_QTYPE_TX_ETH = 1,
	SGE_QTYPE_TX_OFLD,
	SGE_QTYPE_TX_CTRL,
	SGE_QTYPE_FL,
	SGE_QTYPE_RSP,
};

struct ch_reg {
	uint32_t cmd;
	uint32_t addr;
	uint32_t val;
};

struct ch_cntxt {
	uint32_t cmd;
	uint32_t cntxt_type;
	uint32_t cntxt_id;
	uint32_t data[4];
};

/* context types */
enum {
	CNTXT_TYPE_EGRESS,
	CNTXT_TYPE_FL,
	CNTXT_TYPE_RSP,
	CNTXT_TYPE_CQ,
	CNTXT_TYPE_CONG
};

struct ch_desc {
	uint32_t cmd;
	uint32_t queue_num;
	uint32_t idx;
	uint32_t size;
	uint8_t  data[128];
};

struct ch_mem_range {
	uint32_t cmd;
	uint32_t mem_id;
	uint32_t addr;
	uint32_t len;
	uint32_t version;
	uint8_t  buf[0];
};

struct struct_load_cfg {
	uint32_t cmd;
	uint32_t len;
	uint8_t  buf[0];
};

/* ch_mem_range.mem_id values */
enum {
	MEM_CM,
	MEM_PMRX,
	MEM_PMTX,
	MEM_FLASH
};

struct ch_qset_params {
	uint32_t cmd;
	uint32_t qset_idx;
	int32_t  txq_size[3];
	int32_t  rspq_size;
	int32_t  fl_size[2];
	int32_t  intr_lat;
	int32_t  polling;
	int32_t  lro;
	int32_t  cong_thres;
	int32_t  vector;
	int32_t  qnum;
};

struct ch_pktsched_params {
	uint32_t cmd;
	uint8_t  sched;
	uint8_t  idx;
	uint8_t  min;
	uint8_t  max;
	uint8_t  binding;
};

enum {
	PKTSCHED_PORT = 0,
	PKTSCHED_TUNNELQ = 1,
};

struct ch_hw_sched {
	uint32_t cmd;
	uint8_t  sched;
	int8_t   mode;
	int8_t   channel;
	int8_t   weight;
	int32_t  kbps;
	int32_t  class_ipg;
	int32_t  flow_ipg;
};

/*
 * Filter matching rules.  These consist of a set of ingress packet field
 * (value, mask) tuples.  The associated ingress packet field matches the
 * tuple when ((field & mask) == value).  (Thus a wildcard "don't care" field
 * rule can be constructed by specifying a tuple of (0, 0).)  A filter rule
 * matches an ingress packet when all of the individual individual field
 * matching rules are true.
 *
 * Partial field masks are always valid, however, while it may be easy to
 * understand their meanings for some fields (e.g. IP address to match a
 * subnet), for others making sensible partial masks is less intuitive (e.g.
 * MPS match type) ...
 *
 * Most of the following data structures are modeled on T4 capabilities.
 * Drivers for earlier chips use the subsets which make sense for those chips.
 * We really need to come up with a hardware-independent mechanism to
 * represent hardware filter capabilities ...
 */
struct ch_filter_tuple {
	/*
	 * Compressed header matching field rules.  The TP_VLAN_PRI_MAP
	 * register selects which of these fields will participate in the
	 * filter match rules -- up to a maximum of 36 bits.  Because
	 * TP_VLAN_PRI_MAP is a global register, all filters must use the same
	 * set of fields.
	 */
	uint32_t ethtype:16;	/* Ethernet type */
	uint32_t frag:1;	/* fragmentation extension header */
	uint32_t ivlan_vld:1;	/* inner VLAN valid */
	uint32_t ovlan_vld:1;	/* outer VLAN valid */
	uint32_t pfvf_vld:1;	/* PF/VF valid */
	uint32_t macidx:9;	/* exact match MAC index */
	uint32_t fcoe:1;	/* FCoE packet */
	uint32_t iport:3;	/* ingress port */
	uint32_t matchtype:3;	/* MPS match type */
	uint32_t proto:8;	/* protocol type */
	uint32_t tos:8;		/* TOS/Traffic Type */
	uint32_t pf:8;		/* PCI-E PF ID */
	uint32_t vf:8;		/* PCI-E VF ID */
	uint32_t ivlan:16;	/* inner VLAN */
	uint32_t ovlan:16;	/* outer VLAN */

	/*
	 * Uncompressed header matching field rules.  These are always
	 * available for field rules.
	 */
	uint8_t lip[16];	/* local IP address (IPv4 in [3:0]) */
	uint8_t fip[16];	/* foreign IP address (IPv4 in [3:0]) */
	uint16_t lport;		/* local port */
	uint16_t fport;		/* foreign port */
};

/*
 * A filter ioctl command.
 */
struct ch_filter_specification {
	/*
	 * Administrative fields for filter.
	 */
	uint32_t hitcnts:1;	/* count filter hits in TCB */
	uint32_t prio:1;	/* filter has priority over active/server */

	/*
	 * Fundamental filter typing.  This is the one element of filter
	 * matching that doesn't exist as a (value, mask) tuple.
	 */
	uint32_t type:1;	/* 0 => IPv4, 1 => IPv6 */

	/*
	 * Packet dispatch information.  Ingress packets which match the
	 * filter rules will be dropped, passed to the host or switched back
	 * out as egress packets.
	 */
	uint32_t action:2;	/* drop, pass, switch */

	uint32_t rpttid:1;	/* report TID in RSS hash field */

	uint32_t dirsteer:1;	/* 0 => RSS, 1 => steer to iq */
	uint32_t iq:10;		/* ingress queue */

	uint32_t maskhash:1;	/* dirsteer=0: store RSS hash in TCB */
	uint32_t dirsteerhash:1;/* dirsteer=1: 0 => TCB contains RSS hash */
				/*             1 => TCB contains IQ ID */

	/*
	 * Switch proxy/rewrite fields.  An ingress packet which matches a
	 * filter with "switch" set will be looped back out as an egress
	 * packet -- potentially with some Ethernet header rewriting.
	 */
	uint32_t eport:2;	/* egress port to switch packet out */
	uint32_t newdmac:1;	/* rewrite destination MAC address */
	uint32_t newsmac:1;	/* rewrite source MAC address */
	uint32_t newvlan:2;	/* rewrite VLAN Tag */
	uint8_t dmac[ETH_ALEN];	/* new destination MAC address */
	uint8_t smac[ETH_ALEN];	/* new source MAC address */
	uint16_t vlan;		/* VLAN Tag to insert */

	/*
	 * Filter rule value/mask pairs.
	 */
	struct ch_filter_tuple val;
	struct ch_filter_tuple mask;
};

enum {
	FILTER_PASS = 0,	/* default */
	FILTER_DROP,
	FILTER_SWITCH
};

enum {
	VLAN_NOCHANGE = 0,	/* default */
	VLAN_REMOVE,
	VLAN_INSERT,
	VLAN_REWRITE
};

enum {                         /* Ethernet address match types */
	UCAST_EXACT = 0,       /* exact unicast match */
	UCAST_HASH  = 1,       /* inexact (hashed) unicast match */
	MCAST_EXACT = 2,       /* exact multicast match */
	MCAST_HASH  = 3,       /* inexact (hashed) multicast match */
	PROMISC     = 4,       /* no match but port is promiscuous */
	HYPPROMISC  = 5,       /* port is hypervisor-promisuous + not bcast */
	BCAST       = 6,       /* broadcast packet */
};

enum {                         /* selection of Rx queue for accepted packets */
	DST_MODE_QUEUE,        /* queue is directly specified by filter */
	DST_MODE_RSS_QUEUE,    /* filter specifies RSS entry containing queue */
	DST_MODE_RSS,          /* queue selected by default RSS hash lookup */
	DST_MODE_FILT_RSS      /* queue selected by hashing in filter-specified
				  RSS subtable */
};

struct ch_filter {
	uint32_t cmd;		/* common "cxgbtool" command header */
	uint32_t filter_id;	/* the filter index to set */
	struct ch_filter_specification fs;
};

#define MAX_NMTUS 16

struct ch_mtus {
	uint32_t cmd;
	uint32_t nmtus;
	uint16_t mtus[MAX_NMTUS];
};

struct ch_pm {
	uint32_t cmd;
	uint32_t tx_pg_sz;
	uint32_t tx_num_pg;
	uint32_t rx_pg_sz;
	uint32_t rx_num_pg;
	uint32_t pm_total;
};

struct ch_tcam {
	uint32_t cmd;
	uint32_t tcam_size;
	uint32_t nservers;
	uint32_t nroutes;
	uint32_t nfilters;
};

#define TCB_SIZE 128
#define TCB_WORDS (TCB_SIZE / 4)

struct ch_tcb {
	uint32_t cmd;
	uint32_t tcb_index;
	uint32_t tcb_data[TCB_WORDS];
};

struct ch_tcam_word {
	uint32_t cmd;
	uint32_t addr;
	uint32_t buf[3];
};

struct ch_trace {
	uint32_t cmd;
	uint32_t sip;
	uint32_t sip_mask;
	uint32_t dip;
	uint32_t dip_mask;
	uint16_t sport;
	uint16_t sport_mask;
	uint16_t dport;
	uint16_t dport_mask;
	uint32_t vlan:12;
	uint32_t vlan_mask:12;
	uint32_t intf:4;
	uint32_t intf_mask:4;
	uint8_t  proto;
	uint8_t  proto_mask;
	uint8_t  invert_match:1;
	uint8_t  config_tx:1;
	uint8_t  config_rx:1;
	uint8_t  trace_tx:1;
	uint8_t  trace_rx:1;
};

struct ch_up_la {
	uint32_t cmd;
	uint32_t stopped;
	uint32_t idx;
	uint32_t bufsize;
	uint32_t la[0];
};

struct ioq_entry {
	uint32_t ioq_cp;
	uint32_t ioq_pp;
	uint32_t ioq_alen;
	uint32_t ioq_stats;
};

struct ch_up_ioqs {
	uint32_t cmd;

	uint32_t ioq_rx_enable;
	uint32_t ioq_tx_enable;
	uint32_t ioq_rx_status;
	uint32_t ioq_tx_status;

	uint32_t bufsize;
	struct ioq_entry ioqs[0];
};

#endif /* __CXGBTOOL_H__ */
