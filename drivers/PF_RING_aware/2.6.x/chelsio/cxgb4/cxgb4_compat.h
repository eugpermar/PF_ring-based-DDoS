/*
 * This file is part of the Chelsio T4 Ethernet driver.
 *
 * Copyright (C) 2003-2009 Chelsio Communications.  All rights reserved.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the LICENSE file included in this
 * release for licensing terms and conditions.
 */

/*
 * This file is used to allow the driver to be compiled under multiple
 * versions of Linux with as few obtrusive in-line #ifdef's as possible.
 */

#ifndef __CXGB4_COMPAT_H
#define __CXGB4_COMPAT_H

#include <linux/version.h>

/*
 * Set a /proc node's module owner field.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
#define SET_PROC_NODE_OWNER(_p, _owner) \
	do { (_p)->owner = (_owner); } while (0)
#else
#define SET_PROC_NODE_OWNER(_p, _owner) \
	do { } while (0)
#endif

/*
 * Collect up to maxaddrs worth of a netdevice's unicast addresses, starting
 * at a specified offset within the list, into an array of addrss pointers and
 * return the number collected.
 */
static inline unsigned int collect_netdev_uc_list_addrs(const struct net_device *dev,
							const u8 **addr,
							unsigned int offset,
							unsigned int maxaddrs)
{
	unsigned int index = 0;
	unsigned int naddr = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
	const struct dev_addr_list *da;

	for (da = dev->uc_list; da && naddr < maxaddrs; da = da->next)
		if (index++ >= offset)
			addr[naddr++] = da->dmi_addr;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
	const struct netdev_hw_addr *ha;

	list_for_each_entry(ha, &dev->uc.list, list)
		if (index++ >= offset) {
			addr[naddr++] = ha->addr;
			if (naddr >= maxaddrs)
				break;
		}
#else
	const struct netdev_hw_addr *ha;

	netdev_for_each_uc_addr(ha, dev)
		if (index++ >= offset) {
			addr[naddr++] = ha->addr;
			if (naddr >= maxaddrs)
				break;
		}
#endif
	return naddr;
}

/*
 * Collect up to maxaddrs worth of a netdevice's multicast addresses, starting
 * at a specified offset within the list, into an array of addrss pointers and
 * return the number collected.
 */
static inline unsigned int collect_netdev_mc_list_addrs(const struct net_device *dev,
							const u8 **addr,
							unsigned int offset,
							unsigned int maxaddrs)
{
	unsigned int index = 0;
	unsigned int naddr = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
	const struct dev_addr_list *da;

	for (da = dev->mc_list; da && naddr < maxaddrs; da = da->next)
		if (index++ >= offset)
			addr[naddr++] = da->dmi_addr;
#elif LINUX_VERSION_CODE == KERNEL_VERSION(2,6,34)
	const struct dev_mc_list *mclist;

	netdev_for_each_mc_addr(mclist, dev)
		if (index++ >= offset) {
			addr[naddr++] = mclist->dmi_addr;
			if (naddr >= maxaddrs)
				break;
		}
#else
	const struct netdev_hw_addr *ha;

	netdev_for_each_mc_addr(ha, dev)
		if (index++ >= offset) {
			addr[naddr++] = ha->addr;
			if (naddr >= maxaddrs)
				break;
		}
#endif
	return naddr;
}

#ifndef NIPQUAD
#define NIPQUAD(addr) \
	((unsigned char *)&addr)[0], \
	((unsigned char *)&addr)[1], \
	((unsigned char *)&addr)[2], \
	((unsigned char *)&addr)[3]
#endif

#ifndef NIPQUAD_FMT
#define NIPQUAD_FMT "%u.%u.%u.%u"
#endif

#endif  /* !__CXGB4_COMPAT_H */
