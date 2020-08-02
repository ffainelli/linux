// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2020, Florian Fainelli <f.fainelli@gmail.com>
 */
#include <linux/if_vlan.h>
#include <linux/dsa/8021q.h>
#include <linux/dsa/loop.h>
#include "dsa_priv.h"

static bool dsa_loop_vlan_ok(struct dsa_loop_priv *ps, u16 vid, int port)
{
	struct dsa_loop_vlan *vlan;

	/* This should not happen */
	if (vid >= VLAN_N_VID)
		return false;

	vlan = &ps->vlans[vid];

	return !!(BIT(port) & vlan->members);
}

static struct sk_buff *dsa_loop_xmit(struct sk_buff *skb,
				     struct net_device *netdev)
{
	struct dsa_port *dp = dsa_slave_to_port(netdev);
	struct dsa_loop_port *port = dp->priv;
	struct dsa_switch *ds = dp->ds;
	u16 tx_vid;
	u8 pcp;

	tx_vid = dsa_8021q_tx_vid(ds, dp->index);
	pcp = netdev_txq_to_tc(netdev, skb_get_queue_mapping(skb));

	if (dsa_port_is_vlan_filtering(dp) &&
	    !dsa_loop_vlan_ok(ds->priv, tx_vid, dp->index)) {
		port->mib[DSA_LOOP_VLAN_EGRESS_VID_ERR].val++;
		return skb;
	}

	port->mib[DSA_LOOP_VLAN_EGRESS_VID_OK].val++;

	return dsa_8021q_xmit(skb, netdev, ETH_P_8021Q,
			     ((pcp << VLAN_PRIO_SHIFT) | tx_vid));
}

static struct sk_buff *dsa_loop_rcv(struct sk_buff *skb,
				    struct net_device *netdev,
				    struct packet_type *pt)
{
	int source_port, switch_id;
	struct dsa_loop_port *port;
	struct dsa_port *dp;
	u16 vid;

	vid = skb_vlan_tag_get(skb);

	source_port = dsa_8021q_rx_source_port(vid);
	switch_id = dsa_8021q_rx_switch_id(vid);

	skb->dev = dsa_master_find_slave(netdev, switch_id, source_port);
	if (!skb->dev) {
		netdev_warn(netdev,
			    "Decode error, VID: %d, port: %d, switch: %d\n",
			    vid, source_port, switch_id);
		return NULL;
	}

	dp = dsa_slave_to_port(skb->dev);
	port = dp->priv;
	if (dsa_port_is_vlan_filtering(dp) &&
	    !dsa_loop_vlan_ok(port->ps, vid, dp->index)) {
		port->mib[DSA_LOOP_VLAN_INGRESS_VID_ERR].val++;
		return NULL;
	}

	port->mib[DSA_LOOP_VLAN_INGRESS_VID_OK].val++;

	return skb;
}

static const struct dsa_device_ops dsa_loop_netdev_ops = {
	.name		= "dsa-loop",
	.proto		= DSA_TAG_PROTO_LOOP,
	.overhead	= VLAN_HLEN,
	.xmit		= dsa_loop_xmit,
	.rcv		= dsa_loop_rcv,
};
module_dsa_tag_driver(dsa_loop_netdev_ops);

MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_LOOP);
MODULE_LICENSE("GPL");
