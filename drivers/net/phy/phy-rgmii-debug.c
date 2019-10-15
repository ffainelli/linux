// SPDX-License-Identifier: GPL-2.0+
/*
 * PHY library RGMII debugging tool.
 *
 * Author: Florian Fainelli <f.fainelli@gmail.com>
 */
#include <linux/completion.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/phy.h>
#include <linux/workqueue.h>
#include <linux/etherdevice.h>
#include <linux/crc32.h>

#include <uapi/linux/if_ether.h>

struct phy_rgmii_probe_skb {
	struct sk_buff *skb;
	u32 fcs;
};

struct phy_rgmii_debug_priv {
	struct phy_device *phydev;
	struct work_struct work;
	struct completion compl;
	struct phy_rgmii_probe_skb prb;
	unsigned int rcv_ok;
};

static u32 phy_rgmii_probe_skb_fcs(struct sk_buff *skb)
{
	u32 fcs;

	fcs = crc32_le(~0, skb->data, skb->len);
	fcs = ~fcs;

	return fcs;
}

static int phy_rgmii_debug_rcv(struct sk_buff *skb, struct net_device *dev,
			       struct packet_type *pt, struct net_device *unused)
{
	struct phy_rgmii_debug_priv *priv = pt->af_packet_priv;
	struct phy_device *phydev = priv->phydev;
	u32 fcs = phy_rgmii_probe_skb_fcs(skb);

	if (skb->len != priv->prb.skb->len || fcs != priv->prb.fcs) {
		phydev_warn(phydev, "Incorrect reply received!\n");
		goto out;
	}

	print_hex_dump(KERN_INFO, "looped probe skb: ", DUMP_PREFIX_OFFSET,
		       16, 1, skb->data, skb->len, false);

	priv->rcv_ok = 1;
out:
	complete(&priv->compl);
	return 0;
}

static int phy_rgmii_trigger_config(struct phy_device *phydev,
				    phy_interface_t interface)
{
	int ret = 0;

	phy_stop(phydev);

	phydev->interface = interface;

	if (phydev->drv->config_init)
		ret = phydev->drv->config_init(phydev);
	if (ret)
		return ret;

	ret = phy_start_aneg(phydev);
	if (ret)
		return ret;

	phy_start(phydev);

	return 0;
}

static void phy_rgmii_probe_xmit_work(struct work_struct *work)
{
	struct phy_rgmii_debug_priv *priv;

	priv = container_of(work, struct phy_rgmii_debug_priv, work);

	dev_queue_xmit(priv->prb.skb);
}

static int phy_rgmii_prepare_probe(struct phy_rgmii_debug_priv *priv)
{
	struct phy_device *phydev = priv->phydev;
	struct net_device *ndev = phydev->attached_dev;
	struct phy_rgmii_probe_skb *prb = &priv->prb;
	struct sk_buff *skb = prb->skb;
	int ret;

	skb = netdev_alloc_skb(ndev, ndev->mtu);
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0xaa, skb->len);

	/* Build the header */
	ret = eth_header(skb, ndev, ETH_P_EDSA, ndev->dev_addr,
			 NULL, ndev->mtu);
	if (ret != ETH_HLEN) {
		kfree_skb(skb);
		return -EINVAL;
	}

	prb->fcs = phy_rgmii_probe_skb_fcs(skb);

	phydev_info(priv->phydev, "Probe SKB has FCS: 0x%08x\n", prb->fcs);
	print_hex_dump(KERN_INFO, "probe skb: ", DUMP_PREFIX_OFFSET,
		       16, 1, skb->data, skb->len, false);

	return 0;
}

static int phy_rgmii_probe_interface(struct phy_rgmii_debug_priv *priv,
				     phy_interface_t iface)
{
	struct phy_device *phydev = priv->phydev;
	unsigned long timeout;
	int ret;

	ret = phy_rgmii_trigger_config(phydev, iface);
	if (ret) {
		phydev_err(phydev, "%s rejected by driver(s)\n", phy_modes(iface));
		return ret;
	}

	phydev_info(phydev, "Trying \"%s\"\n", phy_modes(iface));

	/* Prepare probe frames now */
	ret = phy_rgmii_prepare_probe(priv);
	if (ret)
		return ret;

	INIT_WORK(&priv->work, phy_rgmii_probe_xmit_work);
	reinit_completion(&priv->compl);
	priv->rcv_ok = 0;

	schedule_work(&priv->work);

	timeout = wait_for_completion_timeout(&priv->compl, 1000);
	if (!timeout) {
		phydev_err(phydev, "transmit timeout!\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	if (priv->rcv_ok != 1)
		ret = -EINVAL;
	else
		ret = 0;
out:
	dev_consume_skb_any(priv->prb.skb);
	return ret;
}

static struct packet_type phy_rgmii_probes_type __read_mostly = {
	.type	= cpu_to_be16(ETH_P_EDSA),
	.func	= phy_rgmii_debug_rcv,
};

static int phy_rgmii_can_debug(struct phy_device *phydev)
{
	if (!phy_interface_is_rgmii(phydev)) {
		phydev_info(phydev, "Not RGMII configured, nothing to do\n");
		return 0;
	}

	if (!phydev->is_gigabit_capable) {
		phydev_err(phydev, "not relevant in non-Gigabit mode\n");
		return -EOPNOTSUPP;
	}

	if (phy_driver_is_genphy(phydev) || phy_driver_is_genphy_10g(phydev)) {
		phydev_err(phydev, "only relevant with non-generic drivers\n");
		return -EOPNOTSUPP;
	}

	if (!phydev->attached_dev) {
		phydev_err(phydev, "No network device attached\n");
		return -EOPNOTSUPP;
	}

	if (!phydev->drv->set_loopback) {
		phydev_err(phydev, "PHY driver does not support loopback\n");
		return -EOPNOTSUPP;
	}

	return 1;
}

int phy_rgmii_debug_probe(struct phy_device *phydev)
{
	struct net_device *ndev = phydev->attached_dev;
	unsigned char operstate = ndev->operstate;
	phy_interface_t rgmii_modes[4] = {
		PHY_INTERFACE_MODE_RGMII,
		PHY_INTERFACE_MODE_RGMII_ID,
		PHY_INTERFACE_MODE_RGMII_RXID,
		PHY_INTERFACE_MODE_RGMII_TXID
	};
	struct phy_rgmii_debug_priv *priv;
	unsigned int i, count = 0;
	int ret;

	ret = phy_rgmii_can_debug(phydev);
	if (ret <= 0)
		return ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (phy_rgmii_probes_type.af_packet_priv)
		return -EBUSY;

	phy_rgmii_probes_type.af_packet_priv = priv;
	priv->phydev = phydev;

	/* Put the PHY in loopback mode */
	ret = phy_loopback(phydev, true);
	if (ret) {
		phydev_err(phydev, "Failed to set PHY in loopback mode\n");
		goto out;
	}

	/* We are now testing this network device */
	ndev->operstate = IF_OPER_TESTING;

	dev_add_pack(&phy_rgmii_probes_type);

	/* Determine where to start */
	for (i = 0; i < ARRAY_SIZE(rgmii_modes); i++) {
		if (phydev->interface == rgmii_modes[i])
			break;
	}

	/* Now probe all modes */
	do {
		ret = phy_rgmii_probe_interface(priv, rgmii_modes[i]);
		if (ret == 0) {
			phydev_info(phydev, "Determined \"%s\" to be correct\n",
				    phy_modes(rgmii_modes[i]));
			break;
		}
		count++;
		i = (i + 1) % ARRAY_SIZE(rgmii_modes);
	} while (count < ARRAY_SIZE(rgmii_modes));

	ret = phy_loopback(phydev, false);
	if (ret)
		phydev_err(phydev, "Failed to restore loopback mode\n");

	dev_remove_pack(&phy_rgmii_probes_type);
out:
	kfree(priv);
	phy_rgmii_probes_type.af_packet_priv = NULL;
	ndev->operstate = operstate;
	return ret;
}
EXPORT_SYMBOL_GPL(phy_rgmii_debug_probe);
