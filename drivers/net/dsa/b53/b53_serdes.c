// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Northstar Plus switch SerDes/SGMII PHY main logic
 *
 * Copyright (C) 2018 Florian Fainelli <f.fainelli@gmail.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <net/dsa.h>

#include "b53_priv.h"
#include "b53_serdes.h"
#include "b53_regs.h"

static inline struct b53_pcs *pcs_to_b53_pcs(struct phylink_pcs *pcs)
{
	return container_of(pcs, struct b53_pcs, pcs);
}

/* The caller holds reg_mutex across lane, block and data accesses. */
static int b53_serdes_write_blk(struct b53_device *dev, u8 offset, u16 block,
				u16 value)
{
	int ret;

	lockdep_assert_held(&dev->reg_mutex);

	ret = dev->ops->write16(dev, B53_SERDES_PAGE, B53_SERDES_BLKADDR,
				block);
	if (ret)
		return ret;

	return dev->ops->write16(dev, B53_SERDES_PAGE, offset, value);
}

static int b53_serdes_read_blk(struct b53_device *dev, u8 offset, u16 block)
{
	u16 value;
	int ret;

	lockdep_assert_held(&dev->reg_mutex);

	ret = dev->ops->write16(dev, B53_SERDES_PAGE, B53_SERDES_BLKADDR,
				block);
	if (ret)
		return ret;

	ret = dev->ops->read16(dev, B53_SERDES_PAGE, offset, &value);
	return ret ? ret : value;
}

static int b53_serdes_set_lane(struct b53_device *dev, u8 lane)
{
	int ret;

	lockdep_assert_held(&dev->reg_mutex);

	if (lane >= B53_N_PCS)
		return -EINVAL;

	if (dev->serdes_lane == lane)
		return 0;

	ret = b53_serdes_write_blk(dev, B53_SERDES_LANE,
				   SERDES_XGXSBLK0_BLOCKADDRESS, lane);
	if (ret)
		return ret;

	dev->serdes_lane = lane;
	return 0;
}

static int b53_serdes_read(struct b53_device *dev, u8 lane,
			   u8 offset, u16 block)
{
	int ret;

	mutex_lock(&dev->reg_mutex);
	ret = b53_serdes_set_lane(dev, lane);
	if (!ret)
		ret = b53_serdes_read_blk(dev, offset, block);
	if (ret < 0)
		dev->serdes_lane = B53_INVALID_LANE;
	mutex_unlock(&dev->reg_mutex);

	return ret;
}

/* Return one if the register changed, zero if unchanged, or an error. */
static int b53_serdes_modify(struct b53_device *dev, u8 lane, u8 offset,
			     u16 block, u16 mask, u16 set)
{
	int ret, old;
	u16 new;

	mutex_lock(&dev->reg_mutex);
	ret = b53_serdes_set_lane(dev, lane);
	if (ret)
		goto out;

	old = b53_serdes_read_blk(dev, offset, block);
	if (old < 0) {
		ret = old;
		goto out;
	}

	new = (old & ~mask) | set;
	if (new == old)
		goto out;

	ret = b53_serdes_write_blk(dev, offset, block, new);
	if (!ret)
		ret = 1;
out:
	if (ret < 0)
		dev->serdes_lane = B53_INVALID_LANE;
	mutex_unlock(&dev->reg_mutex);

	return ret;
}

static void b53_serdes_error(struct b53_device *dev, u8 lane, int ret)
{
	if (ret < 0)
		dev_err_ratelimited(dev->dev, "SerDes lane %u access failed: %pe\n",
				    lane, ERR_PTR(ret));
}

static unsigned int b53_serdes_inband_caps(struct phylink_pcs *pcs,
					   phy_interface_t interface)
{
	switch (interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		return LINK_INBAND_DISABLE | LINK_INBAND_ENABLE;
	default:
		return 0;
	}
}

static int b53_serdes_force_link(struct b53_pcs *priv, int speed, int duplex)
{
	struct b53_device *dev = priv->dev;
	u8 lane = priv->lane;
	u16 bmcr = mii_bmcr_encode_fixed(speed, duplex);
	u16 mask = BMCR_SPEED100 | BMCR_SPEED1000 | BMCR_FULLDPLX;
	int old, misc, rx, fifo, ret, err;

	old = b53_serdes_read(dev, lane, B53_SERDES_MII_REG(MII_BMCR),
			      SERDES_MII_BLK);
	if (old < 0)
		return old;
	misc = b53_serdes_read(dev, lane, B53_SERDES_DIGITAL_MISC1,
			       SERDES_DIGITAL_BLK);
	if (misc < 0)
		return misc;
	rx = b53_serdes_read(dev, lane, B53_SERDES_RX_CONTROL, SERDES_RX_BLK);
	if (rx < 0)
		return rx;
	fifo = b53_serdes_read(dev, lane, B53_SERDES_DIGITAL_CONTROL(3),
			       SERDES_DIGITAL_BLK);
	if (fifo < 0)
		return fifo;

	if ((old & mask) == bmcr && !(misc & FORCE_SPEED_MASK) &&
	    !(rx & RX_SEQ_START) && !(fifo & TX_FIFO_RST))
		return 0;

	/* Hold the receiver and TX FIFO while switching the forced speed,
	 * following the downstream robo SerDes programming sequence.
	 */
	ret = b53_serdes_modify(dev, lane, B53_SERDES_RX_CONTROL,
				SERDES_RX_BLK, RX_SEQ_START, RX_SEQ_START);
	if (ret < 0)
		goto release;

	ret = b53_serdes_modify(dev, lane, B53_SERDES_DIGITAL_CONTROL(3),
				SERDES_DIGITAL_BLK, TX_FIFO_RST, TX_FIFO_RST);
	if (ret < 0)
		goto release;

	ret = b53_serdes_modify(dev, lane, B53_SERDES_DIGITAL_MISC1,
				SERDES_DIGITAL_BLK, FORCE_SPEED_MASK, 0);
	if (ret >= 0)
		ret = b53_serdes_modify(dev, lane, B53_SERDES_MII_REG(MII_BMCR),
					SERDES_MII_BLK, mask, bmcr);
release:
	/* A failed write may have reached the device. Release both holds. */
	err = b53_serdes_modify(dev, lane, B53_SERDES_RX_CONTROL,
				SERDES_RX_BLK, RX_SEQ_START, 0);
	if (ret >= 0)
		ret = err;
	err = b53_serdes_modify(dev, lane, B53_SERDES_DIGITAL_CONTROL(3),
				SERDES_DIGITAL_BLK, TX_FIFO_RST, 0);
	if (ret >= 0)
		ret = err;

	return ret < 0 ? ret : 1;
}

static int b53_serdes_config(struct phylink_pcs *pcs, unsigned int neg_mode,
			     phy_interface_t interface,
			     const unsigned long *advertising,
			     bool permit_pause_to_mac)
{
	struct b53_pcs *priv = pcs_to_b53_pcs(pcs);
	struct b53_device *dev = priv->dev;
	u8 lane = priv->lane;
	int adv, ret, changed;
	u16 ctrl, bmcr;

	if (!test_bit(interface, pcs->supported_interfaces))
		return -EINVAL;

	adv = phylink_mii_c22_pcs_encode_advertisement(interface, advertising);
	if (adv < 0)
		return adv;

	ctrl = interface == PHY_INTERFACE_MODE_1000BASEX ? FIBER_MODE_1000X : 0;
	/* Select the protocol explicitly and act as the MAC in SGMII mode. */
	ret = b53_serdes_modify(dev, lane, B53_SERDES_DIGITAL_CONTROL(1),
				SERDES_DIGITAL_BLK,
				FIBER_MODE_1000X | AUTODET_EN | SGMII_MASTER_MODE,
				ctrl);
	if (ret < 0)
		return ret;
	changed = ret;

	ret = b53_serdes_modify(dev, lane, B53_SERDES_MII_REG(MII_ADVERTISE),
				SERDES_MII_BLK, 0xffff, adv);
	if (ret < 0)
		return ret;
	changed |= ret;

	if (interface == PHY_INTERFACE_MODE_1000BASEX &&
	    neg_mode != PHYLINK_PCS_NEG_INBAND_ENABLED) {
		/* The baud rate must be set before waiting for PCS link-up. */
		ret = b53_serdes_force_link(priv, SPEED_1000, DUPLEX_FULL);
	} else {
		ret = b53_serdes_modify(dev, lane, B53_SERDES_DIGITAL_MISC1,
					SERDES_DIGITAL_BLK, FORCE_SPEED_MASK, 0);
	}
	if (ret < 0)
		return ret;
	changed |= ret;

	bmcr = neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED ? BMCR_ANENABLE : 0;
	/* phylink requests Clause 37 restarts separately, but not SGMII. */
	if (bmcr && changed && interface == PHY_INTERFACE_MODE_SGMII)
		bmcr |= BMCR_ANRESTART;
	ret = b53_serdes_modify(dev, lane, B53_SERDES_MII_REG(MII_BMCR),
				SERDES_MII_BLK,
				BMCR_ANENABLE | BMCR_ISOLATE | BMCR_ANRESTART, bmcr);
	if (ret < 0)
		return ret;

	return changed | ret;
}

static void b53_serdes_an_restart(struct phylink_pcs *pcs)
{
	struct b53_pcs *priv = pcs_to_b53_pcs(pcs);
	int ret;

	ret = b53_serdes_modify(priv->dev, priv->lane,
				B53_SERDES_MII_REG(MII_BMCR), SERDES_MII_BLK,
				BMCR_ANRESTART, BMCR_ANRESTART);
	b53_serdes_error(priv->dev, priv->lane, ret);
}

static void b53_serdes_link_up(struct phylink_pcs *pcs, unsigned int neg_mode,
			       phy_interface_t interface, int speed, int duplex)
{
	struct b53_pcs *priv = pcs_to_b53_pcs(pcs);
	int ret;

	if (interface != PHY_INTERFACE_MODE_SGMII ||
	    neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED)
		return;

	ret = b53_serdes_force_link(priv, speed, duplex);
	b53_serdes_error(priv->dev, priv->lane, ret);
}

static void b53_serdes_get_state(struct phylink_pcs *pcs, unsigned int neg_mode,
				 struct phylink_link_state *state)
{
	struct b53_device *dev = pcs_to_b53_pcs(pcs)->dev;
	u8 lane = pcs_to_b53_pcs(pcs)->lane;
	int dig, bmsr;

	dig = b53_serdes_read(dev, lane, B53_SERDES_DIGITAL_STATUS,
			      SERDES_DIGITAL_BLK);
	if (dig < 0) {
		state->link = false;
		b53_serdes_error(dev, lane, dig);
		return;
	}

	bmsr = b53_serdes_read(dev, lane, B53_SERDES_MII_REG(MII_BMSR),
			       SERDES_MII_BLK);
	if (bmsr < 0) {
		state->link = false;
		b53_serdes_error(dev, lane, bmsr);
		return;
	}

	switch ((dig >> SPEED_STATUS_SHIFT) & SPEED_STATUS_MASK) {
	case SPEED_STATUS_10:
		state->speed = SPEED_10;
		break;
	case SPEED_STATUS_100:
		state->speed = SPEED_100;
		break;
	case SPEED_STATUS_1000:
		state->speed = SPEED_1000;
		break;
	default:
	case SPEED_STATUS_2500:
		state->speed = SPEED_2500;
		break;
	}

	state->duplex = dig & DUPLEX_STATUS ? DUPLEX_FULL : DUPLEX_HALF;
	state->an_complete = !!(bmsr & BMSR_ANEGCOMPLETE);
	state->link = !!(dig & LINK_STATUS);
	if (dig & PAUSE_RESOLUTION_RX_SIDE)
		state->pause |= MLO_PAUSE_RX;
	if (dig & PAUSE_RESOLUTION_TX_SIDE)
		state->pause |= MLO_PAUSE_TX;
}

static int b53_serdes_enable(struct phylink_pcs *pcs)
{
	struct b53_pcs *priv = pcs_to_b53_pcs(pcs);
	int ret;

	ret = b53_serdes_modify(priv->dev, priv->lane,
				B53_SERDES_MII_REG(MII_BMCR), SERDES_MII_BLK,
				BMCR_PDOWN, 0);
	b53_serdes_error(priv->dev, priv->lane, ret);
	return ret < 0 ? ret : 0;
}

static void b53_serdes_disable(struct phylink_pcs *pcs)
{
	struct b53_pcs *priv = pcs_to_b53_pcs(pcs);
	int ret;

	ret = b53_serdes_modify(priv->dev, priv->lane,
				B53_SERDES_MII_REG(MII_BMCR), SERDES_MII_BLK,
				BMCR_PDOWN, BMCR_PDOWN);
	b53_serdes_error(priv->dev, priv->lane, ret);
}

static const struct phylink_pcs_ops b53_pcs_ops = {
	.pcs_inband_caps = b53_serdes_inband_caps,
	.pcs_enable = b53_serdes_enable,
	.pcs_disable = b53_serdes_disable,
	.pcs_get_state = b53_serdes_get_state,
	.pcs_config = b53_serdes_config,
	.pcs_an_restart = b53_serdes_an_restart,
	.pcs_link_up = b53_serdes_link_up,
};

void b53_serdes_phylink_get_caps(struct b53_device *dev, int port,
				 struct phylink_config *config)
{
	u8 lane = b53_serdes_map_lane(dev, port);

	if (lane == B53_INVALID_LANE)
		return;

	if (lane >= B53_N_PCS || !dev->pcs[lane].dev)
		return;

	phy_interface_copy(config->supported_interfaces,
			   dev->pcs[lane].pcs.supported_interfaces);
}
EXPORT_SYMBOL(b53_serdes_phylink_get_caps);

struct phylink_pcs *b53_serdes_phylink_mac_select_pcs(struct b53_device *dev,
						      int port,
						      phy_interface_t interface)
{
	u8 lane = b53_serdes_map_lane(dev, port);

	if (lane == B53_INVALID_LANE || lane >= B53_N_PCS ||
	    !dev->pcs[lane].dev)
		return NULL;

	if (!test_bit(interface, dev->pcs[lane].pcs.supported_interfaces))
		return NULL;

	return &dev->pcs[lane].pcs;
}
EXPORT_SYMBOL(b53_serdes_phylink_mac_select_pcs);

int b53_serdes_init(struct b53_device *dev, int port)
{
	u8 lane = b53_serdes_map_lane(dev, port);
	struct b53_pcs *pcs;
	int id0, msb, lsb;

	if (lane >= B53_N_PCS)
		return -EINVAL;

	id0 = b53_serdes_read(dev, lane, B53_SERDES_ID0, SERDES_ID0);
	if (id0 < 0)
		return id0;

	msb = b53_serdes_read(dev, lane, B53_SERDES_MII_REG(MII_PHYSID1),
			      SERDES_MII_BLK);
	if (msb < 0)
		return msb;

	lsb = b53_serdes_read(dev, lane, B53_SERDES_MII_REG(MII_PHYSID2),
			      SERDES_MII_BLK);
	if (lsb < 0)
		return lsb;

	if (id0 == 0 || id0 == 0xffff) {
		dev_err(dev->dev, "SerDes not initialized, check settings\n");
		return -ENODEV;
	}

	dev_info(dev->dev,
		 "SerDes lane %d, model: %d, rev %c%d (OUI: 0x%08x)\n",
		 lane, id0 & SERDES_ID0_MODEL_MASK,
		 (id0 >> SERDES_ID0_REV_LETTER_SHIFT) + 0x41,
		 (id0 >> SERDES_ID0_REV_NUM_SHIFT) & SERDES_ID0_REV_NUM_MASK,
		 (u32)msb << 16 | lsb);

	pcs = &dev->pcs[lane];
	pcs->dev = dev;
	pcs->lane = lane;
	pcs->pcs.ops = &b53_pcs_ops;
	__set_bit(PHY_INTERFACE_MODE_SGMII, pcs->pcs.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_1000BASEX, pcs->pcs.supported_interfaces);

	return 0;
}
EXPORT_SYMBOL(b53_serdes_init);

MODULE_AUTHOR("Florian Fainelli <f.fainelli@gmail.com>");
MODULE_DESCRIPTION("B53 Switch SerDes driver");
MODULE_LICENSE("Dual BSD/GPL");
