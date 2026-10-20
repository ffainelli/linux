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

static int b53_serdes_write(struct b53_device *dev, u8 lane,
			    u8 offset, u16 block, u16 value)
{
	int ret;

	mutex_lock(&dev->reg_mutex);
	ret = b53_serdes_set_lane(dev, lane);
	if (!ret)
		ret = b53_serdes_write_blk(dev, offset, block, value);
	if (ret)
		dev->serdes_lane = B53_INVALID_LANE;
	mutex_unlock(&dev->reg_mutex);

	return ret;
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

static void b53_serdes_error(struct b53_device *dev, u8 lane, int ret)
{
	if (ret < 0)
		dev_err_ratelimited(dev->dev, "SerDes lane %u access failed: %pe\n",
				    lane, ERR_PTR(ret));
}

static int b53_serdes_config(struct phylink_pcs *pcs, unsigned int neg_mode,
			     phy_interface_t interface,
			     const unsigned long *advertising,
			     bool permit_pause_to_mac)
{
	struct b53_device *dev = pcs_to_b53_pcs(pcs)->dev;
	u8 lane = pcs_to_b53_pcs(pcs)->lane;
	int reg;

	reg = b53_serdes_read(dev, lane, B53_SERDES_DIGITAL_CONTROL(1),
			      SERDES_DIGITAL_BLK);
	if (reg < 0)
		return reg;

	if (interface == PHY_INTERFACE_MODE_1000BASEX)
		reg |= FIBER_MODE_1000X;
	else
		reg &= ~FIBER_MODE_1000X;
	return b53_serdes_write(dev, lane, B53_SERDES_DIGITAL_CONTROL(1),
				SERDES_DIGITAL_BLK, reg);
}

static void b53_serdes_an_restart(struct phylink_pcs *pcs)
{
	struct b53_device *dev = pcs_to_b53_pcs(pcs)->dev;
	u8 lane = pcs_to_b53_pcs(pcs)->lane;
	int reg;

	reg = b53_serdes_read(dev, lane, B53_SERDES_MII_REG(MII_BMCR),
			      SERDES_MII_BLK);
	if (reg < 0)
		goto out;

	reg |= BMCR_ANRESTART;
	reg = b53_serdes_write(dev, lane, B53_SERDES_MII_REG(MII_BMCR),
			       SERDES_MII_BLK, reg);
out:
	b53_serdes_error(dev, lane, reg);
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

void b53_serdes_link_set(struct b53_device *dev, int port, unsigned int mode,
			 phy_interface_t interface, bool link_up)
{
	u8 lane = b53_serdes_map_lane(dev, port);
	int reg;

	if (lane == B53_INVALID_LANE)
		return;

	reg = b53_serdes_read(dev, lane, B53_SERDES_MII_REG(MII_BMCR),
			      SERDES_MII_BLK);
	if (reg < 0)
		goto out;

	if (link_up)
		reg &= ~BMCR_PDOWN;
	else
		reg |= BMCR_PDOWN;
	reg = b53_serdes_write(dev, lane, B53_SERDES_MII_REG(MII_BMCR),
			       SERDES_MII_BLK, reg);
out:
	b53_serdes_error(dev, lane, reg);
}
EXPORT_SYMBOL(b53_serdes_link_set);

static const struct phylink_pcs_ops b53_pcs_ops = {
	.pcs_get_state = b53_serdes_get_state,
	.pcs_config = b53_serdes_config,
	.pcs_an_restart = b53_serdes_an_restart,
};

void b53_serdes_phylink_get_caps(struct b53_device *dev, int port,
				 struct phylink_config *config)
{
	u8 lane = b53_serdes_map_lane(dev, port);

	if (lane == B53_INVALID_LANE)
		return;

	switch (lane) {
	case 0:
		/* It appears lane 0 supports 2500base-X and 1000base-X */
		__set_bit(PHY_INTERFACE_MODE_2500BASEX,
			  config->supported_interfaces);
		config->mac_capabilities |= MAC_2500FD;
		fallthrough;
	case 1:
		/* It appears lane 1 only supports 1000base-X and SGMII */
		__set_bit(PHY_INTERFACE_MODE_1000BASEX,
			  config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_SGMII,
			  config->supported_interfaces);
		config->mac_capabilities |= MAC_1000FD;
		break;
	default:
		break;
	}
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

	if (!phy_interface_mode_is_8023z(interface) &&
	    interface != PHY_INTERFACE_MODE_SGMII)
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

	return 0;
}
EXPORT_SYMBOL(b53_serdes_init);

MODULE_AUTHOR("Florian Fainelli <f.fainelli@gmail.com>");
MODULE_DESCRIPTION("B53 Switch SerDes driver");
MODULE_LICENSE("Dual BSD/GPL");
