// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Exercise the PCS callbacks through a fake banked switch register bus. */
#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/mii.h>

#include "b53_priv.h"
#include "b53_serdes.h"

enum b53_test_block {
	TEST_MII,
	TEST_DIGITAL,
	TEST_ID,
	TEST_RX,
	TEST_LANE,
};

struct b53_serdes_test {
	struct b53_device *dev;
	u16 regs[B53_N_PCS][4][16];
	u8 mac[256];
	u16 block;
	u8 lane;
	unsigned int accesses;
	unsigned int fail_at;
	bool slow;
	bool fail_mac_read;
	bool fail_mac_write;
	unsigned int mac_writes;
};

static int b53_test_block(u16 block)
{
	switch (block) {
	case SERDES_MII_BLK:
		return TEST_MII;
	case SERDES_DIGITAL_BLK:
		return TEST_DIGITAL;
	case SERDES_ID0:
		return TEST_ID;
	case SERDES_RX_BLK:
		return TEST_RX;
	case SERDES_XGXSBLK0_BLOCKADDRESS:
		return TEST_LANE;
	default:
		return -EINVAL;
	}
}

static int b53_test_access(struct b53_device *dev)
{
	struct b53_serdes_test *ctx = dev->priv;

	lockdep_assert_held(&dev->reg_mutex);
	if (ctx->slow)
		usleep_range(50, 100);
	return ++ctx->accesses == ctx->fail_at ? -EIO : 0;
}

static int b53_test_read16(struct b53_device *dev, u8 page, u8 reg, u16 *val)
{
	struct b53_serdes_test *ctx = dev->priv;
	int ret = b53_test_access(dev);
	int block = b53_test_block(ctx->block);

	if (ret)
		return ret;
	if (page != B53_SERDES_PAGE || block < 0 || block == TEST_LANE ||
	    reg < 0x20 || reg > 0x3c || (reg & 1))
		return -EINVAL;
	*val = ctx->regs[ctx->lane][block][(reg - 0x20) / 2];
	return 0;
}

static int b53_test_write16(struct b53_device *dev, u8 page, u8 reg, u16 val)
{
	struct b53_serdes_test *ctx = dev->priv;
	int ret = b53_test_access(dev);
	int block = b53_test_block(ctx->block);

	if (ret)
		return ret;
	if (page != B53_SERDES_PAGE)
		return -EINVAL;
	if (reg == B53_SERDES_BLKADDR) {
		ctx->block = val;
		return 0;
	}
	if (block == TEST_LANE && reg == B53_SERDES_LANE) {
		if (val >= B53_N_PCS)
			return -EINVAL;
		ctx->lane = val;
		return 0;
	}
	if (block < 0 || block == TEST_LANE || reg < 0x20 || reg > 0x3c ||
	    (reg & 1))
		return -EINVAL;
	/* The restart bit self-clears in hardware. */
	if (block == TEST_MII && reg == B53_SERDES_MII_REG(MII_BMCR))
		val &= ~BMCR_ANRESTART;
	ctx->regs[ctx->lane][block][(reg - 0x20) / 2] = val;
	return 0;
}

static int b53_test_read8(struct b53_device *dev, u8 page, u8 reg, u8 *val)
{
	struct b53_serdes_test *ctx = dev->priv;

	if (page != B53_CTRL_PAGE)
		return -EINVAL;
	if (ctx->fail_mac_read)
		return -EIO;
	*val = ctx->mac[reg];
	return 0;
}

static int b53_test_write8(struct b53_device *dev, u8 page, u8 reg, u8 val)
{
	struct b53_serdes_test *ctx = dev->priv;

	if (page != B53_CTRL_PAGE)
		return -EINVAL;
	ctx->mac_writes++;
	if (ctx->fail_mac_write)
		return -EIO;
	ctx->mac[reg] = val;
	return 0;
}

static u8 b53_test_map_lane(struct b53_device *dev, int port)
{
	return port == 5 ? 0 : port == 4 ? 1 : B53_INVALID_LANE;
}

static const struct b53_io_ops b53_test_ops = {
	.read8 = b53_test_read8,
	.write8 = b53_test_write8,
	.read16 = b53_test_read16,
	.write16 = b53_test_write16,
	.serdes_map_lane = b53_test_map_lane,
};

static int b53_serdes_test_init(struct kunit *test)
{
	struct b53_serdes_test *ctx;
	struct device *dev;
	int i;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	dev = kunit_device_register(test, "b53-serdes-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	ctx->dev = b53_switch_alloc(dev, &b53_test_ops, ctx);
	KUNIT_ASSERT_NOT_NULL(test, ctx->dev);
	/* Simulate firmware leaving lane 1 selected before driver probe. */
	ctx->lane = 1;
	ctx->dev->chip_id = BCM58XX_DEVICE_ID;
	ctx->dev->imp_port = 8;
	test->priv = ctx;

	for (i = 0; i < B53_N_PCS; i++) {
		ctx->regs[i][TEST_ID][0] = 1;
		KUNIT_ASSERT_EQ(test, b53_serdes_init(ctx->dev, 5 - i), 0);
	}
	return 0;
}

static void b53_serdes_discovery_test(struct kunit *test)
{
	struct b53_serdes_test *ctx = test->priv;
	struct phylink_config config = {};
	struct phylink_pcs *pcs;

	b53_serdes_phylink_get_caps(ctx->dev, 5, &config);
	KUNIT_EXPECT_TRUE(test, test_bit(PHY_INTERFACE_MODE_1000BASEX,
					 config.supported_interfaces));
	KUNIT_EXPECT_TRUE(test, test_bit(PHY_INTERFACE_MODE_SGMII,
					 config.supported_interfaces));
	pcs = b53_serdes_phylink_mac_select_pcs(ctx->dev, 5,
						PHY_INTERFACE_MODE_2500BASEX);
	KUNIT_EXPECT_PTR_EQ(test, pcs, NULL);
	KUNIT_EXPECT_EQ(test, b53_serdes_init(ctx->dev, 0), -EINVAL);

	ctx->dev->pcs[0].dev = NULL;
	ctx->regs[0][TEST_ID][0] = 0xffff;
	KUNIT_EXPECT_EQ(test, b53_serdes_init(ctx->dev, 5), -ENODEV);
	phy_interface_zero(config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_GMII, config.supported_interfaces);
	b53_serdes_phylink_get_caps(ctx->dev, 5, &config);
	KUNIT_EXPECT_TRUE(test, test_bit(PHY_INTERFACE_MODE_GMII,
					 config.supported_interfaces));
	KUNIT_EXPECT_FALSE(test, test_bit(PHY_INTERFACE_MODE_SGMII,
					  config.supported_interfaces));
	pcs = b53_serdes_phylink_mac_select_pcs(ctx->dev, 5,
						PHY_INTERFACE_MODE_1000BASEX);
	KUNIT_EXPECT_PTR_EQ(test, pcs, NULL);
}

static void b53_serdes_config_test(struct kunit *test)
{
	struct b53_serdes_test *ctx = test->priv;
	struct phylink_pcs *pcs = &ctx->dev->pcs[0].pcs;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(adv) = {};
	int ret;

	ctx->regs[0][TEST_DIGITAL][0] = AUTODET_EN | SGMII_MASTER_MODE |
					    SIGNAL_DETECT_EN;
	ctx->regs[0][TEST_MII][MII_BMCR] = BMCR_ISOLATE;
	linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, adv);
	ret = pcs->ops->pcs_config(pcs, PHYLINK_PCS_NEG_INBAND_ENABLED,
				  PHY_INTERFACE_MODE_1000BASEX, adv, true);
	KUNIT_EXPECT_GT(test, ret, 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_DIGITAL][0],
			(u16)(FIBER_MODE_1000X | SIGNAL_DETECT_EN));
	KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_MII][MII_ADVERTISE],
			(u16)(ADVERTISE_1000XFULL | ADVERTISE_1000XPAUSE));
	KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_MII][MII_BMCR], (u16)BMCR_ANENABLE);
	ret = pcs->ops->pcs_config(pcs, PHYLINK_PCS_NEG_INBAND_ENABLED,
					  PHY_INTERFACE_MODE_1000BASEX, adv, true);
	KUNIT_EXPECT_EQ(test, ret, 0);
	linkmode_clear_bit(ETHTOOL_LINK_MODE_Pause_BIT, adv);
	ret = pcs->ops->pcs_config(pcs, PHYLINK_PCS_NEG_INBAND_ENABLED,
					  PHY_INTERFACE_MODE_1000BASEX, adv, true);
	KUNIT_EXPECT_GT(test, ret, 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_MII][MII_ADVERTISE],
			(u16)ADVERTISE_1000XFULL);
	ret = pcs->ops->pcs_config(pcs, PHYLINK_PCS_NEG_OUTBAND,
					  PHY_INTERFACE_MODE_SGMII, adv, false);
	KUNIT_EXPECT_GE(test, ret, 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_MII][MII_BMCR], (u16)0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_MII][MII_ADVERTISE], (u16)1);
	ret = pcs->ops->pcs_config(pcs, PHYLINK_PCS_NEG_INBAND_DISABLED,
					  PHY_INTERFACE_MODE_2500BASEX, adv, false);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ctx->regs[0][TEST_MII][MII_BMCR] = BMCR_SPEED100;
	ret = pcs->ops->pcs_config(pcs, PHYLINK_PCS_NEG_INBAND_DISABLED,
				  PHY_INTERFACE_MODE_1000BASEX, adv, false);
	KUNIT_EXPECT_GT(test, ret, 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_MII][MII_BMCR],
			(u16)(BMCR_SPEED1000 | BMCR_FULLDPLX));
	ret = pcs->ops->pcs_config(pcs, PHYLINK_PCS_NEG_INBAND_DISABLED,
				  PHY_INTERFACE_MODE_1000BASEX, adv, false);
	KUNIT_EXPECT_EQ(test, ret, 0);
}

static void b53_serdes_state_test(struct kunit *test)
{
	struct b53_serdes_test *ctx = test->priv;
	struct phylink_pcs *pcs = &ctx->dev->pcs[0].pcs;
	struct phylink_link_state state = {
		.interface = PHY_INTERFACE_MODE_1000BASEX,
	};

	linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseX_Full_BIT, state.advertising);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, state.advertising);
	ctx->regs[0][TEST_MII][MII_BMSR] = BMSR_LSTATUS | BMSR_ANEGCOMPLETE;
	ctx->regs[0][TEST_MII][MII_LPA] = LPA_1000XFULL | LPA_1000XPAUSE;
	pcs->ops->pcs_get_state(pcs, PHYLINK_PCS_NEG_INBAND_ENABLED, &state);
	KUNIT_EXPECT_TRUE(test, state.link);
	KUNIT_EXPECT_TRUE(test, state.an_complete);
	KUNIT_EXPECT_EQ(test, state.speed, SPEED_1000);
	KUNIT_EXPECT_EQ(test, state.duplex, DUPLEX_FULL);
	KUNIT_EXPECT_EQ(test, state.pause, MLO_PAUSE_RX | MLO_PAUSE_TX);
	KUNIT_EXPECT_TRUE(test, linkmode_test_bit(ETHTOOL_LINK_MODE_Pause_BIT,
						  state.lp_advertising));

	ctx->regs[0][TEST_MII][MII_LPA] = LPA_1000XFULL;
	/* phylink clears pause before each in-band state read. */
	state.pause = MLO_PAUSE_NONE;
	pcs->ops->pcs_get_state(pcs, PHYLINK_PCS_NEG_INBAND_ENABLED, &state);
	KUNIT_EXPECT_EQ(test, state.pause, 0);
	state.interface = PHY_INTERFACE_MODE_SGMII;
	ctx->regs[0][TEST_MII][MII_LPA] = LPA_SGMII_LINK | LPA_SGMII_100;
	pcs->ops->pcs_get_state(pcs, PHYLINK_PCS_NEG_INBAND_ENABLED, &state);
	KUNIT_EXPECT_TRUE(test, state.link);
	KUNIT_EXPECT_EQ(test, state.speed, SPEED_100);
	KUNIT_EXPECT_EQ(test, state.duplex, DUPLEX_HALF);
	ctx->regs[0][TEST_MII][MII_LPA] = 0;
	pcs->ops->pcs_get_state(pcs, PHYLINK_PCS_NEG_INBAND_ENABLED, &state);
	KUNIT_EXPECT_FALSE(test, state.link);
	ctx->fail_at = ctx->accesses + 1;
	state.link = true;
	state.an_complete = true;
	pcs->ops->pcs_get_state(pcs, PHYLINK_PCS_NEG_INBAND_ENABLED, &state);
	KUNIT_EXPECT_FALSE(test, state.link);
	KUNIT_EXPECT_FALSE(test, state.an_complete);
}

static void b53_serdes_power_mac_test(struct kunit *test)
{
	struct b53_serdes_test *ctx = test->priv;
	struct phylink_pcs *pcs = &ctx->dev->pcs[0].pcs;
	struct dsa_port dp = { .ds = ctx->dev->ds, .index = 5 };
	const struct phylink_mac_ops *ops = ctx->dev->ds->phylink_mac_ops;
	u8 *mac = &ctx->mac[B53_GMII_PORT_OVERRIDE_CTRL(5)];

	ctx->regs[0][TEST_MII][MII_BMCR] = BMCR_PDOWN;
	KUNIT_ASSERT_EQ(test, pcs->ops->pcs_enable(pcs), 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_MII][MII_BMCR], (u16)0);
	ops->mac_link_up(&dp.pl_config, NULL, MLO_AN_INBAND,
			PHY_INTERFACE_MODE_1000BASEX, SPEED_1000, DUPLEX_FULL,
			false, true);
	KUNIT_EXPECT_TRUE(test, *mac & PORT_OVERRIDE_LINK);
	KUNIT_EXPECT_TRUE(test, *mac & PORT_OVERRIDE_RX_FLOW);
	KUNIT_EXPECT_FALSE(test, *mac & PORT_OVERRIDE_TX_FLOW);
	ops->mac_link_down(&dp.pl_config, MLO_AN_INBAND, PHY_INTERFACE_MODE_1000BASEX);
	KUNIT_EXPECT_FALSE(test, *mac & PORT_OVERRIDE_LINK);
	KUNIT_EXPECT_FALSE(test, ctx->regs[0][TEST_MII][MII_BMCR] & BMCR_PDOWN);
	pcs->ops->pcs_disable(pcs);
	KUNIT_EXPECT_TRUE(test, ctx->regs[0][TEST_MII][MII_BMCR] & BMCR_PDOWN);
	KUNIT_EXPECT_TRUE(test, pcs->poll);
}

static void b53_serdes_mac_override_test(struct kunit *test)
{
	struct b53_serdes_test *ctx = test->priv;
	struct dsa_port dp = { .ds = ctx->dev->ds, .index = 5 };
	const struct phylink_mac_ops *ops = ctx->dev->ds->phylink_mac_ops;
	u8 *mac = &ctx->mac[B53_GMII_PORT_OVERRIDE_CTRL(5)];

	/* Clear a stale 2 Gbps selection without dropping override enable. */
	*mac = GMII_PO_SPEED_2000M;
	ops->mac_link_up(&dp.pl_config, NULL, MLO_AN_INBAND,
			PHY_INTERFACE_MODE_SGMII, SPEED_1000, DUPLEX_FULL,
			false, false);
	KUNIT_EXPECT_TRUE(test, *mac & GMII_PO_EN);
	KUNIT_EXPECT_FALSE(test, *mac & GMII_PO_SPEED_2000M);

	*mac = 0;
	ctx->mac_writes = 0;
	ctx->fail_mac_read = true;
	ops->mac_link_up(&dp.pl_config, NULL, MLO_AN_INBAND,
			PHY_INTERFACE_MODE_SGMII, SPEED_1000, DUPLEX_FULL,
			false, false);
	KUNIT_EXPECT_EQ(test, ctx->mac_writes, 0U);
	KUNIT_EXPECT_EQ(test, *mac, (u8)0);

	ctx->fail_mac_read = false;
	ctx->fail_mac_write = true;
	ops->mac_link_up(&dp.pl_config, NULL, MLO_AN_INBAND,
			PHY_INTERFACE_MODE_SGMII, SPEED_1000, DUPLEX_FULL,
			false, false);
	/* A failed configuration must not be followed by forcing link up. */
	KUNIT_EXPECT_EQ(test, ctx->mac_writes, 1U);
	KUNIT_EXPECT_EQ(test, *mac, (u8)0);

	ctx->fail_mac_write = false;
	ctx->mac_writes = 0;
	ops->mac_link_up(&dp.pl_config, NULL, MLO_AN_INBAND,
			PHY_INTERFACE_MODE_SGMII, SPEED_UNKNOWN, DUPLEX_FULL,
			false, false);
	KUNIT_EXPECT_EQ(test, ctx->mac_writes, 0U);
	KUNIT_EXPECT_EQ(test, *mac, (u8)0);
}

static void b53_serdes_outband_irq_test(struct kunit *test)
{
	struct b53_serdes_test *ctx = test->priv;
	unsigned int accesses = ctx->accesses;

	ctx->dev->pcs[0].neg_mode = PHYLINK_PCS_NEG_OUTBAND;
	b53_serdes_link_change(ctx->dev, 5);
	KUNIT_EXPECT_EQ(test, ctx->accesses, accesses);
}

static void b53_serdes_forced_test(struct kunit *test)
{
	struct b53_serdes_test *ctx = test->priv;
	struct phylink_pcs *pcs = &ctx->dev->pcs[0].pcs;
	static const int speeds[] = { SPEED_10, SPEED_100, SPEED_1000 };
	int i;

	for (i = 0; i < ARRAY_SIZE(speeds); i++) {
		ctx->regs[0][TEST_DIGITAL][8] = FORCE_SPEED_MASK;
		pcs->ops->pcs_link_up(pcs, PHYLINK_PCS_NEG_OUTBAND,
				      PHY_INTERFACE_MODE_SGMII, speeds[i], DUPLEX_FULL);
		KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_MII][MII_BMCR],
				mii_bmcr_encode_fixed(speeds[i], DUPLEX_FULL));
		KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_DIGITAL][8], (u16)0);
		KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_RX][1], (u16)0);
		KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_DIGITAL][2], (u16)0);
	}
	ctx->accesses = 0;
	ctx->dev->serdes_lane = B53_INVALID_LANE;
	pcs->ops->pcs_link_up(pcs, PHYLINK_PCS_NEG_OUTBAND,
			      PHY_INTERFACE_MODE_SGMII, SPEED_100, DUPLEX_HALF);
	/* Exercise each failure point, including reset-release accesses. */
	for (i = ctx->accesses; i > 0; i--) {
		ctx->accesses = 0;
		ctx->fail_at = i;
		ctx->dev->serdes_lane = B53_INVALID_LANE;
		pcs->ops->pcs_link_up(pcs, PHYLINK_PCS_NEG_OUTBAND,
				      PHY_INTERFACE_MODE_SGMII, SPEED_100, DUPLEX_HALF);
		ctx->fail_at = 0;
		/* A later link-up must recover from any interrupted sequence. */
		pcs->ops->pcs_link_up(pcs, PHYLINK_PCS_NEG_OUTBAND,
				      PHY_INTERFACE_MODE_SGMII, SPEED_1000, DUPLEX_FULL);
		KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_RX][1], (u16)0);
		KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_DIGITAL][2], (u16)0);
	}
}

static void b53_serdes_error_test(struct kunit *test)
{
	struct b53_serdes_test *ctx = test->priv;
	struct phylink_pcs *pcs = &ctx->dev->pcs[0].pcs;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(adv) = {};
	int i, count, ret;

	ctx->accesses = 0;
	ctx->dev->serdes_lane = B53_INVALID_LANE;
	KUNIT_ASSERT_EQ(test, b53_serdes_init(ctx->dev, 5), 0);
	count = ctx->accesses;
	for (i = 1; i <= count; i++) {
		ctx->accesses = 0;
		ctx->fail_at = i;
		ctx->dev->serdes_lane = B53_INVALID_LANE;
		KUNIT_EXPECT_EQ(test, b53_serdes_init(ctx->dev, 5), -EIO);
		KUNIT_EXPECT_EQ(test, ctx->dev->serdes_lane, B53_INVALID_LANE);
	}

	ctx->fail_at = 0;
	ctx->accesses = 0;
	ctx->dev->serdes_lane = B53_INVALID_LANE;
	ret = pcs->ops->pcs_config(pcs, PHYLINK_PCS_NEG_INBAND_ENABLED,
					  PHY_INTERFACE_MODE_1000BASEX, adv, true);
	KUNIT_ASSERT_GE(test, ret, 0);
	count = ctx->accesses;
	for (i = 1; i <= count; i++) {
		memset(ctx->regs[0][TEST_MII], 0, sizeof(ctx->regs[0][TEST_MII]));
		memset(ctx->regs[0][TEST_DIGITAL], 0, sizeof(ctx->regs[0][TEST_DIGITAL]));
		ctx->accesses = 0;
		ctx->fail_at = i;
		ctx->dev->serdes_lane = B53_INVALID_LANE;
		ret = pcs->ops->pcs_config(pcs, PHYLINK_PCS_NEG_INBAND_ENABLED,
						  PHY_INTERFACE_MODE_1000BASEX, adv, true);
		KUNIT_EXPECT_EQ(test, ret, -EIO);
		KUNIT_EXPECT_EQ(test, ctx->dev->serdes_lane, B53_INVALID_LANE);
	}
}

struct b53_test_worker {
	struct phylink_pcs *pcs;
	struct completion done;
	bool pause;
	int ret;
};

static int b53_test_worker(void *arg)
{
	struct b53_test_worker *worker = arg;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(adv) = {};
	int i;

	if (worker->pause)
		linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, adv);
	for (i = 0; i < 20; i++) {
		worker->ret = worker->pcs->ops->pcs_config(worker->pcs,
				PHYLINK_PCS_NEG_INBAND_ENABLED,
				PHY_INTERFACE_MODE_1000BASEX, adv, true);
		if (worker->ret < 0)
			break;
	}
	complete(&worker->done);
	return 0;
}

static void b53_serdes_lanes_test(struct kunit *test)
{
	struct b53_serdes_test *ctx = test->priv;
	struct b53_test_worker workers[B53_N_PCS] = {};
	struct task_struct *tasks[B53_N_PCS];
	int i;

	ctx->slow = true;
	for (i = 0; i < B53_N_PCS; i++) {
		workers[i].pcs = &ctx->dev->pcs[i].pcs;
		workers[i].pause = i;
		init_completion(&workers[i].done);
		tasks[i] = kthread_run(b53_test_worker, &workers[i], "b53-test-%d", i);
		if (IS_ERR(tasks[i])) {
			KUNIT_FAIL(test, "cannot start lane worker %d", i);
			break;
		}
	}
	while (i--) {
		wait_for_completion(&workers[i].done);
		KUNIT_EXPECT_GE(test, workers[i].ret, 0);
	}
	KUNIT_EXPECT_EQ(test, ctx->regs[0][TEST_MII][MII_ADVERTISE],
			(u16)ADVERTISE_1000XFULL);
	KUNIT_EXPECT_EQ(test, ctx->regs[1][TEST_MII][MII_ADVERTISE],
			(u16)(ADVERTISE_1000XFULL | ADVERTISE_1000XPAUSE));
}

static struct kunit_case b53_serdes_test_cases[] = {
	KUNIT_CASE(b53_serdes_discovery_test),
	KUNIT_CASE(b53_serdes_config_test),
	KUNIT_CASE(b53_serdes_state_test),
	KUNIT_CASE(b53_serdes_power_mac_test),
	KUNIT_CASE(b53_serdes_mac_override_test),
	KUNIT_CASE(b53_serdes_outband_irq_test),
	KUNIT_CASE(b53_serdes_forced_test),
	KUNIT_CASE(b53_serdes_error_test),
	KUNIT_CASE(b53_serdes_lanes_test),
	{}
};

static struct kunit_suite b53_serdes_test_suite = {
	.name = "b53-serdes",
	.init = b53_serdes_test_init,
	.test_cases = b53_serdes_test_cases,
};

kunit_test_suite(b53_serdes_test_suite);

MODULE_LICENSE("Dual BSD/GPL");
