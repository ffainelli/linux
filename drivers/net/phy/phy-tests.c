// SPDX-License-Identifier: GPL-2.0
/* PHY library common test modes
 */
#include <linux/export.h>
#include <linux/netdevice.h>
#include <linux/phy.h>

/* genphy_get_test - Get PHY test specific data
 * @phydev: the PHY device instance
 * @test: the desired test mode
 * @data: test specific data (none)
 */
int genphy_get_test(struct phy_device *phydev, struct ethtool_phy_test *test,
		    u8 *data)
{
	if (test->mode >= PHY_STD_TEST_MODE_MAX)
		return -EOPNOTSUPP;

	return 0;
}
EXPORT_SYMBOL_GPL(genphy_get_test);

/* genphy_set_test - Make a PHY enter one of the standard IEEE defined
 * test modes
 * @phydev: the PHY device instance
 * @test: the desired test mode
 * @data: test specific data (none)
 *
 * This function makes the designated @phydev enter the desired standard
 * 100BaseT2 or 1000BaseT test mode as defined in IEEE 802.3-2012 section TWO
 * and THREE under 32.6.1.2.1 and 40.6.1.1.2 respectively
 */
int genphy_set_test(struct phy_device *phydev,
		    struct ethtool_phy_test *test, const u8 *data)
{
	u16 shift, base, bmcr = 0;
	int ret;

	switch (test->mode) {
	case PHY_STD_TEST_MODE_100BASET2_1:
	case PHY_STD_TEST_MODE_100BASET2_2:
	case PHY_STD_TEST_MODE_100BASET2_3:
		if (!(phydev->supported & PHY_100BT_FEATURES))
			return -EOPNOTSUPP;

		shift = 14;
		base = test->mode - PHY_STD_TEST_MODE_BASE;
		bmcr = BMCR_SPEED100;
		break;

	case PHY_STD_TEST_MODE_1000BASET_1:
	case PHY_STD_TEST_MODE_1000BASET_2:
	case PHY_STD_TEST_MODE_1000BASET_3:
	case PHY_STD_TEST_MODE_1000BASET_4:
		if (!(phydev->supported & PHY_1000BT_FEATURES))
			return -EOPNOTSUPP;

		shift = 13;
		base = test->mode - PHY_STD_TEST_MODE_100BASET2_MAX;
		bmcr = BMCR_SPEED1000;
		break;

	default:
		/* Let an upper driver deal with additional modes it may
		 * support
		 */
		return -EOPNOTSUPP;
	}

	/* Force speed and duplex */
	ret = phy_write(phydev, MII_BMCR, bmcr | BMCR_FULLDPLX);
	if (ret < 0)
		return ret;

	/* Set the desired test mode bit */
	return phy_write(phydev, MII_CTRL1000, (test->mode + base) << shift);
}
EXPORT_SYMBOL_GPL(genphy_set_test);

static const char *const phy_std_test_mode_str[] = {
	"100baseT2-tx-waveform",
	"100baseT2-tx-jitter",
	"100baseT2-tx-idle",
	"1000baseT-tx-waveform",
	"1000baseT-tx-jitter-master",
	"1000baseT-tx-jitter-slave",
	"1000BaseT-tx-distorsion"
};

/* genphy_get_test_count - Get PHY test count
 * @phydev: the PHY device instance
 *
 * Returns the number of supported test modes for this PHY
 */
int genphy_get_test_count(struct phy_device *phydev)
{
	return ARRAY_SIZE(phy_std_test_mode_str);
}
EXPORT_SYMBOL_GPL(genphy_get_test_count);

/* genphy_get_test_len - Return the amount of test specific data given
 * a specific test mode
 * @phydev: the PHY device instance
 * @mode: the desired test mode
 */
int genphy_get_test_len(struct phy_device *phydev,
			struct ethtool_phy_test *test)
{
	switch (test->mode) {
	case PHY_STD_TEST_MODE_100BASET2_1:
	case PHY_STD_TEST_MODE_100BASET2_2:
	case PHY_STD_TEST_MODE_100BASET2_3:
	case PHY_STD_TEST_MODE_1000BASET_1:
	case PHY_STD_TEST_MODE_1000BASET_2:
	case PHY_STD_TEST_MODE_1000BASET_3:
	case PHY_STD_TEST_MODE_1000BASET_4:
		/* no test specific data */
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}
EXPORT_SYMBOL_GPL(genphy_get_test_len);

/* genphy_get_test_strings - Obtain the PHY device supported test modes
 * text representations
 * @phydev: the PHY device instance
 * @data: buffer to store strings
 */
void genphy_get_test_strings(struct phy_device *phydev, u8 *data)
{
	unsigned int i;

	if (!(phydev->supported & PHY_100BT_FEATURES))
		return;

	for (i = 0; i < PHY_STD_TEST_MODE_100BASET2_MAX; i++)
		strlcpy(data + i * ETH_GSTRING_LEN,
			phy_std_test_mode_str[i], ETH_GSTRING_LEN);

	if (!(phydev->supported & PHY_1000BT_FEATURES))
		return;

	for (; i < PHY_STD_TEST_MODE_MAX; i++)
		strlcpy(data + i * ETH_GSTRING_LEN,
			phy_std_test_mode_str[i], ETH_GSTRING_LEN);
}
EXPORT_SYMBOL_GPL(genphy_get_test_strings);
