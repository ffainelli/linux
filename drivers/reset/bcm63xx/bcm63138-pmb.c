/*
 * Broadcom BCM63138 reset controller driver using PMB
 *
 * Copyright (C) 2015 Broadcom Corporation
 * Author: Florian Fainelli <f.fainelli@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/reset-controller.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reset/bcm63xx_pmb.h>

#define BPCM_ZONES_BASE		0x40
#define BPCM_ZONES_SIZE		0xff0
#define BPCM_ZONES_SHIFT	24

#define PMB_BUS_AIP		0
#define PMB_BUS_SATA		0
#define PMB_ADDR_AIP		(4 | PMB_BUS_AIP << PMB_BUS_ID_SHIFT)
#define PMB_ADDR_SATA		(3 | PMB_BUS_SATA << PMB_BUS_ID_SHIFT)
#define PMB_ADDR_MASK		0xff

/* Zone N control bitfields */
#define ZONE_CONTROL		0x00
#define DPG_CTL_EN		BIT(8)
#define PWR_DN_REQ		BIT(9)
#define PWR_UP_REQ		BIT(10)
#define MEM_PWR_CTL_EN		BIT(11)
#define BLK_RESET_ASSERT	BIT(12)

#define BPCM_SR_CONTROL		0x28
#define BPCM_MISC_CONTROL	0x30

struct bcm63138_reset_priv {
	void __iomem	*base;
	spinlock_t	lock;
	struct reset_controller_dev rcdev;
};

#define to_bcm63138_reset_priv(p) \
	container_of((p), struct bcm63138_reset_priv, rcdev)


static int bcm63138_pmc_power_on_sata(struct bcm63138_reset_priv *priv,
				      unsigned long addr)
{
	int ret;
	u32 ctrl;

	ret = bpcm_rd(priv->base, addr, BPCM_ZONES_BASE + ZONE_CONTROL, &ctrl);
	if (ret)
		return ret;

	ctrl &= ~PWR_DN_REQ;
	ctrl |= DPG_CTL_EN | PWR_UP_REQ | MEM_PWR_CTL_EN | BLK_RESET_ASSERT;

	ret = bpcm_wr(priv->base, addr, BPCM_ZONES_BASE + ZONE_CONTROL, ctrl);
	if (ret)
		return ret;

	ret = bpcm_wr(priv->base, addr, BPCM_MISC_CONTROL, 0);
	if (ret)
		return ret;

	ret = bpcm_wr(priv->base, addr, BPCM_SR_CONTROL, 0xffffffff);
	if (ret)
		return ret;

	return bpcm_wr(priv->base, addr, BPCM_SR_CONTROL, 0);
}

static int bcm63138_deassert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct bcm63138_reset_priv *priv = to_bcm63138_reset_priv(rcdev);
	unsigned long flags;
	int ret = 0;

	pr_info("%s: deasserting id: %ld\n", __func__, id);

	spin_lock_irqsave(&priv->lock, flags);

	switch (id & PMB_ADDR_MASK) {
	case PMB_ADDR_SATA:
		ret = bcm63138_pmc_power_on_sata(priv, id);
		break;
	default:
		pr_err("%s: unimplemented reset for id: %ld\n", __func__, id);
		ret = -EINVAL;
		break;
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static struct reset_control_ops bcm63138_reset_ops = {
	.deassert	= bcm63138_deassert,
};

static int bcm63138_of_xlate(struct reset_controller_dev *rcdev,
			     const struct of_phandle_args *reset_spec)
{
	unsigned int addr;
	unsigned int n_zones;

	addr = reset_spec->args[0];
	n_zones = reset_spec->args[1];
	if (addr > PMB_ADDR_MASK || n_zones > 255)
		return -EINVAL;

	return addr | n_zones << BPCM_ZONES_SHIFT;
}

static int bcm63138_reset_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct reset_controller_dev *rdev;
	struct bcm63138_reset_priv *priv;
	struct resource *r;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->lock);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	rdev = &priv->rcdev;
	rdev->ops = &bcm63138_reset_ops;
	rdev->owner = THIS_MODULE;
	rdev->of_node = dn;
	rdev->of_xlate = bcm63138_of_xlate;
	rdev->of_reset_n_cells = 2;

	dev_info(&pdev->dev, "BCM63138 Reset controller at 0x%p\n",
		 priv->base);

	return reset_controller_register(&priv->rcdev);
}

static struct of_device_id bcm63138_reset_match[] = {
	{ .compatible = "brcm,bcm63138-pmb", },
	{ /* sentinel */ },
};

static struct platform_driver bcm63138_reset_driver = {
	.probe	= bcm63138_reset_probe,
	.driver	= {
		.name	= "bcm63138-pmb",
		.of_match_table	= bcm63138_reset_match,
	},
};

int __init bcm63138_reset_init(void)
{
	return platform_driver_register(&bcm63138_reset_driver);
}
arch_initcall(bcm63138_reset_init);
