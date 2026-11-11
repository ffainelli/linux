// SPDX-License-Identifier: GPL-2.0-only
/* GPIO support for Broadcom SiByte SoCs. */

#include <linux/bitmap.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <asm/sibyte/bcm1480_scd.h>
#include <asm/sibyte/sb1250.h>
#include <asm/sibyte/sb1250_regs.h>
#include <asm/sibyte/sb1250_scd.h>

#define SIBYTE_GPIO_NGPIO	16

struct sibyte_gpio {
	struct gpio_chip chip;
	void __iomem *base;
	raw_spinlock_t lock;
};

static int sibyte_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct sibyte_gpio *gpio = gpiochip_get_data(chip);

	if (__raw_readq(gpio->base + R_GPIO_DIRECTION) & BIT(offset))
		return GPIO_LINE_DIRECTION_OUT;

	return GPIO_LINE_DIRECTION_IN;
}

static int sibyte_gpio_direction_input(struct gpio_chip *chip,
				       unsigned int offset)
{
	struct sibyte_gpio *gpio = gpiochip_get_data(chip);
	unsigned long flags;
	u64 direction;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	direction = __raw_readq(gpio->base + R_GPIO_DIRECTION);
	__raw_writeq(direction & ~BIT_ULL(offset),
		     gpio->base + R_GPIO_DIRECTION);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int sibyte_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct sibyte_gpio *gpio = gpiochip_get_data(chip);
	u64 value;

	/* The input inverter belongs to interrupt configuration, not GPIO logic. */
	value = __raw_readq(gpio->base + R_GPIO_READ);
	value ^= __raw_readq(gpio->base + R_GPIO_INPUT_INVERT);

	return !!(value & BIT(offset));
}

static int sibyte_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct sibyte_gpio *gpio = gpiochip_get_data(chip);

	__raw_writeq(BIT_ULL(offset), gpio->base +
		     (value ? R_GPIO_PIN_SET : R_GPIO_PIN_CLR));

	return 0;
}

static int sibyte_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	struct sibyte_gpio *gpio = gpiochip_get_data(chip);
	unsigned long flags;
	u64 direction;

	raw_spin_lock_irqsave(&gpio->lock, flags);
	sibyte_gpio_set(chip, offset, value);
	/* Load the output latch before enabling the output driver. */
	wmb();
	direction = __raw_readq(gpio->base + R_GPIO_DIRECTION);
	__raw_writeq(direction | BIT_ULL(offset),
		     gpio->base + R_GPIO_DIRECTION);
	raw_spin_unlock_irqrestore(&gpio->lock, flags);

	return 0;
}

static int sibyte_gpio_init_valid_mask(struct gpio_chip *chip,
				       unsigned long *valid_mask,
				       unsigned int ngpios)
{
	u64 config = __raw_readq(IOADDR(A_SCD_SYSTEM_CFG));

	/* Pin functions are selected by reset-time straps. */
#ifdef CONFIG_SIBYTE_BCM1x80
	if (config & M_BCM1480_SYS_GEN_PARITY_EN)
		bitmap_clear(valid_mask, 2, 4);
	if (config & M_BCM1480_SYS_PCMCIA_ENABLE)
		bitmap_clear(valid_mask, 6, 10);
#else
	if (config & M_SYS_SER0_RSTB_EN)
		__clear_bit(0, valid_mask);
	if (config & M_SYS_SER1_RSTB_EN)
		__clear_bit(1, valid_mask);
	if (config & M_SYS_GEN_PARITY_EN)
		bitmap_clear(valid_mask, 2, 4);
	if (config & M_SYS_PCMCIA_ENABLE)
		bitmap_clear(valid_mask, 6, 10);
#endif

	return 0;
}

static int sibyte_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sibyte_gpio *gpio;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpio->base))
		return PTR_ERR(gpio->base);

	raw_spin_lock_init(&gpio->lock);
	gpio->chip.label = "sibyte-gpio";
	gpio->chip.parent = dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.base = -1;
	gpio->chip.ngpio = SIBYTE_GPIO_NGPIO;
	gpio->chip.get_direction = sibyte_gpio_get_direction;
	gpio->chip.direction_input = sibyte_gpio_direction_input;
	gpio->chip.direction_output = sibyte_gpio_direction_output;
	gpio->chip.get = sibyte_gpio_get;
	gpio->chip.set = sibyte_gpio_set;
	gpio->chip.init_valid_mask = sibyte_gpio_init_valid_mask;

	/* CSRs use native-endian 64-bit accesses, including on 32-bit kernels. */
	return devm_gpiochip_add_data(dev, &gpio->chip, gpio);
}

static struct platform_driver sibyte_gpio_driver = {
	.probe = sibyte_gpio_probe,
	.driver = {
		.name = "sibyte-gpio",
	},
};
module_platform_driver(sibyte_gpio_driver);

MODULE_DESCRIPTION("Broadcom SiByte GPIO driver");
MODULE_AUTHOR("Florian Fainelli <florian.fainelli@broadcom.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sibyte-gpio");
