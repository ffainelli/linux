/*
 * Buffalo Terastation Pro II/Live Board Setup
 *
 * Maintainer: Sylver Bruneau <sylver.bruneau@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/mtd/physmap.h>
#include <linux/mv643xx_eth.h>
#include <linux/i2c.h>
#include <linux/serial_reg.h>
#include <linux/platform_data/micon.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/pci.h>
#include "common.h"
#include "mpp.h"
#include "orion5x.h"

/*****************************************************************************
 * Terastation Pro 2/Live Info
 ****************************************************************************/

/*
 * Terastation Pro 2 hardware :
 * - Marvell 88F5281-D0
 * - Marvell 88SX6042 SATA controller (PCI)
 * - Marvell 88E1118 Gigabit Ethernet PHY
 * - 256KB NOR flash
 * - 128MB of DDR RAM
 * - PCIe port (not equipped)
 */

/*
 * 256K NOR flash Device bus boot chip select
 */

#define TSP2_NOR_BOOT_BASE	0xf4000000
#define TSP2_NOR_BOOT_SIZE	SZ_256K

/*****************************************************************************
 * 256KB NOR Flash on BOOT Device
 ****************************************************************************/

static struct physmap_flash_data tsp2_nor_flash_data = {
	.width    = 1,
};

static struct resource tsp2_nor_flash_resource = {
	.flags = IORESOURCE_MEM,
	.start = TSP2_NOR_BOOT_BASE,
	.end   = TSP2_NOR_BOOT_BASE + TSP2_NOR_BOOT_SIZE - 1,
};

static struct platform_device tsp2_nor_flash = {
	.name          = "physmap-flash",
	.id            = 0,
	.dev           = {
		.platform_data	= &tsp2_nor_flash_data,
	},
	.num_resources = 1,
	.resource      = &tsp2_nor_flash_resource,
};

/*****************************************************************************
 * PCI
 ****************************************************************************/
#define TSP2_PCI_SLOT0_OFFS		7
#define TSP2_PCI_SLOT0_IRQ_PIN		11

static void __init tsp2_pci_preinit(void)
{
	int pin;

	/*
	 * Configure PCI GPIO IRQ pins
	 */
	pin = TSP2_PCI_SLOT0_IRQ_PIN;
	if (gpio_request(pin, "PCI Int1") == 0) {
		if (gpio_direction_input(pin) == 0) {
			irq_set_irq_type(gpio_to_irq(pin), IRQ_TYPE_LEVEL_LOW);
		} else {
			printk(KERN_ERR "tsp2_pci_preinit failed "
					"to set_irq_type pin %d\n", pin);
			gpio_free(pin);
		}
	} else {
		printk(KERN_ERR "tsp2_pci_preinit failed to "
				"gpio_request %d\n", pin);
	}
}

static int __init tsp2_pci_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq;

	/*
	 * Check for devices with hard-wired IRQs.
	 */
	irq = orion5x_pci_map_irq(dev, slot, pin);
	if (irq != -1)
		return irq;

	/*
	 * PCI IRQs are connected via GPIOs.
	 */
	if (slot == TSP2_PCI_SLOT0_OFFS)
		return gpio_to_irq(TSP2_PCI_SLOT0_IRQ_PIN);

	return -1;
}

static struct hw_pci tsp2_pci __initdata = {
	.nr_controllers = 2,
	.preinit        = tsp2_pci_preinit,
	.setup          = orion5x_pci_sys_setup,
	.scan           = orion5x_pci_sys_scan_bus,
	.map_irq        = tsp2_pci_map_irq,
};

static int __init tsp2_pci_init(void)
{
	if (machine_is_terastation_pro2())
		pci_common_init(&tsp2_pci);

	return 0;
}

subsys_initcall(tsp2_pci_init);

/*****************************************************************************
 * Ethernet
 ****************************************************************************/

static struct mv643xx_eth_platform_data tsp2_eth_data = {
	.phy_addr	= 0,
};

/*****************************************************************************
 * RTC 5C372a on I2C bus
 ****************************************************************************/

#define TSP2_RTC_GPIO	9

static struct i2c_board_info __initdata tsp2_i2c_rtc = {
	I2C_BOARD_INFO("rs5c372a", 0x32),
};

/*****************************************************************************
 * Terastation Pro II specific power off method via UART1-attached
 * microcontroller
 ****************************************************************************/
struct resource tsp2_micon_res = {
	.start	= UART1_PHYS_BASE,
	.end	= UART1_PHYS_BASE + 0x100,
	.flags	= IORESOURCE_MEM,
};

static struct micon_platform_data tsp2_micon_pdata;

static struct platform_device tsp2_micon = {
	.name		= MICON_NAME,
	.id		= -1,
	.dev = {
		.platform_data = &tsp2_micon_pdata,
	},
	.resource	= &tsp2_micon_res,
	.num_resources	= 1,
};

/*****************************************************************************
 * General Setup
 ****************************************************************************/
static unsigned int tsp2_mpp_modes[] __initdata = {
	MPP0_PCIE_RST_OUTn,
	MPP1_UNUSED,
	MPP2_UNUSED,
	MPP3_UNUSED,
	MPP4_NAND,		/* BOOT NAND Flash REn */
	MPP5_NAND,		/* BOOT NAND Flash WEn */
	MPP6_NAND,		/* BOOT NAND Flash HREn[0] */
	MPP7_NAND,		/* BOOT NAND Flash WEn[0] */
	MPP8_GPIO,		/* MICON int */
	MPP9_GPIO,		/* RTC int */
	MPP10_UNUSED,
	MPP11_GPIO,		/* PCI Int A */
	MPP12_UNUSED,
	MPP13_GPIO,		/* UPS on UART0 enable */
	MPP14_GPIO,		/* UPS low battery detection */
	MPP15_UNUSED,
	MPP16_UART,		/* UART1 RXD */
	MPP17_UART,		/* UART1 TXD */
	MPP18_UART,		/* UART1 CTSn */
	MPP19_UART,		/* UART1 RTSn */
	0,
};

static void __init tsp2_init(void)
{
	/*
	 * Setup basic Orion functions. Need to be called early.
	 */
	orion5x_init();

	orion5x_mpp_conf(tsp2_mpp_modes);

	/*
	 * Configure peripherals.
	 */
	mvebu_mbus_add_window_by_id(ORION_MBUS_DEVBUS_BOOT_TARGET,
				    ORION_MBUS_DEVBUS_BOOT_ATTR,
				    TSP2_NOR_BOOT_BASE,
				    TSP2_NOR_BOOT_SIZE);
	platform_device_register(&tsp2_nor_flash);
	tsp2_micon_pdata.tclk = orion5x_tclk;
	platform_device_register(&tsp2_micon);

	orion5x_ehci0_init();
	orion5x_eth_init(&tsp2_eth_data);
	orion5x_i2c_init();
	orion5x_uart0_init();
	orion5x_uart1_init();

	/* Get RTC IRQ and register the chip */
	if (gpio_request(TSP2_RTC_GPIO, "rtc") == 0) {
		if (gpio_direction_input(TSP2_RTC_GPIO) == 0)
			tsp2_i2c_rtc.irq = gpio_to_irq(TSP2_RTC_GPIO);
		else
			gpio_free(TSP2_RTC_GPIO);
	}
	if (tsp2_i2c_rtc.irq == 0)
		pr_warn("tsp2_init: failed to get RTC IRQ\n");
	i2c_register_board_info(0, &tsp2_i2c_rtc, 1);
}

MACHINE_START(TERASTATION_PRO2, "Buffalo Terastation Pro II/Live")
	/* Maintainer:  Sylver Bruneau <sylver.bruneau@googlemail.com> */
	.atag_offset	= 0x100,
	.nr_irqs	= ORION5X_NR_IRQS,
	.init_machine	= tsp2_init,
	.map_io		= orion5x_map_io,
	.init_early	= orion5x_init_early,
	.init_irq	= orion5x_init_irq,
	.init_time	= orion5x_timer_init,
	.fixup		= tag_fixup_mem32,
	.restart	= orion5x_restart,
MACHINE_END
