/*
 * arch/arm/mach-orion5x/kurobox_pro-setup.c
 *
 * Maintainer: Ronen Shitrit <rshitrit@marvell.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/nand.h>
#include <linux/mv643xx_eth.h>
#include <linux/i2c.h>
#include <linux/serial_reg.h>
#include <linux/ata_platform.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/pci.h>
#include <linux/platform_data/mtd-orion_nand.h>
#include <linux/platform_data/micon.h>
#include "common.h"
#include "mpp.h"
#include "orion5x.h"

/*****************************************************************************
 * KUROBOX-PRO Info
 ****************************************************************************/

/*
 * 256K NOR flash Device bus boot chip select
 */

#define KUROBOX_PRO_NOR_BOOT_BASE	0xf4000000
#define KUROBOX_PRO_NOR_BOOT_SIZE	SZ_256K

/*
 * 256M NAND flash on Device bus chip select 1
 */

#define KUROBOX_PRO_NAND_BASE		0xfc000000
#define KUROBOX_PRO_NAND_SIZE		SZ_2M

/*****************************************************************************
 * 256MB NAND Flash on Device bus CS0
 ****************************************************************************/

static struct mtd_partition kurobox_pro_nand_parts[] = {
	{
		.name	= "uImage",
		.offset	= 0,
		.size	= SZ_4M,
	}, {
		.name	= "rootfs",
		.offset	= SZ_4M,
		.size	= SZ_64M,
	}, {
		.name	= "extra",
		.offset	= SZ_4M + SZ_64M,
		.size	= SZ_256M - (SZ_4M + SZ_64M),
	},
};

static struct resource kurobox_pro_nand_resource = {
	.flags		= IORESOURCE_MEM,
	.start		= KUROBOX_PRO_NAND_BASE,
	.end		= KUROBOX_PRO_NAND_BASE + KUROBOX_PRO_NAND_SIZE - 1,
};

static struct orion_nand_data kurobox_pro_nand_data = {
	.parts		= kurobox_pro_nand_parts,
	.nr_parts	= ARRAY_SIZE(kurobox_pro_nand_parts),
	.cle		= 0,
	.ale		= 1,
	.width		= 8,
};

static struct platform_device kurobox_pro_nand_flash = {
	.name		= "orion_nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &kurobox_pro_nand_data,
	},
	.resource	= &kurobox_pro_nand_resource,
	.num_resources	= 1,
};

/*****************************************************************************
 * 256KB NOR Flash on BOOT Device
 ****************************************************************************/

static struct physmap_flash_data kurobox_pro_nor_flash_data = {
	.width		= 1,
};

static struct resource kurobox_pro_nor_flash_resource = {
	.flags			= IORESOURCE_MEM,
	.start			= KUROBOX_PRO_NOR_BOOT_BASE,
	.end			= KUROBOX_PRO_NOR_BOOT_BASE + KUROBOX_PRO_NOR_BOOT_SIZE - 1,
};

static struct platform_device kurobox_pro_nor_flash = {
	.name			= "physmap-flash",
	.id			= 0,
	.dev		= {
		.platform_data	= &kurobox_pro_nor_flash_data,
	},
	.num_resources		= 1,
	.resource		= &kurobox_pro_nor_flash_resource,
};

/*****************************************************************************
 * PCI
 ****************************************************************************/

static int __init kurobox_pro_pci_map_irq(const struct pci_dev *dev, u8 slot,
	u8 pin)
{
	int irq;

	/*
	 * Check for devices with hard-wired IRQs.
	 */
	irq = orion5x_pci_map_irq(dev, slot, pin);
	if (irq != -1)
		return irq;

	/*
	 * PCI isn't used on the Kuro
	 */
	return -1;
}

static struct hw_pci kurobox_pro_pci __initdata = {
	.nr_controllers	= 2,
	.setup		= orion5x_pci_sys_setup,
	.scan		= orion5x_pci_sys_scan_bus,
	.map_irq	= kurobox_pro_pci_map_irq,
};

static int __init kurobox_pro_pci_init(void)
{
	if (machine_is_kurobox_pro()) {
		orion5x_pci_disable();
		pci_common_init(&kurobox_pro_pci);
	}

	return 0;
}

subsys_initcall(kurobox_pro_pci_init);

/*****************************************************************************
 * Ethernet
 ****************************************************************************/

static struct mv643xx_eth_platform_data kurobox_pro_eth_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(8),
};

/*****************************************************************************
 * RTC 5C372a on I2C bus
 ****************************************************************************/
static struct i2c_board_info __initdata kurobox_pro_i2c_rtc = {
	I2C_BOARD_INFO("rs5c372a", 0x32),
};

/*****************************************************************************
 * SATA
 ****************************************************************************/
static struct mv_sata_platform_data kurobox_pro_sata_data = {
	.n_ports	= 2,
};

/*****************************************************************************
 * Kurobox Pro specific power off method via UART1-attached microcontroller
 ****************************************************************************/
struct resource kurobox_pro_micon_res = {
	.start	= UART1_PHYS_BASE,
	.end	= UART1_PHYS_BASE + 0x100,
	.flags	= IORESOURCE_MEM,
};

static struct micon_platform_data kurobox_pro_micon_pdata;

static struct platform_device kurobox_pro_micon = {
	.name		= MICON_NAME,
	.id		= -1,
	.dev		= {
		.platform_data = &kurobox_pro_micon_pdata,
	},
	.resource 	= &kurobox_pro_micon_res,
	.num_resources	= 1,
};

/*****************************************************************************
 * General Setup
 ****************************************************************************/
static unsigned int kurobox_pro_mpp_modes[] __initdata = {
	MPP0_UNUSED,
	MPP1_UNUSED,
	MPP2_GPIO,		/* GPIO Micon */
	MPP3_GPIO,		/* GPIO Rtc */
	MPP4_UNUSED,
	MPP5_UNUSED,
	MPP6_NAND,		/* NAND Flash REn */
	MPP7_NAND,		/* NAND Flash WEn */
	MPP8_UNUSED,
	MPP9_UNUSED,
	MPP10_UNUSED,
	MPP11_UNUSED,
	MPP12_SATA_LED,		/* SATA 0 presence */
	MPP13_SATA_LED,		/* SATA 1 presence */
	MPP14_SATA_LED,		/* SATA 0 active */
	MPP15_SATA_LED,		/* SATA 1 active */
	MPP16_UART,		/* UART1 RXD */
	MPP17_UART,		/* UART1 TXD */
	MPP18_UART,		/* UART1 CTSn */
	MPP19_UART,		/* UART1 RTSn */
	0,
};

static void __init kurobox_pro_init(void)
{
	/*
	 * Setup basic Orion functions. Need to be called early.
	 */
	orion5x_init();

	orion5x_mpp_conf(kurobox_pro_mpp_modes);

	/*
	 * Configure peripherals.
	 */
	orion5x_ehci0_init();
	orion5x_ehci1_init();
	orion5x_eth_init(&kurobox_pro_eth_data);
	orion5x_i2c_init();
	orion5x_sata_init(&kurobox_pro_sata_data);
	orion5x_uart0_init();
	orion5x_uart1_init();
	orion5x_xor_init();

	mvebu_mbus_add_window_by_id(ORION_MBUS_DEVBUS_BOOT_TARGET,
				    ORION_MBUS_DEVBUS_BOOT_ATTR,
				    KUROBOX_PRO_NOR_BOOT_BASE,
				    KUROBOX_PRO_NOR_BOOT_SIZE);
	platform_device_register(&kurobox_pro_nor_flash);
	kurobox_pro_micon_pdata.tclk = orion5x_tclk;
	platform_device_register(&kurobox_pro_micon);

	if (machine_is_kurobox_pro()) {
		mvebu_mbus_add_window_by_id(ORION_MBUS_DEVBUS_TARGET(0),
					    ORION_MBUS_DEVBUS_ATTR(0),
					    KUROBOX_PRO_NAND_BASE,
					    KUROBOX_PRO_NAND_SIZE);
		platform_device_register(&kurobox_pro_nand_flash);
	}

	i2c_register_board_info(0, &kurobox_pro_i2c_rtc, 1);
}

#ifdef CONFIG_MACH_KUROBOX_PRO
MACHINE_START(KUROBOX_PRO, "Buffalo/Revogear Kurobox Pro")
	/* Maintainer: Ronen Shitrit <rshitrit@marvell.com> */
	.atag_offset	= 0x100,
	.nr_irqs	= ORION5X_NR_IRQS,
	.init_machine	= kurobox_pro_init,
	.map_io		= orion5x_map_io,
	.init_early	= orion5x_init_early,
	.init_irq	= orion5x_init_irq,
	.init_time	= orion5x_timer_init,
	.fixup		= tag_fixup_mem32,
	.restart	= orion5x_restart,
MACHINE_END
#endif

#ifdef CONFIG_MACH_LINKSTATION_PRO
MACHINE_START(LINKSTATION_PRO, "Buffalo Linkstation Pro/Live")
	/* Maintainer: Byron Bradley <byron.bbradley@gmail.com> */
	.atag_offset	= 0x100,
	.nr_irqs	= ORION5X_NR_IRQS,
	.init_machine	= kurobox_pro_init,
	.map_io		= orion5x_map_io,
	.init_early	= orion5x_init_early,
	.init_irq	= orion5x_init_irq,
	.init_time	= orion5x_timer_init,
	.fixup		= tag_fixup_mem32,
	.restart	= orion5x_restart,
MACHINE_END
#endif
