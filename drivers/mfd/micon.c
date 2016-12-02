/*
 * Buffalo Kurobox/Terastation Pro II specific power off method via
 * UART1-attached microcontroller
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/platform_data/micon.h>
#include <linux/serial_reg.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

struct micon_priv {
	struct device *dev;
	void __iomem *base;
	int tclk;
	struct notifier_block nb;
};

#define UART1_REG(x)	((UART_##x) << 2)

static int miconread(struct micon_priv *priv, unsigned char *buf, int count)
{
	int i;
	int timeout;

	for (i = 0; i < count; i++) {
		timeout = 10;

		while (!(readl(priv->base + UART1_REG(LSR)) & UART_LSR_DR)) {
			if (--timeout == 0)
				break;
			udelay(1000);
		}

		if (timeout == 0)
			break;
		buf[i] = readl(priv->base + UART1_REG(RX));
	}

	/* return read bytes */
	return i;
}

static int miconwrite(struct micon_priv *priv, const unsigned char *buf,
			   int count)
{
	int i = 0;

	while (count--) {
		while (!(readl(priv->base + UART1_REG(LSR)) & UART_LSR_THRE))
			barrier();
		writel(buf[i++], priv->base + UART1_REG(TX));
	}

	return 0;
}

static int miconsend(struct micon_priv *priv, const unsigned char *data,
			  int count)
{
	int i;
	unsigned char checksum = 0;
	unsigned char recv_buf[40];
	unsigned char send_buf[40];
	unsigned char correct_ack[3];
	int retry = 2;

	/* Generate checksum */
	for (i = 0; i < count; i++)
		checksum -=  data[i];

	do {
		/* Send data */
		miconwrite(priv, data, count);

		/* send checksum */
		miconwrite(priv, &checksum, 1);

		if (miconread(priv, recv_buf, sizeof(recv_buf)) <= 3) {
			dev_err(priv->dev, ">%s: receive failed.\n", __func__);

			/* send preamble to clear the receive buffer */
			memset(&send_buf, 0xff, sizeof(send_buf));
			miconwrite(priv, send_buf, sizeof(send_buf));

			/* make dummy reads */
			mdelay(100);
			miconread(priv, recv_buf, sizeof(recv_buf));
		} else {
			/* Generate expected ack */
			correct_ack[0] = 0x01;
			correct_ack[1] = data[1];
			correct_ack[2] = 0x00;

			/* checksum Check */
			if ((recv_buf[0] + recv_buf[1] + recv_buf[2] +
			     recv_buf[3]) & 0xFF) {
				dev_err(priv->dev, ">%s: Checksum Error : "
					"Received data[%02x, %02x, %02x, %02x]"
					"\n", __func__, recv_buf[0],
					recv_buf[1], recv_buf[2], recv_buf[3]);
			} else {
				/* Check Received Data */
				if (correct_ack[0] == recv_buf[0] &&
				    correct_ack[1] == recv_buf[1] &&
				    correct_ack[2] == recv_buf[2]) {
					/* Interval for next command */
					mdelay(10);

					/* Receive ACK */
					return 0;
				}
			}
			/* Received NAK or illegal Data */
			dev_err(priv->dev, ">%s: Error : NAK or Illegal Data "
					    "Received\n", __func__);
		}
	} while (retry--);

	/* Interval for next command */
	mdelay(10);

	return -1;
}

static int restart_handler(struct notifier_block *this,
			   unsigned long code, void *cmd)
{
	struct micon_priv *priv = container_of(this, struct micon_priv, nb);

	const unsigned char watchdogkill[]	= {0x01, 0x35, 0x00};
	const unsigned char shutdownwait[]	= {0x00, 0x0c};
	const unsigned char poweroff[]		= {0x00, 0x06};
	/* 38400 baud divisor */
	unsigned int divisor = ((priv->tclk + (8 * 38400)) / (16 * 38400));

	if (code != SYS_DOWN && code != SYS_HALT)
		return NOTIFY_DONE;

	dev_info(priv->dev, "%s: triggering power-off...\n", __func__);

	/* hijack uart1 and reset into sane state (38400,8n1,even parity) */
	writel(0x83, priv->base + UART1_REG(LCR));
	writel(divisor & 0xff, priv->base + UART1_REG(DLL));
	writel((divisor >> 8) & 0xff, priv->base + UART1_REG(DLM));
	writel(0x1b, priv->base + UART1_REG(LCR));
	writel(0x00, priv->base + UART1_REG(IER));
	writel(0x07, priv->base + UART1_REG(FCR));
	writel(0x00, priv->base + UART1_REG(MCR));

	/* Send the commands to shutdown the Terastation Pro II */
	miconsend(priv, watchdogkill, sizeof(watchdogkill));
	miconsend(priv, shutdownwait, sizeof(shutdownwait));
	miconsend(priv, poweroff, sizeof(poweroff));

	return NOTIFY_DONE;
}

static int micon_probe(struct platform_device *pdev)
{
	struct micon_platform_data *pdata = pdev->dev.platform_data;
	struct micon_priv *priv;
	struct resource *r;
	int ret;

	if (!pdata)
		return -ENODEV;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/* Don't request exclusive access since 8250 may also utilize this
	 * resource
	 */
	priv->base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (IS_ERR(priv->base))
		return -ENOMEM;

	priv->tclk = pdata->tclk;

	priv->nb.notifier_call = restart_handler;

	return register_reboot_notifier(&priv->nb);
}

static struct platform_driver micon_driver = {
	.probe	= micon_probe,
	.driver = {
		.name = MICON_NAME,
	},
};
builtin_platform_driver(micon_driver);
