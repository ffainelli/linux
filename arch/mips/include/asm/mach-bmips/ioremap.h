/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_MACH_BMIPS_IOREMAP_H
#define __ASM_MACH_BMIPS_IOREMAP_H

#include <linux/types.h>

static inline phys_addr_t fixup_bigphys_addr(phys_addr_t phys_addr, phys_addr_t size)
{
	return phys_addr;
}

extern void __iomem *plat_ioremap(phys_addr_t offset, unsigned long size,
		        unsigned long flags);
extern int plat_iounmap(const volatile void __iomem *addr);

#endif /* __ASM_MACH_BMIPS_GENERIC_IOREMAP_H */
