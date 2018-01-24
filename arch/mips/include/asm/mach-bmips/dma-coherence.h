/*
 * Copyright (C) 2006 Ralf Baechle <ralf@linux-mips.org>
 * Copyright (C) 2009 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_MACH_BMIPS_DMA_COHERENCE_H
#define __ASM_MACH_BMIPS_DMA_COHERENCE_H

#include <asm/bmips.h>
#include <asm/cpu-type.h>
#include <asm/cpu.h>

struct device;

#define plat_map_dma_mem	plat_map_dma_mem
extern dma_addr_t plat_map_dma_mem(struct device *dev, void *addr, size_t size);
#define plat_map_dma_mem_page	plat_map_dma_mem_page
extern dma_addr_t plat_map_dma_mem_page(struct device *dev, struct page *page);
#define plat_dma_addr_to_phys	plat_dma_addr_to_phys
extern unsigned long plat_dma_addr_to_phys(struct device *dev,
	dma_addr_t dma_addr);

#define plat_device_is_coherent	plat_device_is_coherent
static inline int plat_device_is_coherent(struct device *dev)
{
	return 0;
}

#define plat_post_dma_flush	bmips_post_dma_flush

#define plat_map_coherent	plat_map_coherent
extern int plat_map_coherent(dma_addr_t handle, void *cac_va, size_t size,
			     void **uncac_va, gfp_t gfp);
#define plat_unmap_coherent	plat_unmap_coherent
extern void *plat_unmap_coherent(void *addr);

#include <asm/mach-generic/dma-coherence.h>

#endif /* __ASM_MACH_BMIPS_DMA_COHERENCE_H */
