/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2006  Ralf Baechle <ralf@linux-mips.org>
 */
#ifndef __ASM_MACH_JAZZ_DMA_COHERENCE_H
#define __ASM_MACH_JAZZ_DMA_COHERENCE_H

#include <asm/jazzdma.h>

struct device;

#define plat_map_dma_mem	plat_map_dma_mem
static inline dma_addr_t plat_map_dma_mem(struct device *dev, void *addr, size_t size)
{
	return vdma_alloc(virt_to_phys(addr), size);
}

#define plat_map_dma_mem_page	plat_map_dma_mem_page
static inline dma_addr_t plat_map_dma_mem_page(struct device *dev,
	struct page *page)
{
	return vdma_alloc(page_to_phys(page), PAGE_SIZE);
}

#define plat_dma_addr_to_phys	plat_dma_addr_to_phys
static inline unsigned long plat_dma_addr_to_phys(struct device *dev,
	dma_addr_t dma_addr)
{
	return vdma_log2phys(dma_addr);
}

#define plat_unmap_dma_mem	plat_unmap_dma_mem
static inline void plat_unmap_dma_mem(struct device *dev, dma_addr_t dma_addr,
	size_t size, enum dma_data_direction direction)
{
	vdma_free(dma_addr);
}

#define plat_device_is_coherent	plat_device_is_coherent
static inline int plat_device_is_coherent(struct device *dev)
{
	return 0;
}

#include <asm/mach-generic/dma-coherence.h>

#endif /* __ASM_MACH_JAZZ_DMA_COHERENCE_H */
