/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994 - 1999, 2000, 03, 04 Ralf Baechle
 * Copyright (C) 2000, 2002  Maciej W. Rozycki
 * Copyright (C) 1990, 1999, 2000 Silicon Graphics, Inc.
 */
#ifndef _ASM_BMIPS_SPACES_H
#define _ASM_BMIPS_SPACES_H

/* Avoid collisions with system base register (SBR) region on BMIPS3300 */
#include <asm/bmips-spaces.h>

#include <linux/const.h>
#include <asm/mipsregs.h>
#include <asm/addrspace.h>
#include <asm/cpu.h>

/*
 * 1024MB Broadcom 256+768 virtual address map
 *
 * 8000_0000 - 8fff_ffff: 256MB RAM @ 0000_0000, cached
 * 9000_0000 - 9fff_ffff: 256MB EBI/Registers @ 1000_0000, uncached
 * a000_0000 - cfff_ffff: 768MB RAM @ 2000_0000, cached
 * d000_0000 - dfff_ffff: TBD
 * e000_0000 - ff1f_7fff: vmalloc region
 * ff1f_8000 - ff1f_ffff: FIXMAP
 * ff40_0000 - ff7f_ffff: CONSISTENT region
 *
 * PA 5000_0000 and above are accessed through HIGHMEM (BMIPS5000 only).
 */
#define TLB_UPPERMEM_VA         _AC(0xc0000000, UL)
#define TLB_UPPERMEM_PA         _AC(0x40000000, UL)

#ifndef __ASSEMBLY__
static inline unsigned long kseg0_size(void)
{
	switch (read_c0_prid() & PRID_IMP_MASK) {
	case PRID_IMP_BMIPS5000:
	case PRID_IMP_BMIPS5200:
		return _AC(0x40000000, UL);
	default:
		return _AC(0x20000000, UL);
	}
}

static inline unsigned long kseg1_size(void)
{
	switch (read_c0_prid() & PRID_IMP_MASK) {
	case PRID_IMP_BMIPS5000:
	case PRID_IMP_BMIPS5200:
		return _AC(0x0, UL);
	default:
		return _AC(0x20000000, UL);
	}
}

static inline unsigned long map_base(void)
{
	switch (read_c0_prid() & PRID_IMP_MASK) {
	case PRID_IMP_BMIPS5000:
	case PRID_IMP_BMIPS5200:
		return _AC(0xe0000000, UL);
	default:
		return _AC(0xc0000000, UL);
	}
}

static inline unsigned long brcm_max_upper_mb(void)
{
	switch (read_c0_prid() & PRID_IMP_MASK) {
	case PRID_IMP_BMIPS5000:
	case PRID_IMP_BMIPS5200:
		return _AC(768, UL);
	default:
		return _AC(0, UL);
	}
}

static inline unsigned long plat_kseg1(void)
{
	switch (read_c0_prid() & PRID_IMP_MASK) {
	case PRID_IMP_BMIPS5000:
	case PRID_IMP_BMIPS5200:
		return 0x80000000;
	default:
		return 0xa0000000;
	}
}

#define KSEG0_SIZE              kseg0_size()
#define KSEG1_SIZE		kseg1_size()
#define MAP_BASE		map_base()
/* BASE and END must be 4MB-aligned (PGDIR_SIZE) */
#define CONSISTENT_BASE         _AC(0xff400000, UL)
#define CONSISTENT_END          _AC(0xff800000, UL)
#define BRCM_MAX_UPPER_MB       brcm_max_upper_mb()
#else

#define TLB_UPPERMEM_VA         _AC(0xc0000000, UL)
#define TLB_UPPERMEM_PA         _AC(0x40000000, UL)
#define KSEG0_SIZE              _AC(0x40000000, UL)
#define KSEG1_SIZE              _AC(0x00000000, UL)
#define MAP_BASE                _AC(0xe0000000, UL)
/* BASE and END must be 4MB-aligned (PGDIR_SIZE) */
#define CONSISTENT_BASE         _AC(0xff400000, UL)
#define CONSISTENT_END          _AC(0xff800000, UL)
#define BRCM_MAX_UPPER_MB       _AC(768, UL)
#endif

#define BRCM_MAX_LOWER_MB	_AC(256, UL)

#define UPPERMEM_START		_AC(0x20000000, UL)
#define HIGHMEM_START		(UPPERMEM_START + (BRCM_MAX_UPPER_MB << 20))

#include <asm/mach-generic/spaces.h>

#endif /* __ASM_BMIPS_SPACES_H */
