#include <linux/bug.h>
#include <linux/export.h>
#include <linux/types.h>
#include <linux/mmdebug.h>
#include <linux/mm.h>

#include <asm/sections.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/dma.h>

static inline bool __debug_virt_addr_valid(unsigned long x)
{
	if (x >= PAGE_OFFSET && x < (unsigned long)high_memory)
		return true;

	return false;
}

phys_addr_t __virt_to_phys(volatile const void *x)
{
	WARN(!__debug_virt_addr_valid((unsigned long)x),
	     "virt_to_phys used for non-linear address: %pK (%pS)\n",
	     x, x);

	return __virt_to_phys_nodebug(x);
}
EXPORT_SYMBOL(__virt_to_phys);

phys_addr_t __phys_addr_symbol(unsigned long x)
{
	/* This is bounds checking against the kernel image only.
	 * __pa_symbol should only be used on kernel symbol addresses.
	 */
	VIRTUAL_BUG_ON(x < (unsigned long)_text ||
		       x > (unsigned long)_end);

	return __pa_symbol_nodebug(x);
}
EXPORT_SYMBOL(__phys_addr_symbol);
