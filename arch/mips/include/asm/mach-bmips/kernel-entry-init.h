#ifndef __ASM_MACH_BMIPS_GENERIC_KERNEL_ENTRY_H
#define __ASM_MACH_BMIPS_GENERIC_KERNEL_ENTRY_H

	.macro kernel_entry_setup

	# save arguments for CFE callback
	sw      a0, cfe_handle
	sw      a2, cfe_entry
	sw      a3, cfe_seal

	jal     bmips_enable_xks01

	.endm

	.macro  smp_slave_setup
	.endm

#endif /* __ASM_MACH_BMIPS_GENERIC_KERNEL_ENTRY_H */
