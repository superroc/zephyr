/*
 * Copyright (c) 2025 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Linker script for the FR30xx-C
 */

#include <zephyr/devicetree.h>
#include <zephyr/linker/linker-tool.h>

MEMORY
{
	PRAM  (rwx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(pram)), LENGTH = DT_REG_SIZE(DT_NODELABEL(pram))
}

#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>

SECTIONS
{
	SECTION_DATA_PROLOGUE(ram_code_front,,)
	{
		. = ALIGN(8);
		*(ram_code_front)
		*(ram_code_front.*)
	} GROUP_DATA_LINK_IN(PRAM, ROMABLE_REGION)
	
	SECTION_DATA_PROLOGUE(ram_code,,)
	{
		. = ALIGN(8);
		*(ram_code)
		*(ram_code.*)
	} GROUP_DATA_LINK_IN(PRAM, ROMABLE_REGION)
	
	PROVIDE (__ram_code_end = .);
	PROVIDE (__ram_code_start = ADDR(ram_code_front));
	PROVIDE (__ram_code_load_start = LOADADDR(ram_code_front));
}
