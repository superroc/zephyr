/*
 * Copyright (c) 2025 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for FR30xx-C processor
 */

#include <zephyr/device.h>
#include <zephyr/init.h>

#include "soc.h"

extern char __ram_code_start[];
extern char __ram_code_end[];
extern char __ram_code_load_start[];

static void copy_ram_code(void) {
    size_t ram_code_size = __ram_code_end - __ram_code_start;
    memcpy(__ram_code_start, __ram_code_load_start, ram_code_size);
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 */
void soc_early_init_hook(void)
{
	copy_ram_code();

	SystemInit();

    void main_entry_point(void);
    main_entry_point();
    pmu_init();
}
