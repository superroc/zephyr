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
#include "controller.h"

extern char __ram_code_start[];
extern char __ram_code_end[];
extern char __ram_code_load_start[];

static void copy_ram_code(void) {
    size_t ram_code_size = __ram_code_end - __ram_code_start;
    memcpy(__ram_code_start, __ram_code_load_start, ram_code_size);
}

static __RAM_CODE void hw_clock_init(void)
{
    System_ClkConfig_t sys_clk_cfg;

    sys_clk_cfg.SPLL_CFG.PLL_N = 6;
    sys_clk_cfg.SPLL_CFG.PLL_M = 0;
    sys_clk_cfg.SPLL_CFG.PowerEn = 1;
    sys_clk_cfg.MCU_Clock_Source = MCU_CLK_SEL_SPLL_CLK;
    sys_clk_cfg.SOC_DIV = 1;
    sys_clk_cfg.MCU_DIV = 1;
    sys_clk_cfg.APB0_DIV = 1;
    sys_clk_cfg.APB1_DIV = 1;
    sys_clk_cfg.APB2_DIV = 1;
    System_SPLL_config(&sys_clk_cfg.SPLL_CFG, 1000);
    System_MCU_clock_Config(&sys_clk_cfg);

    __SYSTEM_UART_CLK_SELECT_SPLL();
}

static __RAM_CODE void hw_xip_flash_init(bool wake_up)
{
    System_XIPConfig_t xip_config;
    xip_config.CLK_SRC_SEL = XIP_CLK_SEL_SPLL;
    xip_config.DIV_SEL = QSPI_BAUDRATE_DIV_4;
    xip_config.RD_TYPE = FLASH_RD_TYPE_DUAL;
    xip_config.WR_TYPE = FLASH_WR_TYPE_SINGLE;
    system_xip_flash_init(&xip_config, wake_up);
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 */
void soc_early_init_hook(void)
{
	copy_ram_code();

    system_delay_us(1000000);

	SystemInit();

    void main_entry_point(void);
    main_entry_point();
    pmu_init();

    hw_clock_init();
    hw_xip_flash_init(false);

    /* load controller firmware */
    const uint8_t bt_addr[] = {0x12, 0x34, 0x56, 0x78, 0xab, 0x00};
    controller_start(921600, bt_addr, bt_addr, 0);

    /* 
     * init MCU->BT pin, configure PMU_PIN_8 output BBG_EN signal, this pin is used to 
     * notice BT core that MCU is in working mode.
     */
    ool_write(PMU_REG_DIAG_CTRL, 0x82);
    ool_write(PMU_REG_PIN_IOMUX_H, 0x03);
}

// void soc_controller_init(void)
// {
    
// }

// SYS_INIT(soc_controller_init, APPLICATION, 90);
