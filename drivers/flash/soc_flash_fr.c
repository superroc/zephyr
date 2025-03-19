/*
 * Copyright (c) 2025 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT freqchip_fr_flash_controller
#define FLASH_SIZE   DT_REG_SIZE(DT_INST(0, soc_nv_flash))
#define FLASH_ORIGIN DT_REG_ADDR(DT_INST(0, soc_nv_flash))

#include "fr30xx.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>

/* driver definitions */
#define PAGE_SIZE               256
#define SECTOR_SIZE             (0x1000u)

/* driver parameters structure */
static const struct flash_parameters flash_fr_parameters = {
	.write_block_size = DT_PROP(DT_INST(0, soc_nv_flash), write_block_size),
	.erase_value = 0xff,
};


/* Check for correct offset and length */
static bool flash_fr_is_range_valid(off_t offset, size_t len)
{
	/* check for min value */
	if ((offset < 0) || (len < 1)) {
		return false;
	}

	/* check for max value */
	if ((offset + len) > FLASH_SIZE) {
		return false;
	}

	return true;
}

/* API implementation: driver initialization */
static int flash_fr_init(const struct device *dev)
{
	return 0;
}

/* API implementation: erase */
static int flash_fr_erase(const struct device *dev, off_t offset, size_t len)
{
    ARG_UNUSED(dev);

	int page_nums = len / SECTOR_SIZE;

	/* return SUCCESS if len equals 0 (required by tests/drivers/flash) */
	if (!len) {
		return 0;
	}

	/* check for valid range */
	if (!flash_fr_is_range_valid(offset, len)) {
		return -EINVAL;
	}

	/* erase can be done only by pages */
	if (((offset % SECTOR_SIZE) != 0) || ((len % SECTOR_SIZE) != 0)) {
		return -EINVAL;
	}

	while (page_nums) {
		FR_DRIVER_WRAPPER(flash_erase)(QSPI0, FLASH_ORIGIN + offset, SECTOR_SIZE);
        page_nums--;
        offset += SECTOR_SIZE;
	}

	return 0;
}

/* API implementation: write */
static int flash_fr_write(const struct device *dev, off_t offset,
			   const void *data, size_t len)
{
    ARG_UNUSED(dev);

	/* return SUCCESS if len equals 0 (required by tests/drivers/flash) */
	if (!len) {
		return 0;
	}

	/* check for valid range */
	if (!flash_fr_is_range_valid(offset, len)) {
		return -EINVAL;
	}

	/* write flash */
	FR_DRIVER_WRAPPER(flash_write)(QSPI0, FLASH_ORIGIN + offset, len, data);

	return 0;
}

/* API implementation: read */
static int flash_fr_read(const struct device *dev, off_t offset,
			  void *data, size_t len)
{
	ARG_UNUSED(dev);

	/* return SUCCESS if len equals 0 (required by tests/drivers/flash) */
	if (!len) {
		return 0;
	}

	/* check for valid range */
	if (!flash_fr_is_range_valid(offset, len)) {
		return -EINVAL;
	}

	/* read flash */
	FR_DRIVER_WRAPPER(flash_read)(QSPI0, FLASH_ORIGIN + offset, len, (unsigned char *)data);

	return 0;
}

/* API implementation: get_parameters */
static const struct flash_parameters *
flash_fr_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_fr_parameters;
}

/* API implementation: page_layout */
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout dev_layout = {
	.pages_count = FLASH_SIZE / SECTOR_SIZE,
	.pages_size = SECTOR_SIZE,
};

static void flash_fr_pages_layout(const struct device *dev,
				   const struct flash_pages_layout **layout,
				   size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static DEVICE_API(flash, flash_fr_api) = {
	.erase = flash_fr_erase,
	.write = flash_fr_write,
	.read = flash_fr_read,
	.get_parameters = flash_fr_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_fr_pages_layout,
#endif
};

/* Driver registration */
DEVICE_DT_INST_DEFINE(0, flash_fr_init,
		      NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_fr_api);
