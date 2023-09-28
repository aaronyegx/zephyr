/*
 * Copyright (c) 2023 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ambiq_flash_controller

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>

#include <am_mcu_apollo.h>

LOG_MODULE_REGISTER(flash_ambiq, CONFIG_FLASH_LOG_LEVEL);

#define SOC_NV_FLASH_NODE  DT_INST(0, soc_nv_flash)
#define SOC_NV_FLASH_ADDR  DT_REG_ADDR(SOC_NV_FLASH_NODE)
#define SOC_NV_FLASH_SIZE  DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

struct flash_ambiq_data {
	struct k_sem mutex;
};

static struct flash_ambiq_data flash_data;

static const struct flash_parameters flash_ambiq_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

static bool flash_ambiq_valid_range(off_t offset, size_t len, bool read)
{
	if ((offset > SOC_NV_FLASH_SIZE) || ((offset + len) > SOC_NV_FLASH_SIZE)) {
		return false;
	}

	if (!read) {
		// TODO
	}

	return true;
}

static int flash_ambiq_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	ARG_UNUSED(dev);

	if (!flash_ambiq_valid_range(offset, len, true)) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	memcpy(data, (uint8_t *)(SOC_NV_FLASH_ADDR + offset), len);

	return 0;
}

static int flash_ambiq_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	struct flash_ambiq_data *dev_data = dev->data;
	int ret = 0;
	uint32_t ui32Critical = 0;

	if (!flash_ambiq_valid_range(offset, len, false)) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	k_sem_take(&dev_data->mutex, K_FOREVER);

	ui32Critical = am_hal_interrupt_master_disable();
	ret = am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, (uint32_t *)data,
				       (uint32_t *)(SOC_NV_FLASH_ADDR + offset), (len / 4));
	am_hal_interrupt_master_set(ui32Critical);

	k_sem_give(&dev_data->mutex);

	return ret;
}

static int flash_ambiq_erase(const struct device *dev, off_t offset, size_t len)
{
	struct flash_ambiq_data *data = dev->data;
	int ret = 0;

	if (!flash_ambiq_valid_range(offset, len, false)) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	k_sem_take(&data->mutex, K_FOREVER);

	ret = am_hal_mram_main_fill(AM_HAL_MRAM_PROGRAM_KEY, 0xFFFFFFFF,
				    (uint32_t *)(SOC_NV_FLASH_ADDR + offset), (len / 4));

	k_sem_give(&data->mutex);

	return ret;
}

static const struct flash_parameters *flash_ambiq_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_ambiq_parameters;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout pages_layout = {
	.pages_count = DT_REG_SIZE(SOC_NV_FLASH_NODE) / FLASH_ERASE_BLK_SZ,
	.pages_size = FLASH_ERASE_BLK_SZ,
};

static void flash_ambiq_pages_layout(const struct device *dev,
				     const struct flash_pages_layout **layout, size_t *layout_size)
{
	*layout = &pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api flash_ambiq_driver_api = {
	.read = flash_ambiq_read,
	.write = flash_ambiq_write,
	.erase = flash_ambiq_erase,
	.get_parameters = flash_ambiq_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_ambiq_pages_layout,
#endif
};

static int flash_ambiq_init(const struct device *dev)
{
	struct flash_ambiq_data *data = dev->data;

	k_sem_init(&data->mutex, 1, 1);

	return 0;
}

DEVICE_DT_INST_DEFINE(0, flash_ambiq_init, NULL, &flash_data, NULL, POST_KERNEL,
		      CONFIG_FLASH_INIT_PRIORITY, &flash_ambiq_driver_api);
