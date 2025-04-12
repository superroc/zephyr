/*
 * Copyright (c) 2025 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT freqchip_freqchip_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

struct gpio_freqchip_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* port base address */
	uint32_t *base;
};

struct gpio_freqchip_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* output old value 1bit->1pin */
	uint32_t old_output_value;
};

static int gpio_freqchip_pin_configure(const struct device *port,
				  gpio_pin_t pin,
				  gpio_flags_t flags)
{
	const struct gpio_freqchip_config *cfg = port->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)cfg->base;
	struct gpio_freqchip_data *data = port->data;

	printf("%s %s->%p:%d,%x,%x\n", __func__, port->name, gpio, pin, flags, data->old_output_value);

	GPIO_InitTypeDef GPIO_Handle = {0};
	GPIO_Handle.Pin = 1<<pin;
	if (flags & GPIO_OUTPUT) {
		GPIO_Handle.Mode = GPIO_MODE_OUTPUT_PP;
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			GPIO_Handle.Pull = GPIO_PULLUP;
		} else {
			GPIO_Handle.Pull = GPIO_PULLDOWN;
		}
	} else if (flags & GPIO_INPUT) {
		GPIO_Handle.Mode = GPIO_MODE_INPUT;
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			GPIO_Handle.Pull = GPIO_PULLUP;
		} else {
			GPIO_Handle.Pull = GPIO_PULLDOWN;
		}
	}
	gpio_init(gpio, &GPIO_Handle);

	if (flags & GPIO_OUTPUT) {
		uint16_t pin_value = 1<<pin;
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			data->old_output_value &= ~pin_value;
			gpio_write_pin(gpio, pin_value, GPIO_PIN_CLEAR);
		} else if (flags & GPIO_OUTPUT_INIT_HIGH) {
			data->old_output_value |= pin_value;
			gpio_write_pin(gpio, pin_value, GPIO_PIN_SET);
		}
		printf("%x,%x\n", pin_value, data->old_output_value);
	}

	// 这里需要判断这个IO是否可用，我现在默认都是可用的
	if (true) {
		return 0;
	} else {
		return -ENOTSUP;
	}
}

static int gpio_freqchip_port_get_raw(const struct device *port,
				 gpio_port_value_t *value)
{
	printf("%s\n", __func__);
	// 这里需要判断这个IO是否可用，我现在默认都是可用的
	if (true) {
		return 0;
	} else {
		return -ENOTSUP;
	}
}

static int gpio_freqchip_port_set_masked_raw(const struct device *port,
					gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	printf("%s\n", __func__);
	// 这里需要判断这个IO是否可用，我现在默认都是可用的
	if (true) {
		return 0;
	} else {
		return -ENOTSUP;
	}
}

static int gpio_freqchip_port_set_bits_raw(const struct device *port,
				      gpio_port_pins_t pins)
{
	struct gpio_freqchip_data *data = port->data;
	data->old_output_value |= pins;
	printf("%s:%x,%x\n", __func__, pins, data->old_output_value);
	const struct gpio_freqchip_config *cfg = port->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)cfg->base;
	gpio_write_pin(gpio, pins, GPIO_PIN_SET);
	// 这里需要判断这个IO是否可用，我现在默认都是可用的
	if (true) {
		return 0;
	} else {
		return -ENOTSUP;
	}
}

static int gpio_freqchip_port_clear_bits_raw(const struct device *port,
					gpio_port_pins_t pins)
{
	struct gpio_freqchip_data *data = port->data;
	data->old_output_value &= ~pins;
	printf("%s:%x,%x\n", __func__, pins, data->old_output_value);
	const struct gpio_freqchip_config *cfg = port->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)cfg->base;
	gpio_write_pin(gpio, pins, GPIO_PIN_CLEAR);
	// 这里需要判断这个IO是否可用，我现在默认都是可用的
	if (true) {
		return 0;
	} else {
		return -ENOTSUP;
	}
}

static int gpio_freqchip_port_toggle_bits(const struct device *port,
				     gpio_port_pins_t pins)
{
	struct gpio_freqchip_data *data = port->data;
	data->old_output_value ^= pins;
	printf("%s:%x,%x\n", __func__, pins, data->old_output_value);
	const struct gpio_freqchip_config *cfg = port->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)cfg->base;
	if (data->old_output_value & pins) {
		gpio_write_pin(gpio, pins, GPIO_PIN_SET);
	} else {
		gpio_write_pin(gpio, pins, GPIO_PIN_CLEAR);
	}
	// 这里需要判断这个IO是否可用，我现在默认都是可用的
	if (true) {
		return 0;
	} else {
		return -ENOTSUP;
	}
}

static DEVICE_API(gpio, gpio_freqchip_api) = {
	.pin_configure = gpio_freqchip_pin_configure,
	.port_get_raw = gpio_freqchip_port_get_raw,
	.port_set_masked_raw = gpio_freqchip_port_set_masked_raw,
	.port_set_bits_raw = gpio_freqchip_port_set_bits_raw,
	.port_clear_bits_raw = gpio_freqchip_port_clear_bits_raw,
	.port_toggle_bits = gpio_freqchip_port_toggle_bits,
};

static int gpio_freqchip_init(const struct device *dev) {
    printf("FreqChip GPIO %s Initialized\n", dev->name);
    return 0;
}

#define GPIO_FREQCHIP_INIT(n)						\
	static const struct gpio_freqchip_config gpio_freqchip_config_##n = {	\
		.common = {						\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n), \
		},							\
		.base = (uint32_t *)DT_INST_REG_ADDR(n), \
	};								\
									\
	static struct gpio_freqchip_data gpio_freqchip_data_##n;			\
									\
	DEVICE_DT_INST_DEFINE(n, gpio_freqchip_init, NULL, &gpio_freqchip_data_##n,	\
			      &gpio_freqchip_config_##n, POST_KERNEL,	\
			      CONFIG_GPIO_INIT_PRIORITY,		\
			      &gpio_freqchip_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_FREQCHIP_INIT)

// DEVICE_DT_INST_DEFINE(n, gpio_freqchip_init, ...);