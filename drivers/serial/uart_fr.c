/*
 * Copyright (c) 2025 Freqchip
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT freqchip_fr_uart

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

#include "soc.h"

struct fr_uart_config {
	uint32_t reg;
	uint16_t clkid;
	// struct reset_dt_spec reset;
	// const struct pinctrl_dev_config *pcfg;
	uint32_t parity;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct fr_uart_data {
	UART_HandleTypeDef uart;
	
	uint32_t baud_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_fr_isr(const struct device *dev)
{
	struct fr_uart_data *const data = dev->data;

	UART_HandleTypeDef *uart = &data->uart;
	switch(__UART_INT_GET_ID(uart->UARTx)) {
		case INT_INDEX_MODEM:
			break;
		case INT_INDEX_NONE:
			break;
		case INT_INDEX_TXE:
			__UART_INT_TXE_DISABLE(uart->UARTx);
			break;
		case INT_INDEX_RX:
			break;
		case INT_INDEX_LINE:
			break;
		case INT_INDEX_RX_TOUT:
			break;
		default:
			break;
	}

	*(volatile uint32_t *)0x1ffff000 = data->user_cb;
	*(volatile uint32_t *)0x1ffff004 = data->user_data;

	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_fr_init(const struct device *dev)
{
	const struct fr_uart_config *config = dev->config;
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;
	GPIO_InitTypeDef gpio_config;

	/* configure PB4 and PB5 to UART3 function */
    gpio_config.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio_config.Mode = GPIO_MODE_AF_PP;
    gpio_config.Pull = GPIO_PULLUP;
    gpio_config.Alternate = GPIO_FUNCTION_1;
    gpio_init(GPIOB, &gpio_config);

	__SYSTEM_UART3_CLK_ENABLE();
	uart->UARTx = (void *)config->reg;
    uart->Init.BaudRate   = data->baud_rate;
    uart->Init.DataLength = UART_DATA_LENGTH_8BIT;
    uart->Init.StopBits   = UART_STOPBITS_1;
    uart->Init.Parity     = UART_PARITY_NONE;
    uart->Init.FIFO_Mode  = UART_FIFO_ENABLE;
    uart->TxCpltCallback  = NULL;
    uart->RxCpltCallback  = NULL;
    uart_init(uart);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

static int uart_fr_poll_in(const struct device *dev, unsigned char *c)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;

	if (__UART_IS_RxFIFO_EMPTY(uart->UARTx)) {
		return -1;
	}

	*c = __UART_READ_FIFO(uart->UARTx);
	return 0;
}

static void uart_fr_poll_out(const struct device *dev, unsigned char c)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;

	while (__UART_IS_TxFIFO_FULL(uart->UARTx));
	__UART_WRITE_FIFO(uart->UARTx, c);
}

static int uart_fr_err_check(const struct device *dev)
{
	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
int uart_fr_fifo_fill(const struct device *dev, const uint8_t *tx_data,
			 int len)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;

	uart_transmit(uart, tx_data, len);
	__UART_INT_TXE_ENABLE_FE(uart->UARTx);
	
	return 0;
}

int uart_fr_fifo_read(const struct device *dev, uint8_t *rx_data,
			 const int size)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;
	int read_size = 0, last_size = size;

	while (!__UART_IS_RxFIFO_EMPTY(uart->UARTx) && last_size) {
		last_size--;
		read_size++;
		*rx_data++ = __UART_READ_FIFO(uart->UARTx);
	}
	
	return read_size;
}

void uart_fr_irq_tx_enable(const struct device *dev)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;
	__UART_INT_TXE_ENABLE_FE(uart->UARTx);
}

void uart_fr_irq_tx_disable(const struct device *dev)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;
	__UART_INT_TXE_DISABLE(uart->UARTx);
}

int uart_fr_irq_tx_ready(const struct device *dev)
{
	// struct fr_uart_data *data = dev->data;
	// UART_HandleTypeDef *uart = &data->uart;

	// return uart->b_TxBusy == false;

	return true;
}

int uart_fr_irq_tx_complete(const struct device *dev)
{
	// struct fr_uart_data *data = dev->data;
	// UART_HandleTypeDef *uart = &data->uart;
	
	// return uart->b_TxBusy == false;
	return true;
}

void uart_fr_irq_rx_enable(const struct device *dev)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;
	__UART_INT_RX_ENABLE(uart->UARTx);
}

void uart_fr_irq_rx_disable(const struct device *dev)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;
	__UART_INT_RX_DISABLE(uart->UARTx);
}

int uart_fr_irq_rx_ready(const struct device *dev)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;

	return __UART_IS_RxFIFO_EMPTY(uart->UARTx) == false;
}

void uart_fr_irq_err_enable(const struct device *dev)
{
}

void uart_fr_irq_err_disable(const struct device *dev)
{
}

int uart_fr_irq_is_pending(const struct device *dev)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;
	
	return __UART_INT_GET_ID(uart->UARTx) != INT_INDEX_NONE;
}

int uart_fr_irq_update(const struct device *dev)
{
	struct fr_uart_data *data = dev->data;
	UART_HandleTypeDef *uart = &data->uart;
	volatile REG_USR_t usr_status = uart->UARTx->USR;

	return 0;
}

void uart_fr_irq_callback_set(const struct device *dev,
				 uart_irq_callback_user_data_t cb,
				 void *user_data)
{
	struct fr_uart_data *const data = dev->data;

	data->user_cb = cb;
	data->user_data = user_data;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static DEVICE_API(uart, uart_fr_driver_api) = {
	.poll_in = uart_fr_poll_in,
	.poll_out = uart_fr_poll_out,
	.err_check = uart_fr_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_fr_fifo_fill,
	.fifo_read = uart_fr_fifo_read,
	.irq_tx_enable = uart_fr_irq_tx_enable,
	.irq_tx_disable = uart_fr_irq_tx_disable,
	.irq_tx_ready = uart_fr_irq_tx_ready,
	.irq_tx_complete = uart_fr_irq_tx_complete,
	.irq_rx_enable = uart_fr_irq_rx_enable,
	.irq_rx_disable = uart_fr_irq_rx_disable,
	.irq_rx_ready = uart_fr_irq_rx_ready,
	.irq_err_enable = uart_fr_irq_err_enable,
	.irq_err_disable = uart_fr_irq_err_disable,
	.irq_is_pending = uart_fr_irq_is_pending,
	.irq_update = uart_fr_irq_update,
	.irq_callback_set = uart_fr_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define FR_UART_IRQ_HANDLER(n)						\
	static void uart_fr_config_func_##n(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    uart_fr_isr,					\
			    DEVICE_DT_INST_GET(n),				\
			    0);							\
		irq_enable(DT_INST_IRQN(n));					\
	}
#define FR_UART_IRQ_HANDLER_FUNC_INIT(n)					\
	.irq_config_func = uart_fr_config_func_##n
#else /* CONFIG_UART_INTERRUPT_DRIVEN */
#define FR_UART_IRQ_HANDLER(n)
#define FR_UART_IRQ_HANDLER_FUNC_INIT(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define FR_UART_INIT(n)							\
	FR_UART_IRQ_HANDLER(n)						\
	static struct fr_uart_data uart_fr_data_##n = {			\
		.baud_rate = DT_INST_PROP(n, current_speed),			\
	};									\
	static const struct fr_uart_config uart_fr_config_##n = {		\
		.reg = DT_INST_REG_ADDR(n),					\
		.parity = DT_INST_ENUM_IDX(n, parity),				\
		 FR_UART_IRQ_HANDLER_FUNC_INIT(n)				\
	};									\
	DEVICE_DT_INST_DEFINE(n, uart_fr_init,				\
			      NULL,						\
			      &uart_fr_data_##n,				\
			      &uart_fr_config_##n, PRE_KERNEL_1,		\
			      CONFIG_SERIAL_INIT_PRIORITY,			\
			      &uart_fr_driver_api);

DT_INST_FOREACH_STATUS_OKAY(FR_UART_INIT)
