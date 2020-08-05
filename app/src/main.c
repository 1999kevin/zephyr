/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM.
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/uart.h>
#include "BT.h"


// uint8_t* buf = "deliver message\n";

struct voltage_message {
	/* data */
	uint16_t SOF;
	uint16_t len;
	uint16_t cmd;
	uint16_t data;
	uint8_t crc;
	uint16_t EOF;
};

struct read_in_buffer {
	uint8_t *buf;
	int size;
};


void uart_fifo_callback(struct device *dev);
int uart_fifo_init(void);


const struct uart_config cfg = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

struct device *dev;

uint8_t read_in_buf[20]={0};

struct read_in_buffer buf = {
	.buf = read_in_buf,
	.size = 0
};

void main(void)
{
	uart_fifo_init();
	printk("here\n");

}

int uart_fifo_init(void){
	uint8_t c;
	dev = device_get_binding("UART_2");
	if (dev == NULL) {
		printk("cannot find device\n");
		return -1;
	}
	if(uart_configure(dev, &cfg) != 0){
		printk("configure error\n");
	}
	printk("configure succuessfully\n");
		
	uart_irq_callback_set(dev, uart_fifo_callback);

	/* Drain the fifo */
	if (uart_irq_rx_ready(dev)) {
		while (uart_fifo_read(dev, &c, 1)) {
			;
		}
	}
	uart_irq_rx_enable(dev);

	/* Enable all interrupts unconditionally. Note that this is due
	 * to Zephyr issue #8393. This should be removed once the
	 * issue is fixed in upstream Zephyr.
	 */
	irq_unlock(0);

	return 0;

}

void uart_fifo_callback(struct device *dev){
	printk("come to callback\n");
	uint8_t byte[4];
	int rx;

	uart_irq_update(dev);

	if (!uart_irq_rx_ready(dev)) {
		return;
	}
	// int i=0;
	printk("ready to read\n");
	while (true) {
		rx = uart_fifo_read(dev, &buf.buf[buf.size], 1);
		printk("rx:%d\n",rx);
		if (rx != 1) {
			break;
		}
	}
	buf.size++;
	printk("callback end\n");
	printk("byte: 0x%02x 0x%02x 0x%02x\n",buf.buf[0], buf.buf[1],buf.buf[2]);
	// printk("byte :0x%02x, 0x%02x,0x%02x,0x%02x\n",read_in_buf[0],read_in_buf[1],read_in_buf[2],read_in_buf[3]);

}


