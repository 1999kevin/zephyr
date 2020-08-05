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


uint8_t* buf = "deliver message\n";

void uart_poll_out_multi(struct device *dev, unsigned char* str);

const struct uart_config cfg = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

void main(void)
{
	struct device *dev;
	// Usart2_SendStr("deliver message\n");
	dev = device_get_binding("UART_2");
	if (dev == NULL) {
		printk("cannot find device\n");
		return;
	}
	
	if(uart_configure(dev, &cfg) != 0){
		printk("configure error\n");
	}
	printk("configure succuessfully\n");
	int ret = uart_tx(dev, buf, 10, SYS_FOREVER_MS);
	if (ret!=0){
		printk("error code: %d\n",ret);
	}else{
		printk("write data successfully\n");
	}
	uart_poll_out_multi(dev,buf);

}



void uart_poll_out_multi(struct device *dev, unsigned char* str){
	while(*str != '\0'){
		uart_poll_out(dev, *str++);
	}
}

