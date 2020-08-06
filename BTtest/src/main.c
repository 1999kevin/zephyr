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


// uint8_t* buf = "deliver message\n";

const struct uart_config cfg = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

struct voltage_message {
	/* data */
	uint16_t SOF;
	uint16_t len;
	uint16_t cmd;
	uint16_t data;
	uint8_t crc;
	uint16_t EOF;
};


void uart_poll_out_multi(struct device *dev, unsigned char* str, int len);
int compose_message(struct voltage_message *msg, uint16_t voltage);
void voltage_message_format_convert(struct voltage_message *msg, uint8_t *buf);


void main(void)
{
	struct device *dev;
	// Usart2_SendStr("deliver message\n");
	dev = device_get_binding("UART_2");
	if (dev == NULL) {
		printk("cannot find device\n");
		return;
	}

	struct voltage_message msg;

	compose_message(&msg,233);
	
	if(uart_configure(dev, &cfg) != 0){
		printk("configure error\n");
	}
	printk("configure succuessfully\n");
	// int ret = uart_tx(dev, buf, 10, SYS_FOREVER_MS);
	// if (ret!=0){
	// 	printk("error code: %d\n",ret);
	// }else{
	// 	printk("write data successfully\n");
	// }
	unsigned char buf[15]={0};
	voltage_message_format_convert(&msg,buf);
	// uart_poll_out(dev, 'a');
	printk("char 0: %d\n",*buf);
	uart_poll_out_multi(dev,buf, msg.len);
	printk("send out telegraph\n");

}

int compose_message(struct voltage_message *msg, uint16_t voltage){
	msg->SOF = 0xb1b1;
	msg->len = 11;
	msg->cmd = 0;
	msg->data = voltage;
	msg->crc = msg->cmd+msg->data+msg->len;
	msg->EOF = 0x1b1b;
	printk("msg->SOF:%d\n",msg->SOF);
	return 0;
}


void uart_poll_out_multi(struct device *dev, unsigned char* str, int len){
	for (int i=0; i<len;i++){
		uart_poll_out(dev, *str++);
	}
}

void voltage_message_format_convert(struct voltage_message *msg, unsigned char* buf){
	*buf = (unsigned char)((msg->SOF >> 8) & 0xFF);
	*(buf+1) = (unsigned char)((msg->SOF) & 0xFF);
	*(buf+2) = (unsigned char)((msg->len >> 8) & 0xFF);
	*(buf+3) = (unsigned char)((msg->len) & 0xFF);
	*(buf+4) = (unsigned char)((msg->cmd >> 8) & 0xFF);
	*(buf+5) = (unsigned char)((msg->cmd) & 0xFF);
	*(buf+6) = (unsigned char)((msg->data >> 8) & 0xFF);
	*(buf+7) = (unsigned char)((msg->data) & 0xFF);
	*(buf+8) = (unsigned char)(msg->crc);
	*(buf+9) = (unsigned char)((msg->EOF >> 8) & 0xFF);
	*(buf+10) = (unsigned char)((msg->EOF) & 0xFF);

	printk("buf[0]:0x%02x \n",buf[0]);
}


