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
#include "BT.h"


// uint8_t* buf = "deliver message\n";

void main(void)
{

	printk("here\n");

	// voltage_telegram_ready(struct read_in_buffer)
    ring_buf_init(&telegram_queue.rb, MY_RING_BUF_SIZE , telegram_queue.buffer);
	uart_fifo_init();

	uint8_t data[20];
	// uint8_t *print_data = data;
	while(1){
		int size = pull_one_message(data);
		printk("ret:%d\n",size);
		if(size>0){
			printk("get data: %d\n",data[0]);
		}else{
			printk("waiting for data, free space:%d\n",ring_buf_space_get(&telegram_queue.rb));
		}
		k_msleep(5000);
	}


	uint8_t *messgae="message";
	uint8_t ret_data[8];
	int ret=ring_buf_put(&telegram_queue.rb,messgae,7);
	printk("enqueue:%d\n",ret);
	ret=ring_buf_get(&telegram_queue.rb,ret_data,7);
	printk("ret:%d\n",ret);
	printk("ret:data:%s\n",ret_data);
}
