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
#include <drivers/pwm.h>


// uint8_t* buf = "deliver message\n";

#define PERIOD   10000
#define DUTY_CYCLE 0.01
#define PWM_FLAGS	0

void main(void)
{

	printk("here\n");

    ring_buf_init(&telegram_queue.rb, MY_RING_BUF_SIZE , telegram_queue.buffer);
	uart_fifo_init();

	uint8_t data[20];
	// uint8_t *print_data = data;
	while(1){
		int size = pull_one_message(data);
		printk("ret:%d\n",size);
		if(size>0){
			printk("get data: %d\n",data[0]);
			break;
		}else{
			printk("waiting for data, free space:%d\n",ring_buf_space_get(&telegram_queue.rb));
		}
		k_msleep(5000);
	}

	uint16_t voltage = data[6]*16*16+data[7];
	printk("voltage:%d\n", voltage);

	struct device *pwm4;
	// uint32_t period;
	int ret;
	printk("PWM-based blinky\n");
	const char* label4 = "PWM_4";
	pwm4 = device_get_binding(label4);
	if (!pwm4) {
		printk("Error: didn't find %s device\n", label4);
		return;
	}
	printk("%s correct\n", label4);

	uint64_t cycles;
	ret = pwm_get_cycles_per_sec(pwm4,1, &cycles);

	printk("clock rate: %lld\n",cycles);
	ret = pwm_pin_set_usec(pwm4,1, PERIOD, PERIOD*0.01, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
	}
	printk("set %s,channel 1, successfully\n",label4);

}
