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
#include <drivers/gpio.h>


// uint8_t* buf = "deliver message\n";


#define FLAGS	0


#define PERIOD   10000
#define DUTY_CYCLE 0.01
#define PWM_FLAGS	0

void main(void)
{

	printk("here\n");


	struct device *dev_GPIOB = device_get_binding("GPIOB");
	struct device *dev_GPIOA = device_get_binding("GPIOA");
	struct device *dev_UART2 = device_get_binding("UART_2");
	bool led_is_on = true;
	int ret;

	if (dev_GPIOB == NULL || dev_GPIOA == NULL) {
		printk("cannnot find GPIOA or GPIOB\n");
		return;
	}

	/* set system on */
	ret = gpio_pin_configure(dev_GPIOB, 13, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_set(dev_GPIOB, 13, (int)led_is_on);
	printk("system on status: %d\n", ret);

	/* set led power on */
	ret = gpio_pin_configure(dev_GPIOB, 14, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_set(dev_GPIOB, 14, (int)led_is_on);
	printk("led power on status: %d\n", ret);
	
	/* set bluetooth working */
	ret = gpio_pin_configure(dev_GPIOA, 7, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_set(dev_GPIOA, 7, 0);
	printk("bluetooth status:%d\n",ret);

	/* set bluetooth slave mode */
	ret = gpio_pin_configure(dev_GPIOA, 5, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_set(dev_GPIOA, 5, 1);
	printk("bluetooth mode:%d\n",ret);


    ring_buf_init(&telegram_queue.rb, MY_RING_BUF_SIZE , telegram_queue.buffer);
	uart_fifo_init();

	uint8_t data[20];
	// uint8_t *print_data = data;
	int size;
	while(1){
		printk_buf_str(buf);
		printk("buf.size:%d\n",buf.size);  
		size = pull_one_message(data);
		// printk("ret:%d\n",size);
		if(size>0){
			printk("get data: %d\n",data[0]);
			printk("size: %d\n",size);
			break;
		}else{
			// printk("waiting for data, free space:%d\n",ring_buf_space_get(&telegram_queue.rb));
		}
		k_msleep(5000);
	}


	if(size == 11){
		printk("get a voltage message\n");
		uint16_t voltage = data[6]*16*16+data[7];
		printk("voltage:%d\n", voltage);
	}else if(size == 15){
		printk("get a pwm message\n");
		uint16_t period = data[6]*16*16+data[7];
		uint16_t pulse = data[8]*16*16+data[9];
		uint16_t duration = data[10]*16*16+data[11];

		printk("period: %d, pulse: %d, duration: %d\n", period, pulse, duration);


		// uint32_t period;
		int ret;
		// printk("PWM-based blinky\n");
		const char* label4 = "PWM_4";
		struct device *dev_pwm4 = device_get_binding(label4);
		if (!dev_pwm4) {
			printk("Error: didn't find %s device\n", label4);
			return;
		}
		printk("%s correct\n", label4);

		uint64_t cycles;
		ret = pwm_get_cycles_per_sec(dev_pwm4,1, &cycles);

		printk("clock rate: %lld\n",cycles);
		ret = pwm_pin_set_usec(dev_pwm4,1, period, pulse, PWM_FLAGS);
		if(ret < 0){
			printk("error %d\n",ret);
		}
		printk("set %s,channel 1, successfully\n",label4);
	}
	
}
