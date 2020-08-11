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

    // ring_buf_init(&telegram_queue.rb, MY_RING_BUF_SIZE , telegram_queue.buffer);
	// uart_fifo_init();

	// uint8_t data[20];
	// // uint8_t *print_data = data;
	// int size;
	// while(1){
	// 	size = pull_one_message(data);
	// 	// printk("ret:%d\n",size);
	// 	if(size>0){
	// 		printk("get data: %d\n",data[0]);
	// 		break;
	// 	}else{
	// 		printk("waiting for data, free space:%d\n",ring_buf_space_get(&telegram_queue.rb));
	// 	}
	// 	k_msleep(5000);
	// }

	// if(size == 11){
	// 	printk("get a voltage message\n");
	// 	uint16_t voltage = data[6]*16*16+data[7];
	// 	printk("voltage:%d\n", voltage);
	// }else if(size == 15){
	// 	printk("get a pwm message\n");
	// 	uint16_t period = data[6]*16*16+data[7];
	// 	uint16_t pulse = data[8]*16*16+data[9];
	// 	uint16_t duration = data[10]*16*16+data[11];

	// 	printk("period: %d, pulse: %d, duration: %d\n", period, pulse, duration);

	// 	struct device *pwm4;
	// 	// uint32_t period;
	// 	int ret;
	// 	printk("PWM-based blinky\n");
	// 	const char* label4 = "PWM_4";
	// 	pwm4 = device_get_binding(label4);
	// 	if (!pwm4) {
	// 		printk("Error: didn't find %s device\n", label4);
	// 		return;
	// 	}
	// 	printk("%s correct\n", label4);

	// 	uint64_t cycles;
	// 	ret = pwm_get_cycles_per_sec(pwm4,1, &cycles);

	// 	printk("clock rate: %lld\n",cycles);
	// 	ret = pwm_pin_set_usec(pwm4,1, period, pulse, PWM_FLAGS);
	// 	if(ret < 0){
	// 		printk("error %d\n",ret);
	// 	}
	// 	printk("set %s,channel 1, successfully\n",label4);
	// }	dev = device_get_binding("GPIOB");

	struct device *dev;
	bool led_is_on = true;
	int ret;


	dev = device_get_binding("GPIOB");

	if (dev == NULL) {
		return;
	}

	// int pin10 = 13;
	// int pin11 = 11;

	// printk("pin:%d\n",PIN);

	ret = gpio_pin_configure(dev, 13, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_set(dev, 13, (int)led_is_on);
	printk("system on status: %d\n", ret);

	ret = gpio_pin_configure(dev, 14, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_set(dev, 14, (int)led_is_on);
	printk("led power on status: %d\n", ret);
	
	
	// ret = gpio_pin_configure(dev, 10, GPIO_OUTPUT_ACTIVE | FLAGS);
	// if (ret < 0) {
	// 	return;
	// }
	// ret = gpio_pin_set(dev, 10, (int)led_is_on);
	// printk("led1_en status:%d\n",ret);
	// ret = gpio_pin_configure(dev, 11, GPIO_OUTPUT_ACTIVE | FLAGS);
	// if (ret < 0) {
	// 	return;
	// }
	// ret = gpio_pin_set(dev, 11, (int)led_is_on);
	// printk("led1_en status:%d\n",ret);






	printk("here2\n");

	struct device *pwm4;
	// uint32_t period;

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
	ret = pwm_pin_set_usec(pwm4, 2, 10000, 1000, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
	}
	printk("set %s,channel 1, successfully\n",label4);


}
