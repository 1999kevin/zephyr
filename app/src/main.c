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

	uint8_t* msg = "AT:MAC?\r\n";
	while(1){
		uart_poll_out_multi(dev_UART2, msg, 9);
		k_msleep(100);
		printk_buf(buf);
		k_msleep(2000);
	}

}
