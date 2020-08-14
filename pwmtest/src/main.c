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
#include <drivers/pwm.h>
#include <drivers/gpio.h>

/* 1000 msec = 1 sec */
#define PERIOD   10000
#define DUTY_CYCLE 0.1
#define PWM_FLAGS	0

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#if DT_PHA_HAS_CELL(LED0_NODE, gpios, flags)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#endif
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#endif

#ifndef FLAGS
#define FLAGS	0
#endif

void main(void)
{
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
	/*
	 * In case the default MAX_PERIOD_USEC value cannot be set for
	 * some PWM hardware, decrease its value until it can.
	 *
	 * Keep its value at least MIN_PERIOD_USEC * 4 to make sure
	 * the sample changes frequency at least once.
	 */
	// printk("Calibrating for device %s channel %d...\n",
	//        PWM_LABEL, PWM_CHANNEL);
	// period = 10000;
	ret = pwm_pin_set_usec(pwm4, 1, PERIOD, PERIOD*DUTY_CYCLE, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
	}
	printk("set %s,channel 1, successfully\n",label4);
	ret = pwm_pin_set_usec(pwm4, 2, PERIOD, PERIOD*DUTY_CYCLE, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
	}
	printk("set %s,channel 2, successfully\n",label4);


	// struct device *pwm2;
	// const char* label2 = "PWM_2";
	// pwm2 = device_get_binding(label2);
	// if (!pwm2) {
	// 	printk("Error: didn't find %s device\n", label2);
	// 	return;
	// }
	// printk("%s correct\n", label2);
	// /*
	//  * In case the default MAX_PERIOD_USEC value cannot be set for
	//  * some PWM hardware, decrease its value until it can.
	//  *
	//  * Keep its value at least MIN_PERIOD_USEC * 4 to make sure
	//  * the sample changes frequency at least once.
	//  */
	// // printk("Calibrating for device %s channel %d...\n",
	// //        PWM_LABEL, PWM_CHANNEL);
	// // period = 10000;
	// ret = pwm_pin_set_usec(pwm2, 3, PERIOD, PERIOD*DUTY_CYCLE, PWM_FLAGS);
	// if(ret < 0){
	// 	printk("error %d\n",ret);
	// }
	// printk("set %s,channel 3, successfully\n",label2);
	// ret = pwm_pin_set_usec(pwm2, 4, PERIOD, PERIOD*DUTY_CYCLE, PWM_FLAGS);
	// if(ret < 0){
	// 	printk("error %d\n",ret);
	// }
	// printk("set %s,channel 4, successfully\n",label2);


	struct device *dev;
	bool led_is_on = true;
	int ret;

	dev = device_get_binding("GPIOB");
	if (dev == NULL) {
		return;
	}

	int pin10 = 10;
	int pin11 = 11;

	// printk("pin:%d\n",PIN);
	ret = gpio_pin_configure(dev, pin10, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_configure(dev, pin11, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	while (1) {
		gpio_pin_set(dev, pin10, (int)led_is_on);
		gpio_pin_set(dev, pin11, (int)led_is_on);
		led_is_on = !led_is_on;
		k_usleep(PERIOD*DUTY_CYCLE);

		gpio_pin_set(dev, pin10, (int)led_is_on);
		gpio_pin_set(dev, pin11, (int)led_is_on);
		led_is_on = !led_is_on;
		k_usleep(PERIOD-PERIOD*DUTY_CYCLE);

	}

}
