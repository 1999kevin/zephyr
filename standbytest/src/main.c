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
#include "stm32f1xx_hal.h"
// #include "stm32f1xx_hal_def.h"
// #include "stm32f1xx_ll_usb.h"
// #include "pwr.h"

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

void NEW_HAL_PWR_EnterSTANDBYMode(void);


void main(void)
{
	struct device *dev;
	bool led_is_on = false;
	int ret;

	dev = device_get_binding("GPIOB");
	if (dev == NULL) {
		return;
	}

	int pin13 = 13;

	// printk("pin:%d\n",PIN);
	ret = gpio_pin_configure(dev, pin13, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}


	gpio_pin_set(dev, pin13, (int)led_is_on);

	printk("set PB13 off \n");
	NEW_HAL_PWR_EnterSTANDBYMode();
	printk("do not enter standby mode\n");
}



void NEW_HAL_PWR_EnterSTANDBYMode(void)
{
  /* Select STANDBY mode */
  PWR->CR |= (uint32_t)PWR_CR_PDDS;

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  /* This option is used to ensure that store operations are completed */
#if defined ( __CC_ARM)
  __force_stores();
#endif
  /* Request Wait For Interrupt */
  __WFI();
}

