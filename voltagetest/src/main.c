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
// #include <drivers/pwm.h>
// #include <drivers/gpio.h>
#include <drivers/adc.h>

void PrintFloat(float value);

static const struct adc_channel_cfg adc_ch_cfg = {
	.gain  = ADC_GAIN_1,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = 1,
	.differential  = 0
};

uint16_t buffer[100];

const struct adc_sequence sequence = {
	.options = NULL,
	.channels = BIT(1),
	.buffer = buffer,
	.buffer_size = 10,
	.resolution = 12
};

void main(void)
{
	struct device *dev;
	int ret;

	dev = device_get_binding("ADC_1");
	if (dev == NULL) {
		printk("Cannot get ADC device\n");
		return;
	}

	ret = adc_channel_setup(dev, &adc_ch_cfg);

	if (ret != 0) {
		printk("Setting up of adc channel failed with code %d\n", ret);
		return;
	}

	if(adc_read(dev,&sequence) < 0){
		printk("cannot read value\n");
	}else{
		float result = buffer[0];
		printk("voltage:");
		PrintFloat(result*3300/4096);
		printk("mV\n");
	}

}

void PrintFloat(float value){
	int tmp,tmp1,tmp2,tmp3,tmp4,tmp5,tmp6;
	tmp = (int)value;
	tmp1=(int)((value-tmp)*10)%10;
	tmp2=(int)((value-tmp)*100)%10;
	tmp3=(int)((value-tmp)*1000)%10;
	tmp4=(int)((value-tmp)*10000)%10;
	tmp5=(int)((value-tmp)*100000)%10;
	tmp6=(int)((value-tmp)*1000000)%10;
	printk("%d.%d%d%d%d%d%d ",tmp,tmp1,tmp2,tmp3,tmp4,tmp5,tmp6);

}




