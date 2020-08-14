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
#include <drivers/pwm.h>
#include <drivers/gpio.h>
#include "BT.h"
#include "main.h"



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
	struct device *dev_ADC1 = device_get_binding("ADC_1");
	bool led_is_on = true;
	int ret;

	if (dev_GPIOB == NULL || dev_GPIOA == NULL || dev_UART2 == NULL || dev_ADC1 == NULL) {
		printk("cannnot find GPIOA or GPIOB or UART2 or ADC1\n");
		return;
	}

	/* set system on */
	ret = gpio_pin_configure(dev_GPIOB, 13, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_set(dev_GPIOB, 13, (int)led_is_on);
	printk("system on status: %d\n", ret);
	
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

	// /* set BAT_EN on */
	// ret = gpio_pin_configure(dev_GPIOA, 1, GPIO_OUTPUT_ACTIVE | FLAGS);
	// if (ret < 0) {
	// 	printk("get out from here\n");
	// 	return;
	// }
	// ret = gpio_pin_set(dev_GPIOA, 1, 1);
	// printk("BAT_EN:%d\n",ret);




	/* initiate ring buf */
    ring_buf_init(&telegram_queue.rb, MY_RING_BUF_SIZE , telegram_queue.buffer);
	uart_fifo_init();


	/* wait for bluetooth to connect */
	// int size;
	// // uint8_t* ask_msg = "AT:RSSI?\r\n";

	// while(1){
	// 	ret = gpio_pin_get(dev_GPIOA, 6);
	// 	if(ret == 0){
	// 		break;
	// 	}
	// 	else{
	// 		printk("bluetooth not connected\n");
	// 	}
	// 	// uart_poll_out_multi(dev_UART2, ask_msg, 10);
	// 	k_msleep(1000);
	// 	// printk_buf_str(buf);
	// 	// k_msleep(2000);
	// 	// printk_buf_str(buf);

	// }
	// printk("bluetooth connected\n");
	// k_msleep(5000);

	// // /* obtain voltage meeage */
	// float result = 0;
	// ret = adc_channel_setup(dev_ADC1, &adc_ch_cfg);
	// // printk("here2\n");
	// if (ret != 0) {
	// 	printk("Setting up of adc channel failed with code %d\n", ret);
	// 	return;
	// }

	// while(1){
	// 	if(adc_read(dev_ADC1,&sequence) < 0){
	// 		printk("cannot read value\n");
	// 	}else{
	// 		float result = buffer[0];
	// 		result = result/4096.0*3300.0*2.0;
	// 		// printk("result:%d\n",(int)result);
	// 		printk("voltage:%d\n",buffer[0]);
	// 		// voltage_result = (int)(result*3300.0/4096.0*2.0);
	// 		PrintFloat(result);
	// 		printk("mV\n");
	// 	}
	// 	k_msleep(1000);
	// }


	// if(adc_read(dev_ADC1,&sequence) < 0){
	// 	printk("cannot read value\n");
	// }else{
	// 	result = buffer[0]/4096.0*3300.0*2.0;
	// 	// printk("result:%d\n",(int)result);
	// 	printk("voltage:\n");
	// 	// voltage_result = (int)(result*3300.0/4096.0*2.0);
	// 	PrintFloat(result);
	// 	printk("mV\n");
	// }

	// /* transmit voltage message */
	// struct voltage_message msg;
	// compose_message(&msg,(int)(result));

	// unsigned char voltage_buf[15]={0};
	// voltage_message_format_convert(&msg,voltage_buf);
	// // uart_poll_out(dev, 'a');
	// printk("char 0: %d\n",*voltage_buf);
	// uart_poll_out_multi(dev_UART2,voltage_buf, msg.len);
	// printk("send out telegraph\n");

	/* wait for pwm message */
	uint8_t data[20];
	uint8_t* msg = "AT:MAC?\r\n";

	int size;
	while(1){

		// uart_poll_out_multi(dev_UART2, msg, 9);
		// k_msleep(100);
		// printk_buf_str(buf);
	// 	// k_msleep(2000);
	// 	// printk_buf_str(buf);
	// 	// printk("buf.size:%d\n",buf.size);  
		size = pull_one_message(data);
		// printk("ret:%d\n",size);
		if(size>0){
			printk("get data: %d\n",data[0]);
			printk("size: %d\n",size);
			break;
		}else{
			printk("waiting for data\n");
		}
		printk_buf_str(buf);
		k_msleep(5000);

	}

	/* set led power on */
	ret = gpio_pin_configure(dev_GPIOB, 14, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_set(dev_GPIOB, 14, (int)led_is_on);
	printk("led power on status: %d\n", ret);

	uint16_t period = 0;
	uint16_t pulse = 0;
	uint16_t duration = 0;
	if(size == 11){
		printk("get a voltage message\n");
		uint16_t voltage = data[6]*16*16+data[7];
		printk("voltage:%d\n", voltage);
	}else if(size == 15){
		printk("get a pwm message\n");
		period = data[6]*16*16+data[7];
		pulse = data[8]*16*16+data[9];
		duration = data[10]*16*16+data[11];

		printk("period: %d, pulse: %d, duration: %d\n", period, pulse, duration);
	}

		// uint32_t period;
		// int ret;
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
	ret = pwm_pin_set_usec(dev_pwm4,2, period, pulse, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
	}
	printk("set %s,channel 2, successfully\n",label4);



		// uint32_t period;
		// int ret;
		// printk("PWM-based blinky\n");
	const char* label2 = "PWM_2";
	struct device *dev_pwm2 = device_get_binding(label2);
	if (!dev_pwm2) {
		printk("Error: didn't find %s device\n", label2);
		return;
	}
	printk("%s correct\n", label2);

		// uint64_t cycles;
		// ret = pwm_get_cycles_per_sec(dev_pwm4,1, &cycles);

		// printk("clock rate: %lld\n",cycles);
	ret = pwm_pin_set_usec(dev_pwm2,3, period, pulse, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
	}
	printk("set %s,channel 3, successfully\n",label2);
	ret = pwm_pin_set_usec(dev_pwm2,4, period, pulse, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
	}
	printk("set %s,channel 4, successfully\n",label2);

	
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

