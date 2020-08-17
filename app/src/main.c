
#include "function.h"


#define PERIOD   10000
#define DUTY_CYCLE 0.01


void main(void)
{

	printk("here\n");
	// bool led_is_on = true;
	int ret;


	ret = system_init();
	if(ret < 0){
		return;
	}

	/* initiate ring buf */
    ring_buf_init(&telegram_queue.rb, MY_RING_BUF_SIZE , telegram_queue.buffer);
	uart_fifo_init();


	/* wait for bluetooth to connect */
	while(1){
		ret = gpio_pin_get(dev_GPIOA, 6);
		if(ret == 0){
			break;
		}
		else{
			printk("bluetooth not connected\n");
		}
		k_msleep(1000);
	}
	printk("bluetooth connected\n");
	k_msleep(5000);


	/* obtain voltage message */
	float voltage_result = get_voltage();
	if(voltage_result<0){
		return;
	}

	// /* transmit voltage message */
	transmit_voltage_message((int)voltage_result);


	/* wait for message */
	uint8_t data[20];
	// uint8_t* msg = "AT:MAC?\r\n";

	int size;
	while(1){
		size = pull_one_message(data);
		if(size>0){
			break;
		}else{
			printk("waiting for data\n");
		}
		printk_buf_str(buf);
		k_msleep(5000);

	}

	/* set led power on */
	set_led_power_on();

	/* decode message */
	uint16_t period = 0;
	uint16_t pulse = 0;
	uint16_t duration = 0;
	uint16_t voltage = 0;

	if(size == 11){
		decode_voltage_message(data, &voltage);
	}else if(size == 15){
		decode_pwm_message(data, &period, &pulse, &duration);
	}

	/* set pwm to led */
	ret = set_pwm_to_led(period, pulse, duration);
	if(ret<0){
		printk("do not set pwm\n");
		return;
	}
}