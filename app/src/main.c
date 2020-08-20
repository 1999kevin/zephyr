
#include "function.h"

void main(void)
{

	printk("here\n");
	// bool led_is_on = true;
	int ret;

	/* initiate system, bluetooth and ring buf */
	ret = system_init();
	if(ret < 0){
		return;
	}

	// while(1){
	// 	uint8_t* msg = "AT:MAC?\r\n";
	// 	uart_poll_out_multi(dev_UART2, msg, 9);
	// 	k_msleep(1000);
	// 	printk_buf_str(buf);

	// }
	
	/* wait for bluetooth to connect */
	while(!bluetooth_is_connected()){
		printk("waiting for bluetooth to connect\n");
		k_msleep(1000);
	}
	printk("bluetooth connected\n");
	k_msleep(5000);


	/* obtain voltage message */
	float voltage_result = get_voltage();
	if(voltage_result<0){
		return;
	}

	/* transmit voltage message */
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

	if(size == VOLTAGE_MESSAGE_LENGTH){
		decode_voltage_message(data, &voltage);
	}else if(size == PWM_MESSAGE_LENGTH){
		decode_pwm_message(data, &period, &pulse, &duration);
	}

	/* set pwm to led */
	ret = set_pwm_to_led(period, pulse, duration);
	if(ret<0){
		printk("do not set pwm\n");
		return;
	}
}