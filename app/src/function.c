#include "function.h"

struct device *dev_GPIOA;
struct device *dev_GPIOB;
struct device *dev_UART2;
struct device *dev_ADC1;

static const struct adc_channel_cfg adc_ch_cfg = {
	.gain  = ADC_GAIN_1,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = 1,
	.differential  = 0
};

uint16_t adc_buffer[100] = {0};

const struct adc_sequence sequence = {
	.options = NULL,
	.channels = BIT(1),
	.buffer = adc_buffer,
	.buffer_size = 10,
	.resolution = 12
};


/* system_init():
 * This function is used to intiate the whole system
 * This function will turn on system, bluetooth, ring buf and set bluetooth mode to slave
 */
int system_init(void){
    dev_GPIOA = device_get_binding("GPIOA");
    dev_GPIOB = device_get_binding("GPIOB");
    dev_UART2 = device_get_binding("UART_2");
    dev_ADC1 = device_get_binding("ADC_1");

    int ret;
	if (dev_GPIOB == NULL || dev_GPIOA == NULL || dev_UART2 == NULL || dev_ADC1 == NULL) {
		printk("cannnot find GPIOA or GPIOB or UART2 or ADC1\n");
		return -1;
	}

	/* set system on */
	ret = gpio_pin_configure(dev_GPIOB, 13, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_set(dev_GPIOB, 13, 1);
	printk("system on status: %d\n", ret);
	
	/* set bluetooth working */
	ret = gpio_pin_configure(dev_GPIOA, 7, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_set(dev_GPIOA, 7, 0);
	printk("bluetooth status:%d\n",ret);

	/* set bluetooth slave mode */
	ret = gpio_pin_configure(dev_GPIOA, 5, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return -1;
	}
	ret = gpio_pin_set(dev_GPIOA, 5, 1);
	printk("bluetooth mode:%d\n",ret);
    return 0;

	/* initiate ring buf */
    ring_buf_init(&telegram_queue.rb, MY_RING_BUF_SIZE , telegram_queue.buffer);
	uart_fifo_init();

}

/* bluetooth_is_connected():
 * this function would check whether bluetooth is connected
 */
bool bluetooth_is_connected(void){
	if(dev_GPIOA == NULL){
		printk("not GPIOA found\n");
		return false;
	}
	int ret = gpio_pin_get(dev_GPIOA, 6);
	if(ret == 0){
		return true;
	}
	return false;
}



/* get_voltage():
 * This function will return the voltage from ADC_1, channel 1, which is PA1
 */
float get_voltage(void){
    int ret;
    float result = 0;
	ret = adc_channel_setup(dev_ADC1, &adc_ch_cfg);
	// printk("here2\n");
	if (ret != 0) {
		printk("Setting up of adc channel failed with code %d\n", ret);
		return -1;
	}
	if(adc_read(dev_ADC1,&sequence) < 0){
		printk("cannot read value\n");
        return -1;
	}else{
        // result = 
		result = adc_buffer[0]/4096.0*3300.0*2.0;
		// printk("result:%d\n",(int)result);
		printk("voltage:\n");
		PrintFloat(result);
		printk("mV\n");
	}
    return result;
}

/* transmit_voltage_message():
 * This function would transfer voltage data to telegraph format 
 * and send it through bluetooth.
 * Before using this function, you should check bluetooth is connected.
 */
void transmit_voltage_message(int voltage){
    struct voltage_message msg;
	compose_message(&msg,voltage);

	unsigned char voltage_buf[15]={0};
	voltage_message_format_convert(&msg,voltage_buf);
	uart_poll_out_multi(dev_UART2,voltage_buf, msg.len);
	printk("send out telegraph\n");
}


/* set_led_power_on():
 * This function would turn led power on 
 */
void set_led_power_on(void){
    int ret;
    ret = gpio_pin_configure(dev_GPIOB, 14, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_set(dev_GPIOB, 14, 1);
	printk("led power on status: %d\n", ret);
}


/* PrintFloat():
 * This function would help to printk float data. 
 */

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


/* decode_pwm_message():
 * This function will get period, pulse and duration from pointer data
 */ 
void decode_pwm_message(uint8_t* data, uint16_t *period,uint16_t *pulse, uint16_t *duration){
    printk("get a pwm message\n");
	*period = data[6]*16*16+data[7];
	*pulse = data[8]*16*16+data[9];
	*duration = data[10]*16*16+data[11];
    printk("period: %d, pulse: %d, duration: %d\n", *period, *pulse, *duration);
}


/* decode_voltage_message():
 * This function will get voltage from pointer data
 */ 
void decode_voltage_message(uint8_t* data, uint16_t *voltage){
    printk("get a voltage message\n");
	*voltage = data[6]*16*16+data[7];
	printk("voltage:%d\n", *voltage);
}



/* set_pwm_to_led():
 * This function would set pwm with periof and pulse to T4C1,T4C2,T2C3,T2C4
 */ 
int set_pwm_to_led(uint16_t period, uint16_t pulse, uint16_t duration){
    int ret;
    const char* label4 = "PWM_4";
	struct device *dev_pwm4 = device_get_binding(label4);
	if (!dev_pwm4) {
	    printk("Error: didn't find %s device\n", label4);
		return -1;
	}
	printk("%s correct\n", label4);

	// uint64_t cycles;
	// ret = pwm_get_cycles_per_sec(dev_pwm4,1, &cycles);
	// printk("clock rate: %lld\n",cycles);
	ret = pwm_pin_set_usec(dev_pwm4,1, period, pulse, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
        return -1;
	}
	printk("set %s,channel 1, successfully\n",label4);
	ret = pwm_pin_set_usec(dev_pwm4,2, period, pulse, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
        return -1;
	}
	printk("set %s,channel 2, successfully\n",label4);

    const char* label2 = "PWM_2";
	struct device *dev_pwm2 = device_get_binding(label2);
	if (!dev_pwm2) {
		printk("Error: didn't find %s device\n", label2);
		return -1;
	}
	printk("%s correct\n", label2);

	ret = pwm_pin_set_usec(dev_pwm2,3, period, pulse, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
        return -1;
	}
	printk("set %s,channel 3, successfully\n",label2);
	ret = pwm_pin_set_usec(dev_pwm2,4, period, pulse, PWM_FLAGS);
	if(ret < 0){
		printk("error %d\n",ret);
        return -1;
	}
	printk("set %s,channel 4, successfully\n",label2);
    return 0;
}

/* voltage_message_format_convert():
 * This function would convert data in msg to buf, which can be transmitted by bluetooth
 */ 
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


/* compose_message():
 * This function would compose a telegram that contain voltage information
 */
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

/* send_inst_to_bluetooth():
 * This function will send a message to bluetooth and printk the result which is stored in ring_buf
 */

int send_inst_to_bluetooth(uint8_t *msg,int size){
	if(dev_UART2 != NULL){
		return -1;
	}
	uart_poll_out_multi(dev_UART2, msg, size);
	k_msleep(100);
	printk_buf_str(buf);
	return 0;
}



