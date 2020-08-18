#include "BT.h"

struct device *dev;

uint8_t read_in_buf[MAX_LINE_LENGTH]={0};


struct read_in_buffer buf = {
	.buf = read_in_buf,
	.size = 0
};

struct ring_buf_container telegram_queue;



const struct uart_config cfg = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};



/* uart_fifo_init（）：
 * This fucntion is used to initate uart2
 */
int uart_fifo_init(void){
	uint8_t c;
	dev = device_get_binding("UART_2");
	if (dev == NULL) {
		printk("cannot find device\n");
		return -1;
	}
	if(uart_configure(dev, &cfg) != 0){
		printk("configure error\n");
	}
	printk("configure succuessfully\n");

	uart_irq_rx_disable(dev);
	uart_irq_tx_disable(dev);
		
	uart_irq_callback_set(dev, uart_fifo_callback);

	/* Drain the fifo */
	if (uart_irq_rx_ready(dev)) {
		while (uart_fifo_read(dev, &c, 1)) {
			;
		}
	}
	uart_irq_rx_enable(dev);

	/* Enable all interrupts unconditionally. Note that this is due
	 * to Zephyr issue #8393. This should be removed once the
	 * issue is fixed in upstream Zephyr.
	 */
	irq_unlock(0);
	// printk("after intial\n");
	return 0;

}


/* printk_buf_hex()
 * This function would printk read_in_buffer in hex format
 */
void printk_buf_hex(struct read_in_buffer buffer){
	printk("buffer content:");
	for(int i=0; i<buffer.size; i++){
		printk("0x%02x ", buffer.buf[i]);
	}
	printk("\n");
}


/* printk_buf_str()
 * This function would printk read_in_buffer in text format
 */
void printk_buf_str(struct read_in_buffer buffer){
	printk("buffer content:");
	for(int i=0; i<buffer.size; i++){
		/* if it is a character */
		if(buffer.buf[i] > 0x1F && buffer.buf[i] < 0x7F){
			printk("%c",buffer.buf[i]);
		}
	}
	printk("\n");
}


/* uart_fifo_callback（）：
 * this function detect uart interrupt and read data into read_in_buf
 * this function should be used after uart_fifo_init() and  ring_buf_init()
 */
void uart_fifo_callback(struct device *dev){
	// printk("111\n");
	while(uart_irq_update(dev) && uart_irq_is_pending(dev)){
		if (!uart_irq_rx_ready(dev)){
			continue;
		}
		else{
			buf.size += uart_fifo_read(dev, &buf.buf[buf.size], MAX_LINE_LENGTH);
			if (buf.size >= 2 && buf.buf[buf.size-1] == SOF_CHAR && buf.buf[buf.size-2] == SOF_CHAR){
				clear_voltage_buf();
				buf.buf[0] = SOF_CHAR;
				buf.buf[1] = SOF_CHAR;
				buf.size = 2;
			}
			if(buf.size >= 2 && buf.buf[buf.size-1] == EOF_CHAR && buf.buf[buf.size-2] == EOF_CHAR){
				int ret=ring_buf_put(&telegram_queue.rb, buf.buf, buf.size);
				printk("enqueue:%d\n",ret);
				clear_voltage_buf();
			}
		}

	}

	if(buf.size>MAX_LINE_LENGTH){
		printk("buf overfit\n");
		clear_voltage_buf();
	}
}

/* this function would clear read_in_buf */
void clear_voltage_buf(void){
	buf.size = 0;
}

/* this function would check whether we have a valid telegram*/
int is_telegram_correct(uint8_t *data, uint32_t size){
	// struct voltage_message msg;
	int len;
	uint8_t check_value;
	if (size == VOLTAGE_MESSAGE_LENGTH){
		len = data[2] * 16 * 16 + data[3];
		if (len != size){
			printk("voltage message size error\n");
			return -1;
		}
		check_value = data[2] * 16 * 16 + data[3]
					 +data[4] * 16 * 16 + data[5]
					 +data[6] * 16 * 16 + data[7];
		if(check_value == data[8]){
			return size;
		}else{
			printk("voltage message check value error\n");
		}
	}else if (size == PWM_MESSAGE_LENGTH){
		len = data[2] * 16 * 16 + data[3];
		if (len != size){
			printk("pwm message size error\n");
			return -1;
		}
		check_value = data[2] * 16 * 16 + data[3]
		             +data[4] * 16 * 16 + data[5]
					 +data[6] * 16 * 16 + data[7]
					 +data[8] * 16 * 16 + data[9]
					 +data[10]* 16 * 16 + data[11];
		if(check_value == data[12]){
			return size;
		}else{
			printk("pwm message check value error\n");
		}
	}
	return -1;
}

/* this function would pull out one message from ring_buf */
int pull_one_message(uint8_t *data){
	// uint8_t *ret_data;
	uint32_t size=0;
	while(!ring_buf_is_empty(&telegram_queue.rb)){
		// printk("try to get message\n");
		size+=ring_buf_get(&telegram_queue.rb,&data[size],1);
		// printk("size:%d\n",size);
		if (size > 2 && data[size-1] == EOF_CHAR && data[size-2] == EOF_CHAR){
			break;
		}
	}
	printk("data[size-1]:%d\n",data[size-1]);
	if (size>2 && data[size-1] == EOF_CHAR && data[size-2] == EOF_CHAR){
		return is_telegram_correct(data,size);
	}else{
		printk("no complete message in queue\n");
		return -1;
	}
}

/* poll out string */
void uart_poll_out_multi(struct device *dev, unsigned char* str, int len){
	for (int i=0; i<len; i++){
		uart_poll_out(dev, *str++);
	}
}