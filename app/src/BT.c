#include "BT.h"

struct device *dev;

uint8_t read_in_buf[100]={0};


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

	return 0;

}

void printk_buf(struct read_in_buffer buffer){
	printk("buffer content:");
	for(int i=0;i<buffer.size;i++){
		printk("0x%02x ",buffer.buf[i]);
	}
	printk("\n");
}

void uart_fifo_callback(struct device *dev){
	// uart_irq_update(dev);

	// if (!uart_irq_rx_ready(dev)) {
	// 	return;
	// }
	// int i=0;
	// printk("ready to read\n");

	uint32_t start_time;
	uint32_t stop_time;
	uint32_t cycles_spent;
	uint32_t nanoseconds_spent;

	// start_time = k_cycle_get_32();
	while(uart_irq_update(dev) && uart_irq_is_pending(dev)){
		if (!uart_irq_rx_ready(dev)){
			continue;
		}
		else{
			start_time = k_cycle_get_32();
			buf.size += uart_fifo_read(dev, &buf.buf[buf.size], 100);
			stop_time = k_cycle_get_32();
			printk("buf.size:%d\n",buf.size);
			printk("reading value: %d\n",buf.buf[buf.size]);
			// int rx = uart_poll_in(dev,&buf.buf[buf.size]);
			// printk("rx:%d\n",rx);
			// buf.size++;
			// k_msleep(50);
			// if (buf.buf[0] != 27){
			// 	clear_voltage_buf();
			// }
			// if(buf.size>=2 && buf.buf[buf.size-1]==177 && buf.buf[buf.size-2]==177){
			// 	int ret=ring_buf_put(&telegram_queue.rb,buf.buf,buf.size);
			// 	// printk("enqueue:%d\n",ret);
			// 	clear_voltage_buf();
			// }
		}

		// printk("while1\n");
		// k_usleep(1);
	}

	// printk("callback end\n");
	// printk_buf(buf);
	// int UART_REC_BUF_MAX=10;
	// // if ( uart_irq_update(dev) && uart_irq_rx_ready(dev) ){
	// while (uart_irq_update(dev) && uart_irq_rx_ready(dev) ) {                    
	// 	buf.size += uart_fifo_read(dev, &buf.buf[buf.size], UART_REC_BUF_MAX);    
	// 	printk("buf.size:%d\n",buf.size);    
	// 	printk("while1\n");        
	// }  
	// }
	// printk_buf(buf);

	cycles_spent = stop_time - start_time;
	nanoseconds_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent);
	printk("time in ns:%d\n",nanoseconds_spent);

	if(buf.size>20){
		clear_voltage_buf();
	}
}


void clear_voltage_buf(void){
	buf.size = 0;
}


int is_telegram_correct(uint8_t *data, uint32_t size){
	// struct voltage_message msg;
	int len;
	uint8_t check_value;
	if (size == 11){
		len = data[2]*16*16+data[3];
		if (len != size){
			printk("voltage message size error\n");
			return -1;
		}
		check_value = data[2]*16*16+data[3]+data[4]*16*16+data[5]+data[6]*16*16+data[7];
		if(check_value == data[8]){
			return size;
		}else{
			printk("voltage message check value error\n");
		}
	}else if (size == 15){
		len = data[2]*16*16+data[3];
		if (len != size){
			printk("pwm message size error\n");
			return -1;
		}
		check_value = data[2]*16*16+data[3]+data[4]*16*16+data[5]+data[6]*16*16+data[7]+data[8]*16*16+data[9]+data[10]*16*16+data[11];
		if(check_value == data[12]){
			return size;
		}else{
			printk("pwm message check value error\n");
		}
	}
	return -1;
}


int pull_one_message(uint8_t *data){
	// uint8_t *ret_data;
	uint32_t size=0;
	while(!ring_buf_is_empty(&telegram_queue.rb)){
		printk("try to get message\n");
		size+=ring_buf_get(&telegram_queue.rb,&data[size],1);
		printk("size:%d\n",size);
		if (size>2 && data[size-1]==177 && data[size-2]==177){
			break;
		}
	}
	printk("data[size-1]:%d\n",data[size-1]);
	if (size>2 && data[size-1]==177 && data[size-2]==177){
		return is_telegram_correct(data,size);
	}else{
		printk("no complete message in queue\n");
		return -1;
	}
}


void uart_poll_out_multi(struct device *dev, unsigned char* str, int len){
	for (int i=0; i<len;i++){
		uart_poll_out(dev, *str++);
	}
}