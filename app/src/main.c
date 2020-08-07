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
#include <drivers/uart.h>
#include <sys/ring_buffer.h>


// uint8_t* buf = "deliver message\n";
#define MY_RING_BUF_SIZE 64

struct voltage_message {
	/* data */
	uint16_t SOF;
	uint16_t len;
	uint16_t cmd;
	uint16_t data;
	uint8_t crc;
	uint16_t EOF;
};

struct read_in_buffer {
	uint8_t *buf;
	int size;
};

struct ring_buf_container {
    struct ring_buf rb;
    uint8_t buffer[MY_RING_BUF_SIZE];

};

void uart_fifo_callback(struct device *dev);
int uart_fifo_init(void);
void printk_buf(struct read_in_buffer buffer);
int voltage_telegram_ready(struct read_in_buffer buffer);
void clear_voltage_buf(void);
int pull_one_message(uint8_t *data);

const struct uart_config cfg = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
};

struct device *dev;

uint8_t read_in_buf[10]={0};

struct read_in_buffer buf = {
	.buf = read_in_buf,
	.size = 0
};

struct ring_buf_container telegram_queue;

int uart_fifo_init(void){
	uint8_t c;
	dev = device_get_binding("UART_2");
	if (dev == NULL) {
		printk("cannot find device\n");
		return -1;
	}
	// if(uart_configure(dev, &cfg) != 0){
	// 	printk("configure error\n");
	// }
	// printk("configure succuessfully\n");

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
	printk("ready to read\n");

	while(uart_irq_update(dev) && uart_irq_is_pending(dev)){
		if (!uart_irq_rx_ready(dev)){
			continue;
		}
		else{
			buf.size += uart_fifo_read(dev, &buf.buf[buf.size], 10);
			printk("buf.size:%d\n",buf.size);
			// int rx = uart_poll_in(dev,&buf.buf[buf.size]);
			// printk("rx:%d\n",rx);
			// buf.size++;
			// k_msleep(50);
			if (buf.buf[0] != 27){
				clear_voltage_buf();
			}
			if(buf.size>=2 && buf.buf[buf.size-1]==177 && buf.buf[buf.size-2]==177){
				int ret=ring_buf_put(&telegram_queue.rb,buf.buf,buf.size);
				printk("enqueue:%d\n",ret);
				clear_voltage_buf();
			}
		}

		printk("while1\n");
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
	printk_buf(buf);

	if(buf.size>20){
		clear_voltage_buf();
	}
}


void clear_voltage_buf(void){
	buf.size = 0;
}


int voltage_telegram_ready(struct read_in_buffer buffer){
	// struct voltage_message msg;
	int len = buffer.buf[2]*16*16+buffer.buf[3];
	if(buffer.size != len){
		printk("invalid telegram in size checking\n");
		return -1;
	}
	int data = buffer.buf[6]*16*16+buffer.buf[7];
	return data;
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
		return size;
	}else{
		printk("no complete message in queue\n");
		return -1;
	}
}

void main(void)
{

	printk("here\n");

	// voltage_telegram_ready(struct read_in_buffer)
    ring_buf_init(&telegram_queue.rb, MY_RING_BUF_SIZE , telegram_queue.buffer);
	uart_fifo_init();

	uint8_t data[20];
	// uint8_t *print_data = data;
	while(1){
		int size = pull_one_message(data);
		printk("ret:%d\n",size);
		if(size>0){
			printk("get data: %d\n",data[0]);
		}else{
			printk("waiting for data, free space:%d\n",ring_buf_space_get(&telegram_queue.rb));
		}
		k_msleep(5000);
	}


	uint8_t *messgae="message";
	uint8_t ret_data[8];
	int ret=ring_buf_put(&telegram_queue.rb,messgae,7);
	printk("enqueue:%d\n",ret);
	ret=ring_buf_get(&telegram_queue.rb,ret_data,7);
	printk("ret:%d\n",ret);
	printk("ret:data:%s\n",ret_data);
}
