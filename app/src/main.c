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
#include "BT.h"


// uint8_t* buf = "deliver message\n";

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


void uart_fifo_callback(struct device *dev);
int uart_fifo_init(void);
void printk_buf(struct read_in_buffer buffer);


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
		}

		printk("while1\n");
	}

	// printk("callback end\n");
	// printk_buf(buf);
	int UART_REC_BUF_MAX=10;
	// // if ( uart_irq_update(dev) && uart_irq_rx_ready(dev) ){
	while (uart_irq_update(dev) && uart_irq_rx_ready(dev) ) {                    
		buf.size += uart_fifo_read(dev, &buf.buf[buf.size], UART_REC_BUF_MAX);    
		printk("buf.size:%d\n",buf.size);    
		printk("while1\n");        
	}  
	// }
	printk_buf(buf);
}

// static void uart4_isr(struct device * nu){    
// 	if ( uart_irq_update(uart4_dev) && uart_irq_rx_ready(uart4_dev) )  {        
// 		if( uart_start_receive_flag == 0 ) {            
// 			uart_clear_received();        
// 		}else{           
// 			if( uart_receive_index < uart_receive_length ) {                   
// 				while (uart_irq_update(uart4_dev) && uart_irq_rx_ready(uart4_dev) ) {                    
// 					uart_receive_index += uart_fifo_read(uart4_dev, &uart_receive_buf[uart_receive_index], UART_REC_BUF_MAX);                
// 				}                       
// 			} else {                
// 				uart_clear_received();           
// 			}       
// 		}   
// 	}
// }


// int voltage_telegram_ready(struct read_in_buffer buffer){
// 	int len = buffer.buf[2]*16*16+buffer.buf[3];
// 	if(buffer.size != len){
// 		printk("invalid telegram in size checking");
// 		return -1;
// 	}
// 	return 0;
// }

void main(void)
{
	uart_fifo_init();
	printk("here\n");

	// voltage_telegram_ready(struct read_in_buffer)

}
