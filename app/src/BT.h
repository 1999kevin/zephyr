#ifndef __BT_H_
#define __BT_H_

#include <device.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>


#ifdef __cplusplus
extern "C" {
#endif

#define MY_RING_BUF_SIZE 64
#define MAX_LINE_LENGTH 128

struct voltage_message {
	/* data */
	uint16_t SOF;
	uint16_t len;
	uint16_t cmd;
	uint16_t data;
	uint8_t crc;
	uint16_t EOF;
};

struct pwm_message {
	/* data */
	uint16_t SOF;
	uint16_t len;
	uint16_t cmd;
	uint16_t period; //in unit of us
	uint16_t pulse;
	uint16_t duration;
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

extern struct device *dev;
extern uint8_t read_in_buf[];
extern struct read_in_buffer buf;
extern struct ring_buf_container telegram_queue;


void uart_fifo_callback(struct device *dev);
int uart_fifo_init(void);
void printk_buf_hex(struct read_in_buffer buffer);
void printk_buf_str(struct read_in_buffer buffer);
int voltage_telegram_ready(struct read_in_buffer buffer);
void clear_voltage_buf(void);
int pull_one_message(uint8_t *data);
int is_telegram_correct(uint8_t *data, uint32_t size);
void uart_poll_out_multi(struct device *dev, unsigned char* str, int len);

#ifdef __cplusplus
}
#endif

#endif /* __BT_H_ */