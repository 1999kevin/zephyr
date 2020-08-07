#ifndef __BT_H_
#define __BT_H_

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif /* __BT_H_ */