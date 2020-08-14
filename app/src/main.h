#ifndef __MAIN_H_
#define __MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/adc.h>
#include <sys/printk.h>
#include <device.h>

void PrintFloat(float value);
int compose_message(struct voltage_message *msg, uint16_t voltage);
void voltage_message_format_convert(struct voltage_message *msg, uint8_t *buf);

static const struct adc_channel_cfg adc_ch_cfg = {
	.gain  = ADC_GAIN_1,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = 1,
	.differential  = 0
};

uint16_t buffer[100];

const struct adc_sequence sequence = {
	.options = NULL,
	.channels = BIT(1),
	.buffer = buffer,
	.buffer_size = 10,
	.resolution = 12
};

// struct voltage_message {
// 	/* data */
// 	uint16_t SOF;
// 	uint16_t len;
// 	uint16_t cmd;
// 	uint16_t data;
// 	uint8_t crc;
// 	uint16_t EOF;
// };



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H_ */