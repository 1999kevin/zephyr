#ifndef __FUNCTION_H_
#define __FUNCTION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <drivers/pwm.h>
#include "BT.h"

#define FLAGS	0
#define PWM_FLAGS	0
#define PIN_HIGH 1
#define PIN_LOW 0 

/* GPIOA */
#define BLUETOOTH_SLEEP_PIN 7
#define BLUETOOTH_LINK_STATE_PIN 6
#define BLUETOOTH_MODE_PIN 5

/* GPIOB */
#define SYS_ON_PIN 13
#define LED_POWER_ON_PIN 14

extern struct device *dev_GPIOA;
extern struct device *dev_GPIOB;
extern struct device *dev_UART2; 
extern struct device *dev_ADC1;

extern uint16_t adc_buffer[];

extern const struct adc_sequence sequence;



void PrintFloat(float value);
int compose_message(struct voltage_message *msg, uint16_t voltage);
void voltage_message_format_convert(struct voltage_message *msg, uint8_t *buf);
int system_init(void);
bool bluetooth_is_connected(void);
float get_voltage(void);
void transmit_voltage_message(int voltage);
void set_led_power_on(void);
void decode_pwm_message(uint8_t* data, uint16_t *period,uint16_t *pulse, uint16_t *duration);
void decode_voltage_message(uint8_t* data, uint16_t *voltage);
int set_pwm_to_led(uint16_t period, uint16_t pulse, uint16_t duration);
int send_inst_to_bluetooth(uint8_t *msg,int size);




#ifdef __cplusplus
}
#endif

#endif /* __FUNCTION_H_ */