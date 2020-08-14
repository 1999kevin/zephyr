/*
 * Copyright (c) 2017, embedjournal.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/pinmux.h>
#include <sys/sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for STM32_MIN_DEV board */
static const struct pin_config pinconf[] = {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart1), okay) && CONFIG_SERIAL
	{STM32_PIN_PA9,  STM32F1_PINMUX_FUNC_PA9_USART1_TX},
	{STM32_PIN_PA10, STM32F1_PINMUX_FUNC_PA10_USART1_RX},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart2), okay) && CONFIG_SERIAL
	{STM32_PIN_PA2, STM32F1_PINMUX_FUNC_PA2_USART2_TX},
	{STM32_PIN_PA3, STM32F1_PINMUX_FUNC_PA3_USART2_RX},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart3), okay) && CONFIG_SERIAL
	{STM32_PIN_PB10, STM32F1_PINMUX_FUNC_PB10_USART3_TX},
	{STM32_PIN_PB11, STM32F1_PINMUX_FUNC_PB11_USART3_RX},
#endif

// #if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay) && CONFIG_I2C
// 	{STM32_PIN_PB6, STM32F1_PINMUX_FUNC_PB6_I2C1_SCL},
// 	{STM32_PIN_PB7, STM32F1_PINMUX_FUNC_PB7_I2C1_SDA},
// #endif

// #if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay) && CONFIG_I2C
// 	{STM32_PIN_PB10, STM32F1_PINMUX_FUNC_PB10_I2C2_SCL},
// 	{STM32_PIN_PB11, STM32F1_PINMUX_FUNC_PB11_I2C2_SDA},
// #endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm1), okay) && CONFIG_PWM
	{STM32_PIN_PA8, STM32F1_PINMUX_FUNC_PA8_PWM1_CH1},
	// {STM32_PIN_PA9, STM32F1_PINMUX_FUNC_PA9_PWM1_CH2},
	// {STM32_PIN_PA10, STM32F1_PINMUX_FUNC_PA10_PWM1_CH3},
	// {STM32_PIN_PA11, STM32F1_PINMUX_FUNC_PA11_PWM1_CH4},
	// {STM32_PIN_PB10, STM32F1_PINMUX_FUNC_PB10_PWM1_CH3},
	// {STM32_PIN_PB11, STM32F1_PINMUX_FUNC_PB11_PWM1_CH4},
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm2), okay) && CONFIG_PWM
	// {STM32_PIN_PA8, STM32F1_PINMUX_FUNC_PA8_PWM1_CH1},
	// {STM32_PIN_PA9, STM32F1_PINMUX_FUNC_PA9_PWM1_CH2},
	// {STM32_PIN_PA10, STM32F1_PINMUX_FUNC_PA10_PWM1_CH3},
	// {STM32_PIN_PA11, STM32F1_PINMUX_FUNC_PA11_PWM1_CH4},
	{STM32_PIN_PB10, STM32F1_PINMUX_FUNC_PB10_PWM2_CH3},
	{STM32_PIN_PB11, STM32F1_PINMUX_FUNC_PB11_PWM2_CH4},
	// {STM32_PIN_PA2, STM32F1_PINMUX_FUNC_PA2_PWM2_CH3},
	// {STM32_PIN_PA3, STM32F1_PINMUX_FUNC_PA3_PWM2_CH4},
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm4), okay) && CONFIG_PWM
	// {STM32_PIN_PA8, STM32F1_PINMUX_FUNC_PA8_PWM2_CH1},
	// {STM32_PIN_PA9, STM32F1_PINMUX_FUNC_PA9_PWM1_CH2},
	// {STM32_PIN_PA10, STM32F1_PINMUX_FUNC_PA10_PWM1_CH3},
	// {STM32_PIN_PA11, STM32F1_PINMUX_FUNC_PA11_PWM1_CH4},
	{STM32_PIN_PB6, STM32F1_PINMUX_FUNC_PB6_PWM4_CH1},
	{STM32_PIN_PB7, STM32F1_PINMUX_FUNC_PB7_PWM4_CH2},
#endif

// #if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay) && CONFIG_SPI
// #ifdef CONFIG_SPI_STM32_USE_HW_SS
// 	{STM32_PIN_PA4, STM32F1_PINMUX_FUNC_PA4_SPI1_MASTER_NSS_OE},
// #endif /* CONFIG_SPI_STM32_USE_HW_SS */
// 	{STM32_PIN_PA5, STM32F1_PINMUX_FUNC_PA5_SPI1_MASTER_SCK},
// 	{STM32_PIN_PA6, STM32F1_PINMUX_FUNC_PA6_SPI1_MASTER_MISO},
// 	{STM32_PIN_PA7, STM32F1_PINMUX_FUNC_PA7_SPI1_MASTER_MOSI},
// #endif
// #if DT_NODE_HAS_STATUS(DT_NODELABEL(spi2), okay) && CONFIG_SPI
// #ifdef CONFIG_SPI_STM32_USE_HW_SS
// 	{STM32_PIN_PB12, STM32F1_PINMUX_FUNC_PB12_SPI2_MASTER_NSS_OE},
// #endif /* CONFIG_SPI_STM32_USE_HW_SS */
// 	{STM32_PIN_PB13, STM32F1_PINMUX_FUNC_PB13_SPI2_MASTER_SCK},
// 	{STM32_PIN_PB14, STM32F1_PINMUX_FUNC_PB14_SPI2_MASTER_MISO},
// 	{STM32_PIN_PB15, STM32F1_PINMUX_FUNC_PB15_SPI2_MASTER_MOSI},
// #endif
#ifdef CONFIG_USB_DC_STM32
	{STM32_PIN_PA11, STM32F1_PINMUX_FUNC_PA11_USB_DM},
	{STM32_PIN_PA12, STM32F1_PINMUX_FUNC_PA12_USB_DP},
#endif /* CONFIG_USB_DC_STM32 */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc1), okay) && CONFIG_ADC
	{STM32_PIN_PA0, STM32F1_PINMUX_FUNC_PA0_ADC123_IN0},
	{STM32_PIN_PA1, STM32F1_PINMUX_FUNC_PA1_ADC123_IN1},
#endif
};

static int pinmux_stm32_init(struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1, CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
