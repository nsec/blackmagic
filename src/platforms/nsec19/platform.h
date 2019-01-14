/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"

// #define PLATFORM_HAS_TRACESWO
// #define PLATFORM_HAS_POWER_SWITCH
#ifdef ENABLE_DEBUG
#define PLATFORM_HAS_DEBUG
#define USBUART_DEBUG
#endif
#define BOARD_IDENT             "NorthSec 2019 Probe"
#define BOARD_IDENT_DFU	        "NorthSec 2019 Probe (Upgrade)"
#define BOARD_IDENT_UPD	        "NorthSec 2019 Probe (DFU Upgrade)"
#define DFU_IDENT               "NorthSec 2019 Firmware Upgrade"
#define DFU_IFACE_STRING        "@Internal Flash   /0x08000000/8*001Ka,120*001Kg"
#define UPD_IFACE_STRING        "@Internal Flash   /0x08000000/8*001Kg"
#define PLATFORM_HAS_USBUART

/* Important pin mappings for NorthSec badge implementation:
 * uC is a STM32F070F6P6.
 *
 * nFR52 (internal) SWDIO/TMS: PA5
 * nFR52 (internal) SWCLK/TCK: PA6
 * nFR52 (internal) nRST: PA7
 *
 * External SWDIO/TMS: PB12
 * External SWCLK/TCK: PB15
 * External nRST: PB2
 *
 * USB cable -: PA11
 * USB cable +: PA12
 */

/* Hardware definitions... */
#define JTAG_PORT 	GPIOA
#define TDI_PORT	JTAG_PORT
#define TMS_DIR_PORT	JTAG_PORT
#define TMS_PORT	JTAG_PORT
#define TCK_PORT	JTAG_PORT
#define TDO_PORT	JTAG_PORT
// #define TDI_PIN		GPIO3
#ifdef NSEC_BADGE_USE_EXTERNAL_PINS
#define TMS_DIR_PORT	GPIOB
#define TMS_DIR_PIN	GPIO1
#define TMS_PIN		GPIO4
#define TCK_PIN		GPIO5
#else
#define TMS_DIR_PORT	JTAG_PORT
#define TMS_DIR_PIN	GPIO4
#define TMS_PIN		GPIO5
#define TCK_PIN		GPIO6
#endif

// #define TDO_PIN		GPIO6

#define SWDIO_DIR_PORT	JTAG_PORT
#define SWDIO_PORT 	JTAG_PORT
#define SWCLK_PORT 	JTAG_PORT
#define SWDIO_DIR_PIN	TMS_DIR_PIN
#define SWDIO_PIN	TMS_PIN
#define SWCLK_PIN	TCK_PIN

#define SRST_PORT	GPIOA
#define SRST_PIN	GPIO7
#define SRST_SENSE_PORT	GPIOA
#define SRST_SENSE_PIN	GPIO7

#define TMS_SET_MODE() do { \
	gpio_set(TMS_DIR_PORT, TMS_DIR_PIN); \
	gpio_mode_setup(TMS_DIR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, TMS_PIN); \
	gpio_set_output_options(TMS_DIR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, \
			TMS_PIN); \
} while(0)

#define SWDIO_MODE_FLOAT() do { \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SWDIO_PIN); \
	gpio_clear(SWDIO_DIR_PORT, SWDIO_DIR_PIN); \
} while(0)

#define SWDIO_MODE_DRIVE() do { \
	gpio_set(SWDIO_DIR_PORT, SWDIO_DIR_PIN); \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, SWDIO_PIN); \
	gpio_set_output_options(SWDIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, \
			SWDIO_PIN); \
} while(0)

#define USB_DRIVER st_usbfs_v2_usb_driver
#define USB_IRQ    NVIC_USB_IRQ
#define USB_ISR    usb_isr
/* Interrupt priorities.  Low numbers are high priority.
 * For now USART1 preempts USB which may spin while buffer is drained.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB             (2 << 6)
#define IRQ_PRI_USBUSART	(1 << 4)
#define IRQ_PRI_USBUSART_TIM	(3 << 4)
#define IRQ_PRI_TRACE           (0 << 5)

#define USBUSART USART2
#define USBUSART_CR1 USART2_CR1
#define USBUSART_IRQ NVIC_USART2_IRQ
#define USBUSART_CLK RCC_USART2
#define USBUSART_TX_PORT GPIOA
#define USBUSART_TX_PIN  GPIO2
#define USBUSART_RX_PORT GPIOA
#define USBUSART_RX_PIN  GPIO9
#define USBUSART_ISR usart2_isr
#define USBUSART_TIM TIM7
#define USBUSART_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM7)
#define USBUSART_TIM_IRQ NVIC_TIM7_IRQ
#define USBUSART_TIM_ISR tim7_isr

#define UART_PIN_SETUP() do { \
	gpio_mode_setup(USBUSART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, \
	                USBUSART_TX_PIN); \
	gpio_mode_setup(USBUSART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, \
	                USBUSART_RX_PIN); \
	gpio_set_af(USBUSART_TX_PORT, GPIO_AF1, USBUSART_TX_PIN); \
	gpio_set_af(USBUSART_RX_PORT, GPIO_AF1, USBUSART_RX_PIN); \
    } while(0)

#define TRACE_TIM TIM3
#define TRACE_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM3)
#define TRACE_IRQ   NVIC_TIM3_IRQ
#define TRACE_ISR   tim3_isr

#ifdef ENABLE_DEBUG
extern bool debug_bmp;
int usbuart_debug_write(const char *buf, size_t len);

#define DEBUG printf
#else
#define DEBUG(...)
#endif

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)
#define SET_ERROR_STATE(state)

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define snprintf sniprintf
#define vasprintf vasiprintf

#endif
