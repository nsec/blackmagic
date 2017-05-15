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

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"
#include "morse.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>

/* 
 * 2017 - nsec17 badge
 */
int platform_hwversion(void)
{
	return 2017;
}

void platform_init(void)
{
	SCS_DEMCR |= SCS_DEMCR_VC_MON_EN;
#ifdef ENABLE_DEBUG
	void initialise_monitor_handles(void);
	initialise_monitor_handles();
#endif

	rcc_clock_setup_in_hsi48_out_48mhz();

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_SYSCFG_COMP);
	rcc_periph_clock_enable(RCC_CRC);
	rcc_set_usbclk_source(RCC_HSI48);

	/* Setup GPIO ports */
	//gpio_clear(USB_PU_PORT, USB_PU_PIN);
	//gpio_mode_setup(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
	//		USB_PU_PIN);

	//gpio_set_mode(JTAG_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	//		GPIO_CNF_OUTPUT_PUSHPULL,
	//		TMS_DIR_PIN | TMS_PIN | TCK_PIN | TDI_PIN);
	gpio_mode_setup(JTAG_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
			TMS_DIR_PIN | TMS_PIN | TCK_PIN);
	gpio_set_output_options(JTAG_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
			TMS_DIR_PIN | TMS_PIN | TCK_PIN);
	/* This needs some fixing... */
	/* Toggle required to sort out line drivers... */
	//gpio_port_write(GPIOA, 0x8102);
	//gpio_port_write(GPIOB, 0x2000);

	//gpio_port_write(GPIOA, 0x8182);
	//gpio_port_write(GPIOB, 0x2002);

	//gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
	//		GPIO_CNF_OUTPUT_PUSHPULL,
	//		LED_UART | LED_IDLE_RUN | LED_ERROR);

	/* FIXME: This pin in intended to be input, but the TXS0108 fails
	 * to release the device from reset if this floats. */
	//gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
	//		GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
	/* Enable SRST output. Original uses a NPN to pull down, so setting the
	 * output HIGH asserts. Mini is directly connected so use open drain output
	 * and set LOW to assert.
	 */
	//platform_srst_set_val(false);
	//gpio_set_mode(SRST_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	//		(((platform_hwversion() == 0) ||
	//		  (platform_hwversion() >= 3))
	//		 ? GPIO_CNF_OUTPUT_PUSHPULL
	//		 : GPIO_CNF_OUTPUT_OPENDRAIN),
	//		SRST_PIN);

	/* Enable internal pull-up on PWR_BR so that we don't drive
	   TPWR locally or inadvertently supply power to the target. */
	//if (platform_hwversion () == 1) {
	//	gpio_set(PWR_BR_PORT, PWR_BR_PIN);
	//	gpio_set_mode(PWR_BR_PORT, GPIO_MODE_INPUT,
	//	              GPIO_CNF_INPUT_PULL_UPDOWN, PWR_BR_PIN);
	//} else if (platform_hwversion() > 1) {
	//	gpio_set(PWR_BR_PORT, PWR_BR_PIN);
	//	gpio_set_mode(PWR_BR_PORT, GPIO_MODE_OUTPUT_50_MHZ,
	//	              GPIO_CNF_OUTPUT_OPENDRAIN, PWR_BR_PIN);
	//}

	platform_timing_init();
	cdcacm_init();
}

void platform_srst_set_val(bool assert)
{
	if ((platform_hwversion() == 0) ||
	    (platform_hwversion() >= 3)) {
		gpio_set_val(SRST_PORT, SRST_PIN, assert);
	} else {
		gpio_set_val(SRST_PORT, SRST_PIN, !assert);
	}
	if (assert) {
		for(int i = 0; i < 10000; i++) asm("nop");
	}
}

bool platform_srst_get_val(void)
{
	if (platform_hwversion() == 0) {
		return gpio_get(SRST_SENSE_PORT, SRST_SENSE_PIN) == 0;
	} else if (platform_hwversion() >= 3) {
		return gpio_get(SRST_SENSE_PORT, SRST_SENSE_PIN) != 0;
	} else {
		return gpio_get(SRST_PORT, SRST_PIN) == 0;
	}
}

bool platform_target_get_power(void)
{
	return 0;
}

void platform_target_set_power(bool power)
{
    (void) power;
}

const char *platform_target_voltage(void)
{
	return "Unsupported";
}

void platform_request_boot(void)
{
	/* Disconnect USB cable */
	//gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, 0, USB_PU_PIN);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO11);

	/* Drive boot request pin */
	// gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
	// 		GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	// gpio_clear(GPIOB, GPIO12);
	// gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPI012);
	// gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO12);
	// gpio_clear(GPIOA, GPIO12);
}

#ifdef ENABLE_DEBUG
enum {
	RDI_SYS_OPEN = 0x01,
	RDI_SYS_WRITE = 0x05,
	RDI_SYS_ISTTY = 0x09,
};

int rdi_write(int fn, const char *buf, size_t len)
{
	(void)fn;
	if (debug_bmp)
		return len - usbuart_debug_write(buf, len);

	return 0;
}

struct ex_frame {
	union {
		int syscall;
		int retval;
	};
	const int *params;
	uint32_t r2, r3, r12, lr, pc;
};

void debug_monitor_handler_c(struct ex_frame *sp)
{
	/* Return to after breakpoint instruction */
	sp->pc += 2;

	switch (sp->syscall) {
	case RDI_SYS_OPEN:
		sp->retval = 1;
		break;
	case RDI_SYS_WRITE:
		sp->retval = rdi_write(sp->params[0], (void*)sp->params[1], sp->params[2]);
		break;
	case RDI_SYS_ISTTY:
		sp->retval = 1;
		break;
	default:
		sp->retval = -1;
	}

}

asm(".globl debug_monitor_handler\n"
    ".thumb_func\n"
    "debug_monitor_handler: \n"
    "    mov r0, sp\n"
    "    b debug_monitor_handler_c\n");

#endif
