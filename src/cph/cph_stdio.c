/*
 * cph_stdio.c
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#include <cph_stdio.h>


static struct usart_module usart_instance;


void cph_stdio_init(void) {

	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.baudrate = STDIO_BAUD;
	config_usart.mux_setting = STDIO_MUX;
	config_usart.pinmux_pad0 = STDIO_PAD0;
	config_usart.pinmux_pad1 = STDIO_PAD1;
	config_usart.pinmux_pad2 = STDIO_PAD2;
	config_usart.pinmux_pad3 = STDIO_PAD3;

	stdio_serial_init(&usart_instance, STDIO_HW, &config_usart);
	usart_enable(&usart_instance);
}


