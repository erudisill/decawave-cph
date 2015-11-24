/*
 * cph_millis.c
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#include <cph_millis.h>

volatile uint32_t g_cph_millis = 0;

void SysTick_Handler(void) {
	g_cph_millis++;
}

void cph_millis_init(void) {
	SysTick_Config(system_cpu_clock_get_hz() / 1000);
}

void cph_millis_delay(uint32_t millis) {
	uint32_t current;
	current = g_cph_millis;
	while ((g_cph_millis - current) < millis)
		;
}
