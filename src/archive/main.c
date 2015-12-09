/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <cph.h>
#include <anchor.h>
#include <tag.h>

int ss_twr_init(void);

int main(void) {
	system_init();

	cph_millis_init();
	cph_stdio_init();

	TRACE("DECAWAVE CPH       \r\n");
	TRACE(SOFTWARE_VER_STRING);
	TRACE("\r\n");

	uint32_t f = system_gclk_gen_get_hz(0);
	TRACE("CPU FREQ: %lu\r\n", f);

	// Blink LED for 5 seconds
	for (int i = 0; i < (5 * 4); i++) {
		port_pin_set_output_level(LED_PIN, false);
		cph_millis_delay(125);
		port_pin_set_output_level(LED_PIN, true);
		cph_millis_delay(125);
	}

	// init cph_deca
//#ifdef ANCHOR
//	anchor_init();
//#else
//	tag_init();
//#endif
	ss_twr_init();	// blocks

	system_interrupt_enable_global();

	uint32_t count = 0;
	uint32_t timestamp = cph_get_millis();
	uint32_t elapsed = 0;
	while (1) {
		cph_deca_run();
		elapsed = (cph_get_millis() - timestamp);
		if (elapsed > 5000) {
			timestamp = cph_get_millis();
			cph_deca_print_status(count++);
		}
	}



}
