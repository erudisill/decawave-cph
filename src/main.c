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
#include <cph_deca_port.h>

int main(void) {
	system_init();

	cph_millis_init();
	cph_stdio_init();

	TRACE("%s\r\n", APP_NAME);

	uint32_t f = system_gclk_gen_get_hz(0);
	TRACE("CPU FREQ: %lu\r\n", f);

	// Blink LED for 5 seconds
	for (int i = 0; i < (5 * 4); i++) {
		port_pin_set_output_level(LED_PIN, false);
		cph_millis_delay(125);
		port_pin_set_output_level(LED_PIN, true);
		cph_millis_delay(125);
	}

    // Start with board specific hardware init.
	cph_deca_init_gpio();
	cph_deca_spi_init();

	system_interrupt_enable_global();

	app_run();
}
