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
	cph_deca_init();

	system_interrupt_enable_global();

#ifdef DEBUG_RECEIVE

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


#else
	uint32_t count = 0;
	uint32_t delay_time = 0;
	buffer[0] = 'C';
	buffer[1] = 'P';
	buffer[2] = 'H';
	while (1) {
		cph_millis_delay(1000);
		print_status(count++);
		_burst_count = BURST_COUNT;
		dwt_writetxdata(3, buffer, 0);
		dwt_writetxfctrl34, 0);
		if (count > 1) {
			delay_time = dwt_readsystimestamphi32();
			delay_time += +0x17CDC00;	// 100ms
			dwt_setdelayedtrxtime(delay_time);
			dwt_starttx(DWT_START_TX_DELAYED);
		} else {
			dwt_starttx(DWT_START_TX_IMMEDIATE);
		}
		port_pin_set_output_level(LED_PIN, false);
	}
#endif

}
