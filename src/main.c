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
#include <cph_deca.h>

cph_config_t * cph_config;

#ifdef ANCHOR
int cph_mode = CPH_MODE_ANCHOR;
#else
int cph_mode = CPH_MODE_TAG;
#endif

uint16_t cph_coordid = 0;

volatile uint32_t cph_signal = 0x00;

struct usart_module * usart_instance;
uint16_t rx_char = 0x00;

static void init_config(void) {
	bool do_reset = false;

	cph_config = (cph_config_t*)cph_config_init();

	// If no magic, first run.
	if (cph_config->magic[0] != 'C' || cph_config->magic[1] != 'P' || cph_config->magic[2] != 'H' || cph_config->magic[3] != 'T') {
		do_reset = true;
	}
	// Not the first run, but if FW versions don't match, reset
	else if (cph_config->fw_major != FW_MAJOR || cph_config->fw_minor != FW_MINOR) {
		do_reset = true;
	}

	if (do_reset) {
		cph_config->magic[0] = 'C';
		cph_config->magic[1] = 'P';
		cph_config->magic[2] = 'H';
		cph_config->magic[3] = 'T';
		cph_config->fw_major = FW_MAJOR;
		cph_config->fw_minor = FW_MINOR;
		cph_config->hw_major = BOARD_REV_MAJOR;
		cph_config->hw_minor = BOARD_REV_MINOR;
		cph_config->panid = MAC_PAN_ID;
		cph_config->shortid = 0;
		cph_config_write();
	}

	TRACE("HW:%2X.%02X  FW:%2X.%02X  PAN_ID:%04X  SHORT_ID:%04X\r\n",
			cph_config->hw_major,
			cph_config->hw_minor,
			cph_config->fw_major,
			cph_config->fw_minor,
			cph_config->panid,
			cph_config->shortid);
}

void usart_callback_rx(struct usart_module * usart) {
//	port_pin_toggle_output_level(LED_PIN);
	cph_signal = rx_char;
	usart_read_job(usart_instance, &rx_char);
}

int main(void) {
	system_init();

	cph_millis_init();
	usart_instance = cph_stdio_init();
	cph_stdio_set_rx_callback(usart_callback_rx);

	// start listening
	usart_read_job(usart_instance, &rx_char);

	system_interrupt_enable_global();

	TRACE(APP_NAME, FW_MAJOR, FW_MINOR);

	uint32_t f = system_gclk_gen_get_hz(0);
	TRACE("CPU FREQ: %lu\r\n", f);

	// Blink LED for 5 seconds
	for (int i = 0; i < (5 * 4); i++) {
		if (cph_mode & CPH_MODE_ANCHOR)
		{
			if (rx_char == 'c') {
				cph_mode |= CPH_MODE_COORD;
				printf("IS COORDINATOR\r\n");
				break;
			}
		}
		port_pin_set_output_level(LED_PIN, false);
		cph_millis_delay(125);
		port_pin_set_output_level(LED_PIN, true);
		cph_millis_delay(125);
	}

	// Get configuration page from NVM
	init_config();

	// Generate short id if we don't have one
	if (cph_config->shortid == 0) {
		cph_config->shortid = cph_utils_get_shortid_candidate();
		cph_config_write();
		TRACE("Generated candidate shortid 0x%04X\r\n", cph_config->shortid);
	}

    // Start with board specific hardware init.
	cph_deca_init_gpio();
	cph_deca_spi_init();

	app_run();
}
