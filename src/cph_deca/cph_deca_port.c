/*
 * cph_deca_port.c
 *
 *  Created on: Nov 19, 2015
 *      Author: ericrudisill
 */

#include "cph_deca_port.h"


static struct spi_module spi_master_instance;
static struct spi_slave_inst spi_slave;


void cph_deca_spi_init(void) {
	struct spi_config config_spi_master;
	struct spi_slave_inst_config slave_dev_config;

	spi_slave_inst_get_config_defaults(&slave_dev_config);
	slave_dev_config.ss_pin = DW_SPI_SS_PIN;
	spi_attach_slave(&spi_slave, &slave_dev_config);

	spi_get_config_defaults(&config_spi_master);
	// DOPO: 0x00						DIPO: 0x03
	// PA16/SERCOM1[0]: MOSI			PA19/SERCOM1[]3: MISO
	config_spi_master.mux_setting = DW_SPI_MUX;
	config_spi_master.pinmux_pad0 = DW_SPI_PAD0;
	config_spi_master.pinmux_pad1 = DW_SPI_PAD1;
	config_spi_master.pinmux_pad2 = DW_SPI_PAD2;
	config_spi_master.pinmux_pad3 = DW_SPI_PAD3;
	config_spi_master.transfer_mode = DW_SPI_TRANSFER_MODE;

	spi_init(&spi_master_instance, DW_SPI_HW, &config_spi_master);

	spi_enable(&spi_master_instance);

}

void cph_deca_spi_ss_deselect(void) {
	spi_select_slave(&spi_master_instance, &spi_slave, false);
}

void cph_deca_spi_ss_select(void) {
	spi_select_slave(&spi_master_instance, &spi_slave, true);
}


int cph_deca_spi_write(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer) {
	status_code_t result = STATUS_OK;

	cph_deca_spi_ss_select();

	if ((result = spi_write_buffer_wait(&spi_master_instance, headerBuffer, headerLength)) == STATUS_OK) {
		result = spi_write_buffer_wait(&spi_master_instance, bodyBuffer, bodylength);
	}

	cph_deca_spi_ss_deselect();

	if (result != STATUS_OK) {
		printf("writetospi_serial timeout\r\n");
	}

	return result;
}

int cph_deca_spi_read(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer) {
	status_code_t result = STATUS_OK;

	cph_deca_spi_ss_select();

	if ((result = spi_write_buffer_wait(&spi_master_instance, headerBuffer, headerLength)) == STATUS_OK) {
		result = spi_read_buffer_wait(&spi_master_instance, readBuffer, readlength, 0xff);
	}

	cph_deca_spi_ss_deselect();

	if (result != STATUS_OK) {
		printf("readfromspi_serial timeout\r\n");
	}

	return result;
}



void cph_deca_init_gpio(void) {
	struct extint_chan_conf config_chan;
	extint_chan_get_config_defaults(&config_chan);

	config_chan.gpio_pin = DW_IRQ_PIN;
	config_chan.gpio_pin_mux = DW_IRQ_PIN_MUX;
	config_chan.gpio_pin_pull = DW_IRQ_PIN_PULL;
	config_chan.detection_criteria = DW_IRQ_PIN_DETECT;

	extint_chan_set_config(DW_IRQ_LINE, &config_chan);

	extint_register_callback(cph_deca_isr, DW_IRQ_LINE, EXTINT_CALLBACK_TYPE_DETECT);
}

void cph_deca_isr_disable(void) {
	extint_chan_disable_callback(DW_IRQ_LINE, EXTINT_CALLBACK_TYPE_DETECT);
}

void cph_deca_isr_enable(void) {
	extint_chan_enable_callback(DW_IRQ_LINE, EXTINT_CALLBACK_TYPE_DETECT);
}
bool cph_deca_isr_is_detected(void) {
	return port_pin_get_input_level(DW_IRQ_PIN);
	//return extint_chan_is_detected(DW_IRQ_LINE);
}

decaIrqStatus_t cph_deca_isr_mutex_on(void) {
	TRACE("cph_deca_isr_mutex_on\r\n");
	cph_deca_isr_disable();
	return 0x00;
}

void cph_deca_isr_mutex_off(decaIrqStatus_t s) {
	TRACE("cph_deca_isr_mutex_off\r\n");
	cph_deca_isr_enable();
}

void cph_deca_isr(void) {
	do {
		dwt_isr();
	} while (cph_deca_isr_is_detected() == 1);
}


void cph_deca_reset(void) {
	struct port_config config_port = DW_RST_PIN_CONFIG;

	// Safety
	extint_chan_disable_callback(DW_RST_LINE, EXTINT_CALLBACK_TYPE_DETECT);

	// Pull the pin low
	config_port.direction = PORT_PIN_DIR_OUTPUT;
	config_port.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_output_level(DW_RST_PIN, false);
	port_pin_set_config(DW_RST_PIN, &config_port);

	// Now release it
	config_port.direction = PORT_PIN_DIR_INPUT;
	config_port.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(DW_RST_PIN, &config_port);


	// Why sleep here?  Left over from original code.
	cph_millis_delay(1);
}
