/*
 * cph_deca_port.h
 *
 *  Created on: Nov 19, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_CPH_DECA_PORT_H_
#define SRC_CPH_CPH_DECA_PORT_H_

#include <cph.h>
#include <deca_device_api.h>

void cph_deca_spi_init(void);
void cph_deca_spi_ss_select(void);
void cph_deca_spi_ss_deselect(void);
int cph_deca_spi_write(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer);
int cph_deca_spi_read(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer);

void cph_deca_init_gpio(void);

void cph_deca_isr_disable(void);
void cph_deca_isr_enable(void);
bool cph_deca_isr_is_detected(void) ;
decaIrqStatus_t cph_deca_isr_mutex_on(void);
void cph_deca_isr_mutex_off(decaIrqStatus_t s);
void cph_deca_isr(void);

void cph_deca_reset(void);

// Simple mutex operation. Just enable/disable the ISR for now
#define decamutexon()			cph_deca_isr_mutex_on()
#define decamutexoff(x)			cph_deca_isr_mutex_off(x)

// Redirect DW API SPI calls to our functions
#define writetospi				cph_deca_spi_write
#define readfromspi				cph_deca_spi_read

#define Sleep(n)				cph_millis_delay(n)

#endif /* SRC_CPH_CPH_DECA_PORT_H_ */
