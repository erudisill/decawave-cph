/*
 * cph_deca.c
 *
 *  Created on: Jan 11, 2016
 *      Author: ericrudisill
 */

#include <cph_deca.h>
#include <deca_regs.h>

static uint8 frame_seq_nb = 0;

void cph_deca_load_frame(cph_deca_msg_header_t * hdr, uint16_t size) {
	// Write message to frame buffer
	hdr->seq = frame_seq_nb;
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	dwt_writetxdata(size, (uint8_t*) hdr, 0);
	dwt_writetxfctrl(size, 0);
}

uint32_t cph_deca_send_immediate() {
	uint32_t status_reg;

	dwt_starttx(DWT_START_TX_IMMEDIATE);
	status_reg = cph_deca_wait_for_tx_finished();
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	frame_seq_nb++;

	return status_reg;
}

uint32_t cph_deca_send_delayed() {
	uint32_t status_reg;

	int result = dwt_starttx(DWT_START_TX_DELAYED);
	if (result == DWT_SUCCESS) {
		status_reg = cph_deca_wait_for_tx_finished();
	} else {
		TRACE("ERROR: dwt_starttx response returned %d\r\n", result);
	}
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	frame_seq_nb++;

	return status_reg;
}

uint32_t cph_deca_send_response_expected() {
	uint32_t status_reg;

	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	status_reg = cph_deca_wait_for_rx_finished();
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	frame_seq_nb++;

	return status_reg;
}

cph_deca_msg_header_t * cph_deca_read_frame(uint8_t * rx_buffer, uint32_t *frame_len) {

	/* Clear good RX frame event in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

	/* A frame has been received, read it into the local buffer. */
	*frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
	if (*frame_len <= CPH_MAX_MSG_SIZE) {
		dwt_readrxdata(rx_buffer, *frame_len, 0);
		return (cph_deca_msg_header_t*)rx_buffer;
	}
	else {
		return 0;
	}

}
