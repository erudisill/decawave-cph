/*
 * cph_deca_range.c
 *
 *  Created on: Mar 10, 2016
 *      Author: ericrudisill
 */

#include <cph_deca_range.h>

/* Frames used in the ranging process.  */
static cph_deca_msg_range_request_t tx_poll_msg = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_ANCHOR_ID,	// mac.dest  	'A' 'W'
		MAC_TAG_ID,		// mac.source	'E' 'V'
		FUNC_RANGE_POLL,		// functionCode
		0x0000			// mac_cs
		};

static cph_deca_msg_range_final_t tx_range_final_msg = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_TAG_ID,		// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_RANGE_FINA,		// functionCode
		0x00000000,		// pollRxTs
		0x00000000,		// respTxTs
		0x00000000,		// finalTxTs
		0x0000			// mac_cs
		};


/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

typedef unsigned long long uint64;

static uint64 get_rx_timestamp_u64(void);
static uint64 get_tx_timestamp_u64(void);


#if defined(RANGE_METHOD_DS_TWR)
int cph_deca_range(cph_deca_anchor_range_t * range, uint8 * rx_buffer) {
	int result = CPH_OK;

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS * 2);

	// Setup POLL/FINAL frames to request to range with anchor
	tx_poll_msg.header.source = cph_config->shortid;
	tx_poll_msg.header.dest = range->shortid;
	tx_range_final_msg.header.source = cph_config->shortid;

	cph_deca_load_frame((cph_deca_msg_header_t*) &tx_poll_msg, sizeof(tx_poll_msg));
	status_reg = cph_deca_send_response_expected();

	if (status_reg & SYS_STATUS_RXFCG) {
		uint32 frame_len;
		cph_deca_msg_header_t * rx_header;

		// A frame has been received, read it into the local buffer.
		rx_header = cph_deca_read_frame(rx_buffer, &frame_len);
		if (rx_header) {
			// If valid response, calculate distance
			if (rx_header->functionCode == FUNC_RANGE_RESP) {

				uint32_t final_tx_time;
				uint64 poll_tx_ts, resp_rx_ts, final_tx_ts;

				// Retrieve poll transmission and response reception timestamps.
				poll_tx_ts = get_tx_timestamp_u64();
				resp_rx_ts = get_rx_timestamp_u64();

				// Compute final message transmission time
				final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(final_tx_time);

				// Final TX timestamp is the transmission time we programmed plus the TX antenna delay.
				final_tx_ts = (((uint64) (final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

				// Write timestamps
				tx_range_final_msg.pollTxTs = (uint32_t) (poll_tx_ts & 0x00000000ffffffff);
				tx_range_final_msg.responseRxTs = (uint32_t) (resp_rx_ts & 0x00000000ffffffff);
				tx_range_final_msg.responseTxTs = ((cph_deca_msg_range_response_t*) rx_header)->responseTxTs;
				tx_range_final_msg.finalTxTs = (uint32_t) (final_tx_ts & 0x00000000ffffffff);

				// Send the final set of timestamps after calculated delay...wait for results
				tx_range_final_msg.header.dest = range->shortid;
				cph_deca_load_frame((cph_deca_msg_header_t*) &tx_range_final_msg, sizeof(tx_range_final_msg));
				//cph_deca_send_delayed();
				status_reg = cph_deca_send_delayed_response_expected();

				// RESULT RECEIVED
				if (status_reg & SYS_STATUS_RXFCG) {
					rx_header = cph_deca_read_frame(rx_buffer, &frame_len);
					if (rx_header) {
						if (rx_header->functionCode == FUNC_RANGE_RESU) {
							// Got it!  Record the range.
							range->range = ((cph_deca_msg_range_result_t*)rx_buffer)->range.range;
						} else {
							result = CPH_BAD_FRAME;
						}
					} else {
						result = CPH_BAD_LENGTH;
					}

				} else {
					// Clear RX error events in the DW1000 status register.
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
					result = CPH_ERROR;
				}


			} else {
				result = CPH_BAD_FRAME;
			}
		} else {
			result = CPH_BAD_LENGTH;
		}
	} else {
		// Clear RX error events in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		result = CPH_ERROR;
	}

	return result;
}

#else

int cph_deca_range(cph_deca_anchor_range_t * range, uint8 * rx_buffer) {
	int result = CPH_OK;

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

	// Setup POLL frame to request to range with anchor
	tx_poll_msg.header.dest = range->shortid;

	cph_deca_load_frame((cph_deca_msg_header_t*)&tx_poll_msg, sizeof(tx_poll_msg));
	status_reg = cph_deca_send_response_expected();

	if (status_reg & SYS_STATUS_RXFCG) {
		uint32 frame_len;
		cph_deca_msg_header_t * rx_header;

		// A frame has been received, read it into the local buffer.
		rx_header = cph_deca_read_frame(rx_buffer, &frame_len);
		if (rx_header) {
			// If valid response, calculate distance
			if (rx_header->functionCode == FUNC_RANGE_RESP) {
				uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
				int32 rtd_init, rtd_resp;

				// Retrieve poll transmission and response reception timestamps.
				poll_tx_ts = dwt_readtxtimestamplo32();
				resp_rx_ts = dwt_readrxtimestamplo32();

				// Get timestamps embedded in response message.
				poll_rx_ts = ((cph_deca_msg_range_response_t*) (rx_header))->pollRxTs;
				resp_tx_ts = ((cph_deca_msg_range_response_t*) (rx_header))->responseTxTs;

				// Compute time of flight and distance.
				rtd_init = resp_rx_ts - poll_tx_ts;
				rtd_resp = resp_tx_ts - poll_rx_ts;

				tof = ((rtd_init - rtd_resp) / 2.0) * DWT_TIME_UNITS;
				range->range = tof * SPEED_OF_LIGHT;

			} else {
				result = CPH_BAD_FRAME;
			}
		} else {
			result = CPH_BAD_LENGTH;
		}
	} else {
		// Clear RX error events in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		result = CPH_ERROR;
	}

	return result;
}

#endif



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void) {
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readrxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void) {
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readtxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}
