/*
 * anchor.c
 *
 *  Created on: Dec 9, 2015
 *      Author: ericrudisill
 */

#include <stdio.h>
#include <string.h>

#include <cph.h>
#include <cph_deca.h>
#include <cph_deca_range.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_sleep.h>

static cph_deca_msg_range_response_t tx_range_response = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_TAG_ID,		// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_RANGE_RESP,		// functionCode
		0x00000000,		// pollRxTs
		0x00000000,		// respTxTs
		0x0000			// mac_cs
		};

static cph_deca_msg_discover_reply_t tx_discover_reply = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_TAG_ID,		// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_DISC_REPLY,	// functionCode
		0x0000,			// coordid
		0x0000			// mac_cs
		};

static cph_deca_msg_coord_announce_t tx_coord_announce = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		0xFFFF,			// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_COORD_ANNO,	// functionCode
		0x0000			// mac_cs
		};

static cph_deca_msg_range_result_t tx_range_result = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_TAG_ID,		// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_RANGE_RESU,	// functionCode
		0,				// cph_deca_anchor_range_t
		0x0000			// mac_cs
		};

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
static uint8 rx_buffer[CPH_MAX_MSG_SIZE];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
typedef signed long long int64;

static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* List of paired tags */
static cph_deca_pair_info_t paired_tags[MAX_TAGS];

/* Declaration of static functions. */
static uint64 get_rx_timestamp_u64(void);
static uint64 get_tx_timestamp_u64(void);

static bool can_respond_to_discover(uint16_t shortid) {
	for (int i = 0; i < MAX_TAGS; i++) {
		if (paired_tags[i].shortid == shortid) {
			uint32_t elapsed = g_cph_millis - paired_tags[i].paired_ts;
			if (elapsed < PAIR_LIFETIME) {
				// Already paired - return no
				return false;
			} else {
				// Paired, but timed out - return yes
				return true;
			}
		}
	}
	// No pair found - return yes
	return true;
}

static bool update_paired_tags(uint16_t shortid) {
	int first_empty = -1;
	int i = 0;

	for (i = 0; i < MAX_TAGS; i++) {
		if (paired_tags[i].shortid == 0 && first_empty == -1)
			first_empty = i;
		if (paired_tags[i].shortid == shortid) {
			// Found tag, update timestamp and return success
			paired_tags[i].paired_ts = g_cph_millis;
			return true;
		}
	}

	// Not found, so try and add it
	if (first_empty == -1) {
		// No empty slot, return fail
		return false;
	}

	paired_tags[first_empty].shortid = shortid;
	paired_tags[first_empty].paired_ts = g_cph_millis;
	return true;

}

static void announce_coord(int repeat) {

	TRACE("announcing coordinator %04X", cph_coordid);
	if (cph_coordid == cph_config->shortid) {
		TRACE(" (me)");
	}
	TRACE("\r\n");

	// Write announce message to frame buffer
	tx_coord_announce.coordid = cph_coordid;
	cph_deca_load_frame(&tx_coord_announce.header, sizeof(tx_coord_announce));

	// Burst out our announcement
	for (int i = 0; i < repeat; i++) {
		cph_deca_send_immediate();
		deca_sleep(10);
	}
}

static double range_with_anchor(uint16_t reps, uint16_t periodms) {
	int status = CPH_OK;
	double accum;
	int count;

	cph_deca_anchor_range_t anchor;

	anchor.shortid = 0x6518;

	TRACE("RANGING with %04X\r\n", anchor.shortid);

	// Go back to idle
	dwt_forcetrxoff();

	accum = 0;
	count = 0;

	for (int i=0;i<reps;i++) {
		anchor.range = 0;
		if ((status = cph_deca_range(&anchor, rx_buffer)) == CPH_OK) {
			TRACE("%02d Range: %3.2fm\r\n", count, anchor.range);
			accum += anchor.range;
			count++;
		}
		else {
			TRACE("RANGE ERROR!  %02X\r\n", status);
		}
		cph_millis_delay(periodms);
	}

	return accum / count;
}

void anchor_run(void) {
	uint32_t announce_coord_ts = 0;
	uint32_t elapsed = 0;

	// Setup DW1000
	cph_deca_init_device();
	cph_deca_init_network(cph_config->panid, cph_config->shortid);

	// Init list of paired tags
	memset(paired_tags, 0, sizeof(cph_deca_pair_info_t) * MAX_TAGS);

	// Set our shortid in common messages
	tx_range_response.header.source = cph_config->shortid;
	tx_discover_reply.header.source = cph_config->shortid;
	tx_coord_announce.header.source = cph_config->shortid;

	// Announce ourselves if we're the coordinator
	if (cph_mode & CPH_MODE_COORD) {
		cph_coordid = cph_config->shortid;
		cph_mode |= CPH_MODE_COORD;
		announce_coord(COORD_ANNOUNCE_START_BURST);
	}

	/* Loop forever responding to ranging requests. */
	while (1) {

		if (cph_coordid) {
			elapsed = cph_get_millis() - announce_coord_ts;
			if (elapsed > COORD_ANNOUNCE_INTERVAL) {
				announce_coord(1);
				announce_coord_ts = cph_get_millis();
			}
		}

		/* Activate reception immediately. */
#if defined(RANGE_METHOD_DS_TWR)
		dwt_setrxtimeout(0);
#endif
		dwt_rxenable(0);

		cph_signal = 0x00;
		//status_reg = cph_deca_wait_for_rx_finished_or_signal(&cph_signal);
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
			if (cph_signal != 0) {
				break;
			}
		};

		if (cph_signal) {
			if (cph_signal == 'r') {
				double avg_range = range_with_anchor(50, 10);
				TRACE("AVERAGE RANGE ====> %3.2fm\r\n", avg_range);
				continue;
			}
		}

		if (status_reg & SYS_STATUS_RXFCG) {
			uint32 frame_len;
			cph_deca_msg_header_t * rx_header;

			rx_header = cph_deca_read_frame(rx_buffer, &frame_len);


#if defined(COORD_NOT_ANCHOR)
			// If we're the coordinator, only accept range reports
			if (cph_mode & CPH_MODE_COORD) {
				if (rx_header->functionCode != FUNC_RANGE_REPO)
					continue;
			}
#endif

			// Look for Poll message
			if (rx_header->functionCode == FUNC_RANGE_POLL) {
				uint32 resp_tx_time;

				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();

				/* Compute final message transmission time. See NOTE 7 below. */
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

#if defined(RANGE_METHOD_DS_TWR)
				/* Set expected delay and timeout for final message reception. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
#endif

				/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
				resp_tx_ts = (((uint64) (resp_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

				/* Write all timestamps in the final message. See NOTE 8 below. */
				tx_range_response.pollRxTs = poll_rx_ts;
				tx_range_response.responseTxTs = resp_tx_ts;

				/* Send the response message */
				tx_range_response.header.dest = rx_header->source;
				cph_deca_load_frame(&tx_range_response.header, sizeof(tx_range_response));
				cph_deca_send_delayed();

//				TRACE("ts: %08X  %08X\r\n", (uint32_t)(poll_rx_ts >> 8), resp_tx_time);

			} else if (rx_header->functionCode == FUNC_RANGE_FINA) {

				uint64 final_rx_ts;
				uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
				uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
				double Ra, Rb, Da, Db;
				double distance, tof;
				int64 tof_dtu;

				// Retrieve response transmission and final reception timestamps.
				resp_tx_ts = get_tx_timestamp_u64();	//ERIC: Should pull this from final message
				final_rx_ts = get_rx_timestamp_u64();

				// Get timestamps embedded in the final message.
				poll_tx_ts = ((cph_deca_msg_range_final_t*) rx_header)->pollTxTs;
				resp_rx_ts = ((cph_deca_msg_range_final_t*) rx_header)->responseRxTs;
				final_tx_ts = ((cph_deca_msg_range_final_t*) rx_header)->finalTxTs;

				// Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped.
				poll_rx_ts_32 = (uint32) poll_rx_ts;
				resp_tx_ts_32 = (uint32) resp_tx_ts;
				final_rx_ts_32 = (uint32) final_rx_ts;
				Ra = (double) (resp_rx_ts - poll_tx_ts);
				Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
				Da = (double) (final_tx_ts - resp_rx_ts);
				Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);
				tof_dtu = (int64) ((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

				tof = tof_dtu * DWT_TIME_UNITS;
				distance = tof * SPEED_OF_LIGHT;

				// Send result back to tag
				tx_range_result.header.dest = rx_header->source;
				tx_range_result.range.shortid = cph_config->shortid;
				tx_range_result.range.range = distance;
				cph_deca_load_frame(&tx_range_result.header, sizeof(tx_range_result));
				cph_deca_send_immediate();

				if ((cph_mode & CPH_MODE_COORD) == 0) {
					TRACE("%04X DIST: %3.2f m\r\n", rx_header->source, distance);
				}

			} else if (rx_header->functionCode == FUNC_DISC_ANNO) {

				if (can_respond_to_discover(rx_header->source)) {
					/* Write and send the announce message. */
					tx_discover_reply.coordid = cph_coordid;
					tx_discover_reply.header.dest = rx_header->source;
					cph_deca_load_frame(&tx_discover_reply.header, sizeof(tx_discover_reply));
					cph_deca_send_immediate();
				} else {
					TRACE("ignoring pair with %04X\r\n", (rx_header->source));
				}

			} else if (rx_header->functionCode == FUNC_PAIR_RESP) {
				//TODO: Record the pairing details and check them when receiving a discover request
				//      For now, nothing to do
				if (update_paired_tags(rx_header->source)) {
					TRACE("paired with %04X\r\n", (rx_header->source));
				} else {
					TRACE("failed to pair with %04X\r\n", (rx_header->source));
				}

			} else if (rx_header->functionCode == FUNC_COORD_ANNO) {
				uint16_t id = ((cph_deca_msg_coord_announce_t*) rx_buffer)->coordid;
				if (id != cph_coordid) {
					cph_coordid = id;
					if (cph_coordid == cph_config->shortid) {
						TRACE("becoming coord\r\n");
						cph_mode |= CPH_MODE_COORD;
					} else {
						if (cph_mode & CPH_MODE_COORD) {
							TRACE("giving coord to %04X\r\n", cph_coordid);
						} else {
							TRACE("recognizing coord as %04X\r\n", cph_coordid);
						}
						cph_mode &= (~CPH_MODE_COORD);
					}
				}

			} else if (rx_header->functionCode == FUNC_RANGE_REPO) {
				cph_deca_msg_range_report_t * results = ((cph_deca_msg_range_report_t*) rx_buffer);
				TRACE("* %04X", rx_header->source);
				for (int i = 0; i < results->numranges; i++) {
					TRACE(" %04X:%3.2f", results->ranges[i].shortid, results->ranges[i].range);
				}
				TRACE("\r\n");

			} else {
				TRACE("ERROR: unknown function code - data %02X: ", rx_header->functionCode);
				for (int i = 0; i < frame_len; i++)
					TRACE("%02X ", rx_buffer[i]);
				TRACE("\r\n");
			}
		} else {

			// Ignore frame rejections and timeouts
			uint32_t test = status_reg & (~(SYS_STATUS_AFFREJ | SYS_STATUS_RXRFTO));
			if (test & SYS_STATUS_ALL_RX_ERR) {
				TRACE("ERROR: dwt_rxenable has status of %08X\r\n", status_reg);
				/* Clear RX error events in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			}
		}
	}
}

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
