/*
 * anchor.c
 *
 *  Created on: Dec 9, 2015
 *      Author: ericrudisill
 */

#include <stdio.h>
#include <string.h>

#include <cph.h>
#include <cph_deca_port.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_sleep.h>

// Default configuration for DW communication
static dwt_config_t config = DW_CONFIG;

/* Frames used in the ranging process. See NOTE 3 below. */
static msg_poll rx_poll_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_ANCHOR_ID,	// mac_dest
		MAC_TAG_ID,		// mac_source
		FUNC_POLL,		// functionCode
		0x0000			// mac_cs
		};

static msg_resp tx_resp_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_TAG_ID,		// mac_dest
		MAC_ANCHOR_ID,	// mac_source
		FUNC_RESP,		// functionCode
		0x00000000,		// pollRxTs
		0x00000000,		// respTxTs
		0x0000			// mac_cs
		};

static msg_announce_anchor tx_announce_anchor_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_TAG_ID,		// mac_dest
		MAC_ANCHOR_ID,	// mac_source
		FUNC_ANNOUNCE_ANCHOR,	// functionCode
		0x0000,			// coordid
		0x0000			// mac_cs
		};

static msg_announce_anchor rx_pair_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_TAG_ID,		// mac_dest
		MAC_ANCHOR_ID,	// mac_source
		FUNC_PAIR,		// functionCode
		0x0000			// mac_cs
		};

static msg_announce_coord tx_announce_coord_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		0xFFFF,			// mac_dest
		MAC_ANCHOR_ID,	// mac_source
		FUNC_ANNOUNCE_COORD,	// functionCode
		0x0000			// mac_cs
		};

static msg_range_results rx_range_results_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_TAG_ID,	// mac_dest
		MAC_ANCHOR_ID,		// mac_source
		FUNC_RANGE_RESULTS,		// functionCode
		0,				// num ranges
		0,		// results
		0x0000			// mac_cs
		};

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
static uint8 rx_buffer[CPH_MAX_MSG_SIZE];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* List of paired tags */
static pair_info_t paired_tags[MAX_TAGS];

/* Declaration of static functions. */
static uint64 get_rx_timestamp_u64(void);

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
	tx_announce_coord_msg.mac_sequence = frame_seq_nb;
	tx_announce_coord_msg.coordid = cph_coordid;
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	dwt_writetxdata(sizeof(tx_announce_coord_msg), (uint8_t*) (&tx_announce_coord_msg), 0);
	dwt_writetxfctrl(sizeof(tx_announce_coord_msg), 0);

	// Burst out our announcement
	for (int i = 0; i < repeat; i++) {
		dwt_starttx(DWT_START_TX_IMMEDIATE);
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS)) {
		};
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		frame_seq_nb++;
		deca_sleep(10);
	}
}

void anchor_run(void) {
	uint32_t announce_coord_ts = 0;
	uint32_t elapsed = 0;

	memset(paired_tags, 0, sizeof(pair_info_t) * MAX_TAGS);

	reset_DW1000();
	spi_set_rate_low();
	dwt_initialise(DWT_LOADUCODE);
	spi_set_rate_high();

	dwt_configure(&config);

	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	if (cph_config->shortid == 0) {
		cph_config->shortid = cph_utils_get_shortid_candidate();
		cph_config_write();
		TRACE("Generated candidate shortid 0x%04X\r\n", cph_config->shortid);
	}

	dwt_setpanid(cph_config->panid);
	dwt_setaddress16(cph_config->shortid);
	dwt_enableframefilter(DWT_FF_DATA_EN);

	rx_poll_msg.mac_dest = cph_config->shortid;
	tx_resp_msg.mac_source = cph_config->shortid;
	tx_announce_anchor_msg.mac_source = cph_config->shortid;
	rx_pair_msg.mac_dest = cph_config->shortid;
	tx_announce_coord_msg.mac_source = cph_config->shortid;

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
		dwt_rxenable(0);

		/* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
		};

		if (status_reg & SYS_STATUS_RXFCG) {
			uint32 frame_len;

			/* Clear good RX frame event in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
			if (frame_len <= CPH_MAX_MSG_SIZE) {
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}

			// Look for Poll message
			if (((msg_poll*) rx_buffer)->functionCode == FUNC_POLL) {
				uint32 resp_tx_time;

				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();

				/* Compute final message transmission time. See NOTE 7 below. */
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
				resp_tx_ts = (((uint64) (resp_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

				/* Write all timestamps in the final message. See NOTE 8 below. */
				tx_resp_msg.pollRxTs = poll_rx_ts;
				tx_resp_msg.respTxTs = resp_tx_ts;

				/* Write and send the response message. See NOTE 9 below. */
				tx_resp_msg.mac_sequence = 0;
				tx_resp_msg.mac_dest = ((msg_poll*) rx_buffer)->mac_source;
				dwt_writetxdata(sizeof(tx_resp_msg), (uint8_t*) &tx_resp_msg, 0);
				dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
				int result = dwt_starttx(DWT_START_TX_DELAYED);
				uint32_t ts = dwt_readsystimestamphi32();

				if (result == DWT_SUCCESS) {
					/* Poll DW1000 until TX frame sent event set. See NOTE 6 below. */
					while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS)) {
					};
					//TRACE("SUCCESS: dwt_startx response sent %d .. status_reg:%08X\r\n", frame_seq_nb, status_reg);
				} else {
					TRACE("ERROR: dwt_starttx response returned %d  .. resp_tx_time:%08X   systime:%08X\r\n", result,
							resp_tx_time, ts);
				}

				/* Clear TXFRS event. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

				/* Increment frame sequence number after transmission of the poll message (modulo 256). */
				frame_seq_nb++;
			} else if (((msg_poll*) rx_buffer)->functionCode == FUNC_DISCOVER) {

				if (can_respond_to_discover(((msg_poll*) rx_buffer)->mac_source)) {
					/* Write and send the announce message. */
					tx_announce_anchor_msg.mac_sequence = 0;
					tx_announce_anchor_msg.coordid = cph_coordid;
					tx_announce_anchor_msg.mac_dest = ((msg_poll*) rx_buffer)->mac_source;
					dwt_writetxdata(sizeof(tx_announce_anchor_msg), (uint8_t*) &tx_announce_anchor_msg, 0);
					dwt_writetxfctrl(sizeof(tx_announce_anchor_msg), 0);
					int result = dwt_starttx(DWT_START_TX_IMMEDIATE);

					if (result == DWT_SUCCESS) {
						/* Poll DW1000 until TX frame sent event set. See NOTE 6 below. */
						while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS)) {
						};
					} else {
						TRACE("ERROR: dwt_starttx announce returned %d\r\n");
					}

					/* Clear TXFRS event. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

					/* Increment frame sequence number after transmission of the poll message (modulo 256). */
					frame_seq_nb++;
				} else {
					TRACE("ignoring pair with %04X\r\n", ((msg_poll*) rx_buffer)->mac_source);
				}
			} else if (((msg_poll*) rx_buffer)->functionCode == FUNC_PAIR) {
				//TODO: Record the pairing details and check them when receiving a discover request
				//      For now, nothing to do
				if (update_paired_tags(((msg_poll*) rx_buffer)->mac_source)) {
					TRACE("paired with %04X\r\n", ((msg_poll*) rx_buffer)->mac_source);
				} else {
					TRACE("failed to pair with %04X\r\n", ((msg_poll*) rx_buffer)->mac_source);
				}
			} else if (((msg_poll*) rx_buffer)->functionCode == FUNC_ANNOUNCE_COORD) {
				uint16_t id = ((msg_announce_coord*) rx_buffer)->coordid;
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
			} else if (((msg_poll*) rx_buffer)->functionCode == FUNC_RANGE_RESULTS) {
				msg_range_results * results = ((msg_range_results*) rx_buffer);
				TRACE("* %04X", results->mac_source);
				for (int i = 0; i < results->numranges; i++) {
					TRACE(" %04X:%3.2f", results->ranges[i].shortid, results->ranges[i].range);
				}
				TRACE("\r\n");

			} else {
				TRACE("ERROR: unknown function code - data: ");
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
