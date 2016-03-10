/*
 *
 * tag.c
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

#define ANCHOR_ID		0x616A

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

static cph_deca_msg_discover_announce_t tx_discover_msg = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		0xFFFF,			// mac.dest
		MAC_TAG_ID,		// mac.source
		FUNC_DISC_ANNO,	// functionCode
		0x0000			// mac_cs
		};

static cph_deca_msg_pair_response_t tx_pair_msg = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_ANCHOR_ID,	// mac.dest
		MAC_TAG_ID,		// mac.source
		FUNC_PAIR_RESP,		// functionCode
		0x0000			// mac_cs
		};

static cph_deca_msg_range_report_t tx_range_results_msg = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_ANCHOR_ID,	// mac.dest
		MAC_TAG_ID,		// mac.source
		FUNC_RANGE_REPO,		// functionCode
		0,				// num ranges
		0,		// results
		0x0000			// mac_cs
		};

static cph_deca_msg_range_response_t tx_range_response_msg = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_TAG_ID,		// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_RANGE_BURST,		// functionCode
		0x00000000,		// pollRxTs
		0x00000000,		// respTxTs
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

typedef unsigned long long uint64;

// Discovered anchors and their ranges
static cph_deca_anchor_range_t anchors[ANCHORS_MIN];
static unsigned int anchors_status;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
static uint8 rx_buffer[CPH_MAX_MSG_SIZE];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

static uint64 get_sys_timestamp_u64(void);

static int discover(int idx) {
	int result = CPH_OK;

	dwt_setrxaftertxdelay(0);
//	dwt_setrxtimeout(600);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS * 2);

	// Broadcast anchor discovery request
	cph_deca_load_frame((cph_deca_msg_header_t*) &tx_discover_msg, sizeof(tx_discover_msg));
	status_reg = cph_deca_send_response_expected();

	if (status_reg & SYS_STATUS_RXFCG) {
		uint32 frame_len;
		cph_deca_msg_header_t * rx_header;

		// A frame has been received, read it into the local buffer.
		rx_header = cph_deca_read_frame(rx_buffer, &frame_len);
		if (rx_header) {
			// If valid response, send the reply
			if (rx_header->functionCode == FUNC_DISC_REPLY) {

				uint16_t shortid = rx_header->source;

				// Now send the pair response back
				tx_pair_msg.header.dest = shortid;
				cph_deca_load_frame((cph_deca_msg_header_t*) &tx_pair_msg, sizeof(tx_pair_msg));
				cph_deca_send_immediate();

				// Grab the coordinator id
				if (((cph_deca_msg_discover_reply_t*) rx_buffer)->coordid != cph_coordid) {
					cph_coordid = ((cph_deca_msg_discover_reply_t*) rx_buffer)->coordid;
					printf("coordinator discovered at %04X\r\n", cph_coordid);
				}

				// Check for duplicate
				for (int i = 0; i < ANCHORS_MIN; i++) {
					if (anchors[i].shortid == shortid) {
						printf("shortid %04X already exists in anchors[%d]\r\n", shortid, i);
						result = CPH_DUPLICATE;
						break;
					}
				}

				// Not a duplicate so store the shortid
				if (result == CPH_OK) {
					anchors[idx].shortid = shortid;
					anchors[idx].range = 0;
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

void init_anchors(void) {
	// Init anchors table
	for (int i = 0; i < ANCHORS_MIN; i++) {
		anchors[i].shortid = 0;
		anchors[i].range = 0;
	}
	anchors_status = ANCHORS_MASK;
}

void refresh_anchors(void) {

	init_anchors();

	// Discover anchors
	uint32_t anchor_refresh_ts = 0;
	while (anchors_status) {
		printf("Discovering anchors .. anchors_status:%02X\r\n", anchors_status);

		// Check for refresh of anchors
		uint32_t elapsed = cph_get_millis() - anchor_refresh_ts;
		if (elapsed > ANCHORS_REFRESH_INTERVAL) {
			printf("Anchors discovery timeout.  anchors_status:%02X\r\n", anchors_status);
			init_anchors();
			anchor_refresh_ts = cph_get_millis();
		}

		for (int i = 0; i < ANCHORS_MIN; i++) {
			if (anchors_status & (1 << i)) {
				int result = discover(i);
				if (result == CPH_OK) {
					anchors_status &= (~(1 << i));
					printf("anchor[%d] %04X\r\n", i, anchors[i].shortid);
				}
				deca_sleep(RNG_DELAY_MS);
			}
		}
		deca_sleep(POLL_DELAY_MS);
	}

	printf("Anchors discovered. Moving to poll.  anchors_status:%02X\r\n", anchors_status);
}

static void send_ranges(int tries) {
	printf("%d\t%04X\t", tries, cph_coordid);
	for (int i = 0; i < ANCHORS_MIN; i++) {
		printf("%04X: %3.2f m\t", anchors[i].shortid, anchors[i].range);
	}
	printf("\r\n");

	if (cph_coordid) {
		// Now send the results
		tx_range_results_msg.header.dest = cph_coordid;
		tx_range_results_msg.numranges = ANCHORS_MIN;
		memcpy(&tx_range_results_msg.ranges[0], &anchors[0], sizeof(cph_deca_anchor_range_t) * ANCHORS_MIN);

		cph_deca_load_frame((cph_deca_msg_header_t*) &tx_range_results_msg, sizeof(tx_range_results_msg));
		cph_deca_send_immediate();
	}
}

void tag_burst_run(void) {
	// Setup DECAWAVE
	cph_deca_init_device();
	cph_deca_init_network(cph_config->panid, cph_config->shortid);

	// Set basic poll message as coming from us, broadcast to all
	tx_range_response_msg.header.source = cph_config->shortid;
	tx_range_response_msg.header.dest = 0xFFFF;
	tx_range_response_msg.header.functionCode = FUNC_RANGE_BURST;

	// Poll loop
	while (1) {

		printf("burst\r\n");

		uint8_t seq = 0;
		uint64 poll_rx_ts = get_sys_timestamp_u64();
		uint64 resp_tx_ts = poll_rx_ts;

		seq = 0;

		for (int i = 0; i < 5; i++) {
			tx_range_response_msg.pollRxTs = resp_tx_ts;
//			resp_tx_ts +=  (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME);
			resp_tx_ts += (8000 * UUS_TO_DWT_TIME);
			uint32_t resp_tx_time = resp_tx_ts >> 8;
			dwt_setdelayedtrxtime(resp_tx_time);
			tx_range_response_msg.responseTxTs = resp_tx_ts;

			tx_range_response_msg.header.seq = seq++;
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
			dwt_writetxdata(sizeof(tx_range_response_msg), (uint8_t*) &tx_range_response_msg, 0);
			dwt_writetxfctrl(sizeof(tx_range_response_msg), 0);
			cph_deca_send_delayed();
		}

		// Execute a delay between ranging exchanges.
//		deca_sleep(POLL_DELAY_MS);
		deca_sleep(1000);
	}
}

void tag_run(void) {
	uint32_t start_ms, elapsed_ms, wait_ms;

	// Setup DECAWAVE
	cph_deca_init_device();
	cph_deca_init_network(cph_config->panid, cph_config->shortid);

	// Set our short id in common messages
	tx_poll_msg.header.source = cph_config->shortid;
	tx_range_final_msg.header.source = cph_config->shortid;
	tx_discover_msg.header.source = cph_config->shortid;
	tx_pair_msg.header.source = cph_config->shortid;
	tx_range_results_msg.header.source = cph_config->shortid;

	// First, discover anchors
	uint32_t anchor_refresh_ts = 0;
	refresh_anchors();
	anchor_refresh_ts = cph_get_millis();

	// Poll loop
	while (1) {

		int ranges_countdown = MAX_RANGES_BEFORE_POLL_TIMEOUT;
		anchors_status = ANCHORS_MASK;

		start_ms = cph_get_millis();

		while (anchors_status && (--ranges_countdown)) {

			// Check for refresh of anchors
			uint32_t elapsed = cph_get_millis() - anchor_refresh_ts;
			if (elapsed > ANCHORS_REFRESH_INTERVAL) {
				printf("Anchors refresh timeout.  anchors_status:%02X\r\n", anchors_status);
				refresh_anchors();
				anchor_refresh_ts = cph_get_millis();
				// Since we refreshed the anchors, need to range again for ALL anchors during this poll
				anchors_status = ANCHORS_MASK;
			}

			// Range each anchor once during this poll
			for (int i = 0; i < ANCHORS_MIN; i++) {
				if (anchors_status & (1 << i)) {
					anchors[i].range = 0;
					int result = cph_deca_range(&anchors[i], rx_buffer);

					if (result == CPH_OK) {
						anchors_status &= (~(1 << i));
					}

					deca_sleep(RNG_DELAY_MS);
				}
			}

			deca_sleep(RNG_DELAY_MS);
		}

		if (ranges_countdown) {
			send_ranges(MAX_RANGES_BEFORE_POLL_TIMEOUT - ranges_countdown);
		} else {
			printf("ranges_countdown expired!\r\n");
		}

		// Execute a delay between ranging exchanges.
		elapsed_ms = cph_get_millis() - start_ms;
		wait_ms = POLL_DELAY_MS - elapsed_ms;
		if (wait_ms > POLL_DELAY_MS)
			wait_ms = POLL_DELAY_MS;
		deca_sleep(wait_ms);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_sys_timestamp_u64()
 *
 * @brief Get the SYS time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_sys_timestamp_u64(void) {
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readsystime(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

