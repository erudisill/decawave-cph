/*
 * tag.c
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

#define ANCHOR_ID		0x616A

/* Frames used in the ranging process.  */
static msg_poll tx_poll_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_ANCHOR_ID,	// mac_dest  	'A' 'W'
		MAC_TAG_ID,		// mac_source	'E' 'V'
		FUNC_POLL,		// functionCode
		0x0000			// mac_cs
		};

static msg_resp rx_resp_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_TAG_ID,		// mac_dest		'E' 'V'
		MAC_ANCHOR_ID,	// mac_source  	'A' 'W'
		FUNC_RESP,		// functionCode
		0x00000000,		// pollRxTs
		0x00000000,		// respTxTs
		0x0000			// mac_cs
		};

static msg_discover tx_discover_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		0xFFFF,			// mac_dest
		MAC_TAG_ID,		// mac_source
		FUNC_DISCOVER,	// functionCode
		0x0000			// mac_cs
		};

static msg_announce_anchor rx_announce_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_ANCHOR_ID,	// mac_dest
		MAC_TAG_ID,		// mac_source
		FUNC_ANNOUNCE_ANCHOR,	// functionCode
		0x0000			// mac_cs
		};

static msg_pair tx_pair_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_ANCHOR_ID,	// mac_dest
		MAC_TAG_ID,		// mac_source
		FUNC_PAIR,		// functionCode
		0x0000			// mac_cs
		};


static msg_range_results tx_range_results_msg = {
MAC_FC,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_ANCHOR_ID,	// mac_dest
		MAC_TAG_ID,		// mac_source
		FUNC_RANGE_RESULTS,		// functionCode
		0,				// num ranges
		0,		// results
		0x0000			// mac_cs
		};


// Discovered anchors and their ranges
static anchor_range_t anchors[ANCHORS_MIN];
static unsigned int anchors_status;

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;

static int range(double * dist) {
	int result = CPH_OK;

	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

	// Setup POLL frame to request to range with anchor
	tx_poll_msg.mac_sequence = frame_seq_nb;
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	dwt_writetxdata(sizeof(tx_poll_msg), (uint8_t*) (&tx_poll_msg), 0);
	dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

	// Start transmission, indicating that a response is expected so that reception is enabled automatically
	// after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed.
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	// We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout.
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
	};

	// Increment frame sequence number after transmission of the poll message (modulo 256).
	frame_seq_nb++;

	if (status_reg & SYS_STATUS_RXFCG) {
		uint32 frame_len;

		// Clear good RX frame event in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		// A frame has been received, read it into the local buffer.
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN) {
			dwt_readrxdata(rx_buffer, frame_len, 0);

			// If valid response, calculate distance
			if (((msg_resp*) rx_buffer)->functionCode == 0xE1) {
				uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
				int32 rtd_init, rtd_resp;

				// Retrieve poll transmission and response reception timestamps.
				poll_tx_ts = dwt_readtxtimestamplo32();
				resp_rx_ts = dwt_readrxtimestamplo32();

				// Get timestamps embedded in response message.
				poll_rx_ts = ((msg_resp*) (rx_buffer))->pollRxTs;
				resp_tx_ts = ((msg_resp*) (rx_buffer))->respTxTs;

				// Compute time of flight and distance.
				rtd_init = resp_rx_ts - poll_tx_ts;
				rtd_resp = resp_tx_ts - poll_rx_ts;

				tof = ((rtd_init - rtd_resp) / 2.0) * DWT_TIME_UNITS;
				*dist = tof * SPEED_OF_LIGHT;
			} else {
				result = CPH_BAD_FRAME;
			}
		} else {
			result = CPH_BAD_LENGTH;
		}
	} else {
//		printf("STATUS: %08X\r\n", status_reg);
		// Clear RX error events in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		result = CPH_ERROR;
	}

	return result;
}

static int discover(int idx) {
	int result = CPH_OK;

	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(600);

	// Setup POLL frame to request to range with anchor
	tx_discover_msg.mac_sequence = frame_seq_nb;
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	dwt_writetxdata(sizeof(tx_discover_msg), (uint8_t*) (&tx_discover_msg), 0);
	dwt_writetxfctrl(sizeof(tx_discover_msg), 0);

	// Start transmission, indicating that a response is expected so that reception is enabled automatically
	// after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed.
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	// We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout.
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
	};

	// Increment frame sequence number after transmission of the poll message (modulo 256).
	frame_seq_nb++;

	if (status_reg & SYS_STATUS_RXFCG) {
		uint32 frame_len;

		// Clear good RX frame event in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		// A frame has been received, read it into the local buffer.
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= sizeof(msg_announce_anchor)) {
			dwt_readrxdata(rx_buffer, frame_len, 0);

			// If valid response, send the reply
			if (((msg_resp*) rx_buffer)->functionCode == FUNC_ANNOUNCE_ANCHOR) {

				uint16_t shortid = ((msg_resp*) rx_buffer)->mac_source;

				// Now send the pair response back
				tx_pair_msg.mac_sequence = frame_seq_nb;
				tx_pair_msg.mac_dest = shortid;
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
				dwt_writetxdata(sizeof(tx_pair_msg), (uint8_t*) (&tx_pair_msg), 0);
				dwt_writetxfctrl(sizeof(tx_pair_msg), 0);
				dwt_starttx(DWT_START_TX_IMMEDIATE);
				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
				{ };
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
                frame_seq_nb++;

                // Grab the coordinator id
                if (((msg_announce_anchor*) rx_buffer)->coordid != cph_coordid) {
                	cph_coordid = ((msg_announce_anchor*) rx_buffer)->coordid;
                	printf("coordinator discovered at %04X\r\n", cph_coordid);
                }

                // Check for duplicate
				for (int i=0;i<ANCHORS_MIN;i++) {
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
//		status_reg = dwt_read32bitreg(SYS_STATUS_ID);
//		printf("discover: error status_reg:%08X\r\n", status_reg);
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

		for (int i=0;i<ANCHORS_MIN;i++) {
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
	printf("%d\t%04X\t", tries,cph_coordid);
	for (int i = 0; i < ANCHORS_MIN; i++) {
		printf("%04X: %3.2f m\t", anchors[i].shortid, anchors[i].range);
	}
	printf("\r\n");

	if (cph_coordid) {
		// Now send the results
		tx_range_results_msg.mac_sequence = frame_seq_nb;
		tx_range_results_msg.mac_dest = cph_coordid;
		tx_range_results_msg.numranges = ANCHORS_MIN;
		memcpy(&tx_range_results_msg.ranges[0], &anchors[0], sizeof(anchor_range_t) * ANCHORS_MIN);

		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		dwt_writetxdata(sizeof(tx_range_results_msg), (uint8_t*) (&tx_range_results_msg), 0);
		dwt_writetxfctrl(sizeof(tx_range_results_msg), 0);
		dwt_starttx(DWT_START_TX_IMMEDIATE);

		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
		{ };

		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		frame_seq_nb++;
	}
}

void tag_run(void) {

	// Setup DECAWAVE
	reset_DW1000();
	spi_set_rate_low();
	dwt_initialise(DWT_LOADUCODE);
	spi_set_rate_high();

	dwt_configure(&config);

	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	// Determine short id
	if (cph_config->shortid == 0) {
		cph_config->shortid = cph_utils_get_shortid_candidate();
		cph_config_write();
		TRACE("Generated candidate shortid 0x%04X\r\n", cph_config->shortid);
	}
	tx_poll_msg.mac_source = cph_config->shortid;
	rx_resp_msg.mac_dest = cph_config->shortid;
	tx_discover_msg.mac_source = cph_config->shortid;
	rx_announce_msg.mac_dest = cph_config->shortid;
	tx_pair_msg.mac_source = cph_config->shortid;
	tx_range_results_msg.mac_source = cph_config->shortid;

	// Configure network parameters
	dwt_setpanid(cph_config->panid);
	dwt_setaddress16(cph_config->shortid);
	dwt_enableframefilter(DWT_FF_DATA_EN);


	// First, discover anchors
	uint32_t anchor_refresh_ts = 0;
	refresh_anchors();
	anchor_refresh_ts = cph_get_millis();

	// Poll loop
	while (1) {

		int ranges_countdown = MAX_RANGES_BEFORE_POLL_TIMEOUT;
		anchors_status = ANCHORS_MASK;

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
					tx_poll_msg.mac_dest = anchors[i].shortid;
					rx_resp_msg.mac_source = anchors[i].shortid;
					anchors[i].range = 0;
					int result = range(&anchors[i].range);

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
		}
		else {
			printf("ranges_countdown expired!\r\n");
		}

		// Execute a delay between ranging exchanges.
		deca_sleep(POLL_DELAY_MS);
	}
}

