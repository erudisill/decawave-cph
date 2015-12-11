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


/* Frames used in the ranging process.  */
static msg_poll tx_poll_msg =
{
		0x8841,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_ANCHOR_ID,	// mac_dest  	'A' 'W'
		MAC_TAG_ID,		// mac_source	'E' 'V'
		0xE0,			// functionCode
		0x0000			// mac_cs
};

static msg_resp rx_resp_msg =
{
		0x8841,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_TAG_ID,		// mac_dest		'E' 'V'
		MAC_ANCHOR_ID,	// mac_source  	'A' 'W'
		0xE1,			// functionCode
		0x00000000,		// pollRxTs
		0x00000000,		// respTxTs
		0x0000			// mac_cs
};

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



static int poll(double * dist) {
	int result = CPH_OK;

	// Setup POLL frame to request to range with anchor
	tx_poll_msg.mac_sequence = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg), (uint8_t*)(&tx_poll_msg), 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

    // Start transmission, indicating that a response is expected so that reception is enabled automatically
    // after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed.
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout.
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
    { };

    // Increment frame sequence number after transmission of the poll message (modulo 256).
    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG)
    {
        uint32 frame_len;

        // Clear good RX frame event in the DW1000 status register.
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        // A frame has been received, read it into the local buffer.
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RX_BUF_LEN)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);

			// If valid response, calculate distance
			rx_buffer[ALL_MSG_SN_IDX] = 0;
			if (memcmp(rx_buffer, (uint8_t*)&rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
			{
				uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
				int32 rtd_init, rtd_resp;

				// Retrieve poll transmission and response reception timestamps.
				poll_tx_ts = dwt_readtxtimestamplo32();
				resp_rx_ts = dwt_readrxtimestamplo32();

				// Get timestamps embedded in response message.
				poll_rx_ts = ((msg_resp*)(rx_buffer))->pollRxTs;
				resp_tx_ts = ((msg_resp*)(rx_buffer))->respTxTs;

				// Compute time of flight and distance.
				rtd_init = resp_rx_ts - poll_tx_ts;
				rtd_resp = resp_tx_ts - poll_rx_ts;

				tof = ((rtd_init - rtd_resp) / 2.0) * DWT_TIME_UNITS;
				*dist = tof * SPEED_OF_LIGHT;
			}
			else {
				result = CPH_BAD_FRAME;
			}
        }
        else {
        	result = CPH_BAD_LENGTH;
        }
    }
    else
    {
        // Clear RX error events in the DW1000 status register.
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    	result = CPH_ERROR;
    }

    return result;
}

void tag_run(void)
{
    reset_DW1000();
    spi_set_rate_low();
    dwt_initialise(DWT_LOADUCODE);
    spi_set_rate_high();

    dwt_configure(&config);

    dwt_setpanid(MAC_PAN_ID);
    dwt_setaddress16(MAC_SHORT);
    dwt_enableframefilter(DWT_FF_DATA_EN);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    while (1)
    {
    	int result = poll(&distance);
    	if (result == CPH_OK) {
			printf("DIST: %3.2f m\r\n", distance);
    	}
    	else {
    		printf("ERROR: %d\r\n", result);
    	}

        // Execute a delay between ranging exchanges.
        deca_sleep(RNG_DELAY_MS);
    }
}


