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


/* Frames used in the ranging process. See NOTE 3 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);


static int poll(double * dist) {
	int result = CPH_OK;

	// Setup POLL frame to request to range with anchor
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
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
			if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
			{
				uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
				int32 rtd_init, rtd_resp;

				// Retrieve poll transmission and response reception timestamps.
				poll_tx_ts = dwt_readtxtimestamplo32();
				resp_rx_ts = dwt_readrxtimestamplo32();

				// Get timestamps embedded in response message.
				resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
				resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

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

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn resp_msg_get_ts()
 *
 * @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
 *        least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to get
 *         ts  timestamp value
 *
 * @return none
 */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}


