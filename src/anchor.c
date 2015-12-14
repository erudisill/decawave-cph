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
static msg_poll rx_poll_msg =
{
		0x8841,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_ANCHOR_ID,	// mac_dest
		MAC_TAG_ID,		// mac_source
		0xE0,			// functionCode
		0x0000			// mac_cs
};

static msg_resp tx_resp_msg =
{
		0x8841,			// mac_frameControl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac_sequence
		MAC_PAN_ID,		// mac_panid
		MAC_TAG_ID,		// mac_dest
		MAC_ANCHOR_ID,	// mac_source
		0xE1,			// functionCode
		0x00000000,		// pollRxTs
		0x00000000,		// respTxTs
		0x0000			// mac_cs
};
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 12
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Declaration of static functions. */
static uint64 get_rx_timestamp_u64(void);

void anchor_run(void)
{
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


    /* Loop forever responding to ranging requests. */
    while (1)
    {
    	//print_status(frame_seq_nb);

        /* Activate reception immediately. */
        dwt_rxenable(0);

        /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            uint32 frame_len;

            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUFFER_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            // Look for Poll message
            if (((msg_poll*)rx_buffer)->functionCode == 0xE0)
            {
                uint32 resp_tx_time;

                /* Retrieve poll reception timestamp. */
                poll_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 7 below. */
                resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);

                /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
                resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

                /* Write all timestamps in the final message. See NOTE 8 below. */
                tx_resp_msg.pollRxTs = poll_rx_ts;
                tx_resp_msg.respTxTs = resp_tx_ts;

                /* Write and send the response message. See NOTE 9 below. */
                tx_resp_msg.mac_sequence = 0;
                tx_resp_msg.mac_dest = ((msg_poll*)rx_buffer)->mac_source;
                dwt_writetxdata(sizeof(tx_resp_msg), (uint8_t*)&tx_resp_msg, 0);
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
                int result = dwt_starttx(DWT_START_TX_DELAYED);
            	uint32_t ts = dwt_readsystimestamphi32();

                if (result == DWT_SUCCESS) {
					/* Poll DW1000 until TX frame sent event set. See NOTE 6 below. */
					while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
					{ };
					printf("SUCCESS: dwt_startx response sent %d\r\n", frame_seq_nb);
                }
                else {
                	printf("ERROR: dwt_starttx returned %d  .. resp_tx_time:%08X   systime:%08X\r\n", result, resp_tx_time, ts);
                }

                /* Clear TXFRS event. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                frame_seq_nb++;
            }
        }
        else
        {
        	printf("ERROR: dwt_rxenable has status of %08X\r\n", status_reg);
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
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
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}






