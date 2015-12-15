/*
 * cph.h
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_H_
#define SRC_CPH_H_

#include <asf.h>

#include <cph_millis.h>
#include <cph_stdio.h>
#include <cph_queue.h>
#include <cph_config.h>
#include <cph_utils.h>

#define FW_MAJOR				0x01
#define FW_MINOR				0x01

#define	ANCHOR
//#define TAG

#define TRACE(...)				printf(__VA_ARGS__)


// 								 'C''P'
#define MAC_PAN_ID				0x4350
#define MAC_ANCHOR_ID			0x4157
#define MAC_TAG_ID				0x4556
#define MAC_FC					0x8841

#ifdef ANCHOR
#define APP_NAME  				"CPH ANCHOR Version %2X.%02X\r\n"
#define MAC_ADDRESS				0x4350010000000077
#define MAC_SHORT				MAC_ANCHOR_ID
#define app_run					anchor_run
void anchor_run(void);

#else
#define APP_NAME  				"CPH TAG Version %2X.%02X\r\n"
#define MAC_ADDRESS				0x4350010000000078
#define MAC_SHORT				MAC_TAG_ID
#define app_run					tag_run
void tag_run(void);

#endif


#define DW_CONFIG		\
		{																													\
		    2,               /* Channel number. */																			\
		    DWT_PRF_64M,     /* Pulse repetition frequency. */																\
		    DWT_PLEN_128,    /* Preamble length. */																			\
		    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */										\
		    9,               /* TX preamble code. Used in TX only. */														\
		    9,               /* RX preamble code. Used in RX only. */														\
		    0,               /* Use non-standard SFD (Boolean) */															\
		    DWT_BR_6M8,      /* Data rate. */																				\
		    DWT_PHRMODE_STD, /* PHY header mode. */																			\
		    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */				\
		}


/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds.  For TAG */
#define POLL_TX_TO_RESP_RX_DLY_UUS 330
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 370

/* Delay between frames, in UWB microseconds.  For ANCHOR */
#define POLL_RX_TO_RESP_TX_DLY_UUS 660


/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547


// Min Number of anchors to range with
#define MIN_ANCHORS		3

// Delay to start listening after discover
#define DISCOVER_TX_TO_ANNOUNCE_RX_DELAY_UUS	400
#define DISCOVER_RX_TO_ANNOUNCE_TX_DELAY_UUS	460

enum {
	CPH_OK = 0,
	CPH_ERROR,
	CPH_BAD_FRAME,
	CPH_BAD_LENGTH
};

#define	FUNC_POLL		0xE0
#define FUNC_RESP		0xE1
#define FUNC_DISCOVER	0xE2
#define FUNC_ANNOUNCE	0xE3

#define PACKED	__attribute__((packed))

typedef struct PACKED {
	uint16_t mac_frameControl;
	uint8_t mac_sequence;
	uint16_t mac_panid;
	uint16_t mac_dest;
	uint16_t mac_source;
	uint8_t functionCode;
	uint16_t mac_cs;
} msg_poll;

typedef struct PACKED {
	uint16_t mac_frameControl;
	uint8_t mac_sequence;
	uint16_t mac_panid;
	uint16_t mac_dest;
	uint16_t mac_source;
	uint8_t functionCode;
	uint32_t pollRxTs;
	uint32_t respTxTs;
	uint16_t mac_cs;
} msg_resp;

typedef struct PACKED {
	uint16_t mac_frameControl;
	uint8_t mac_sequence;
	uint16_t mac_panid;
	uint16_t mac_dest;
	uint16_t mac_source;
	uint8_t functionCode;
	uint16_t mac_cs;
} msg_discover;

typedef struct PACKED {
	uint16_t mac_frameControl;
	uint8_t mac_sequence;
	uint16_t mac_panid;
	uint16_t mac_dest;
	uint16_t mac_source;
	uint8_t functionCode;
	uint16_t mac_cs;
} msg_announce;

typedef struct PACKED {
	uint16_t shortid;
	double range;
} anchor_range_t;




typedef struct PACKED {
	uint8_t magic[4];
	uint8_t hw_major;
	uint8_t hw_minor;
	uint8_t fw_major;
	uint8_t fw_minor;
	uint16_t panid;
	uint16_t shortid;
} cph_config_t;

extern cph_config_t * cph_config;


#endif /* SRC_CPH_H_ */
