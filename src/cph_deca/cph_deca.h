/*
 * cph_deca.h
 *
 *  Created on: Jan 11, 2016
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_DECA_CPH_DECA_H_
#define SRC_CPH_DECA_CPH_DECA_H_

#include <cph_deca_port.h>
#include <deca_regs.h>
#include <deca_device_api.h>

// 								 'C''P'
#define MAC_PAN_ID				0x4350
#define MAC_ANCHOR_ID			0x4157
#define MAC_TAG_ID				0x4556
#define MAC_FC					0x8841
#define MAC_FC_ACK				0x8861

#if defined(ANCHOR)
#define APP_NAME  				"\r\nCPH ANCHOR Version %2X.%02X\r\n"
#define MAC_ADDRESS				0x4350010000000077
#define MAC_SHORT				MAC_ANCHOR_ID
#define app_run					anchor_run
void anchor_run(void);

#elif defined(TAG)
#define APP_NAME  				"\r\nCPH TAG Version %2X.%02X\r\n"
#define MAC_ADDRESS				0x4350010000000078
#define MAC_SHORT				MAC_TAG_ID
#define app_run					tag_run
//#define app_run					tag_burst_run
void tag_run(void);
void tag_burst_run(void);

#else
#error "ANCHOR or TAG must be defined"
#endif



#if defined(RANGE_METHOD_DS_TWR)

#define DW_CONFIG		\
{																															\
    2,               			/* Channel number. */																		\
    DWT_PRF_64M,     			/* Pulse repetition frequency. */															\
    DWT_PLEN_1024,  		 	/* Preamble length. */																		\
    DWT_PAC32,      		 	/* Preamble acquisition chunk size. Used in RX only. */										\
    9,              		 	/* TX preamble code. Used in TX only. */													\
    9,              		 	/* RX preamble code. Used in RX only. */													\
    1,             		 	 	/* Use non-standard SFD (Boolean) */														\
    DWT_BR_110K,    		 	/* Data rate. */																			\
    DWT_PHRMODE_STD, 			/* PHY header mode. */																		\
    (1025 + 64 - 32) 			/* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */			\
}

#else

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

#endif



/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 	5

/* Inter-poll delay period, in milliseconds */
#define POLL_DELAY_MS 	200

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

////////////ADJUSTED
//#define TX_ANT_DLY 16486
//#define RX_ANT_DLY 16486

//#define TX_ANT_DLY 0
//#define RX_ANT_DLY (16486 * 2)


/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536


#if defined(RANGE_METHOD_DS_TWR)

/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 150
#define POLL_TX_TO_RESP_RX_DLY_UUS 100
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
//#define RESP_RX_TO_FINAL_TX_DLY_UUS		3100
#define RESP_RX_TO_FINAL_TX_DLY_UUS		3500
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
//#define POLL_RX_TO_RESP_TX_DLY_UUS		2600
#define POLL_RX_TO_RESP_TX_DLY_UUS		3000
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS		500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS			3300

#else

///* Delay between frames, in UWB microseconds.  For TAG */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 330
///* Receive response timeout. See NOTE 5 below. */
//#define RESP_RX_TIMEOUT_UUS 370

/* Delay between frames, in UWB microseconds.  For TAG */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 900

/* Delay between frames, in UWB microseconds.  For ANCHOR */
//#define POLL_RX_TO_RESP_TX_DLY_UUS 660
#define POLL_RX_TO_RESP_TX_DLY_UUS 550

/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100

#endif


/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547


// Min Number of anchors to range with - if this changes, so should ANCHORS_MASK
//#define ANCHORS_MIN		4
//#define ANCHORS_MIN		3
#define ANCHORS_MIN		1

// Used for tracking status of anchor ids (by bitmask) during discovery and poll
//#define ANCHORS_MASK	0x0F
//#define ANCHORS_MASK	0x07
#define ANCHORS_MASK	0x01

// Anchor refresh interval
#define ANCHORS_REFRESH_INTERVAL	10000

// Coord announce startup burst repeat count
#define COORD_ANNOUNCE_START_BURST	10

// Coord announce period in ms
#define COORD_ANNOUNCE_INTERVAL		7000

// Max ranges before poll timeout - keeps from blasting radio when an anchor is not responding
#define MAX_RANGES_BEFORE_POLL_TIMEOUT	5

// Max number of tags to pair with
#define MAX_TAGS		32

// Lifetime of tag pairing
#define PAIR_LIFETIME	5000

// Delay to start listening after discover
#define DISCOVER_TX_TO_ANNOUNCE_RX_DELAY_UUS	400
#define DISCOVER_RX_TO_ANNOUNCE_TX_DELAY_UUS	460

enum {
	CPH_OK = 0,
	CPH_ERROR,
	CPH_BAD_FRAME,
	CPH_BAD_LENGTH,
	CPH_DUPLICATE
};

enum {
	CPH_MODE_ANCHOR = 0x01,
	CPH_MODE_TAG = 0x02,
	CPH_MODE_COORD = 0x80
};

#define	FUNC_RANGE_POLL				0xA0
#define FUNC_RANGE_RESP				0xA1
#define FUNC_RANGE_FINA				0xA2
#define FUNC_RANGE_RESU				0xA3
#define FUNC_RANGE_REPO				0xA4

#define FUNC_RANGE_BURST			0xAF

#define FUNC_DISC_ANNO				0xB2
#define FUNC_DISC_REPLY				0xB3
#define FUNC_PAIR_RESP				0xB4

#define FUNC_COORD_ANNO				0xC5

#define CPH_MAX_MSG_SIZE		128

#define PACKED	__attribute__((packed))

typedef struct PACKED {
	uint16_t ctl;
	uint8_t seq;
	uint16_t panid;
	uint16_t dest;
	uint16_t source;
	uint8_t functionCode;
} cph_deca_msg_header_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t mac_cs;
} cph_deca_msg_range_request_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint32_t pollRxTs;
	uint32_t responseTxTs;
	uint16_t mac_cs;
} cph_deca_msg_range_response_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint32_t pollTxTs;
	uint32_t responseTxTs;
	uint32_t responseRxTs;
	uint32_t finalTxTs;
	uint16_t mac_cs;
} cph_deca_msg_range_final_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t mac_cs;
} cph_deca_msg_discover_announce_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t coordid;
	uint16_t mac_cs;
} cph_deca_msg_discover_reply_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t mac_cs;
} cph_deca_msg_pair_response_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t coordid;
	uint16_t mac_cs;
} cph_deca_msg_coord_announce_t;

typedef struct PACKED {
	uint16_t shortid;
	double range;
} cph_deca_anchor_range_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	cph_deca_anchor_range_t range;
	uint16_t mac_cs;
} cph_deca_msg_range_result_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint8_t numranges;
	cph_deca_anchor_range_t ranges[ANCHORS_MIN];		//TODO: Make this dynamic
	uint16_t mac_fs;
} cph_deca_msg_range_report_t;

typedef struct PACKED {
	uint16_t shortid;
	uint32_t paired_ts;
} cph_deca_pair_info_t;



inline uint32_t cph_deca_wait_for_tx_finished(void) {
	uint32_t status_reg;
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS)) {
	};
	return status_reg;
}

inline uint32_t cph_deca_wait_for_rx_finished(void) {
	uint32_t status_reg;
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
	};
	return status_reg;
}

//inline uint32_t cph_deca_wait_for_rx_finished_or_signal(volatile uint32_t * signal) {
//	uint32_t status_reg;
//	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
//		if (*signal != 0) {
//			port_pin_toggle_output_level(LED_PIN);
//			break;
//		}
//	};
//	return status_reg;
//}

void cph_deca_load_frame(cph_deca_msg_header_t * hdr, uint16_t size);
cph_deca_msg_header_t * cph_deca_read_frame(uint8_t * rx_buffer, uint32_t *frame_len);
uint32_t cph_deca_send_immediate();
uint32_t cph_deca_send_delayed();
uint32_t cph_deca_send_delayed_response_expected();
uint32_t cph_deca_send_response_expected();
void cph_deca_init_device();
void cph_deca_init_network(uint16_t panid, uint16_t shortid);
#endif /* SRC_CPH_DECA_CPH_DECA_H_ */
