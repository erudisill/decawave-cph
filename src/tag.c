/*
 * tag.c
 *
 *  Created on: Dec 7, 2015
 *      Author: ericrudisill
 */

#include <cph.h>
#include <cph_deca.h>
#include <cph_deca_state.h>
#include <cph_deca_port.h>
#include <tag.h>

static void tag_init_run(cph_deca_state_t * curr_state);
static void tag_tx_enter(cph_deca_state_t * prev_state);
static void tag_tx_run(cph_deca_state_t * curr_state);

static cph_deca_state_t tag_states[] =
{
		{	CPH_DECA_STATE_NONE,			0,	0,								0,								0 },
		{	CPH_DECA_STATE_INIT,			0,	0,								tag_init_run,					0 },
		{	CPH_DECA_STATE_RX,				0,	0,								0,								0 },
		{	CPH_DECA_STATE_TX,				0,	tag_tx_enter,					tag_tx_run,						0 }
};


static int burst_count = TAG_BURST_COUNT;

static cph_queue_info_t event_queue;
static cph_deca_event_t event_queue_data[TAG_EVENTS_MAX];

static uint8_t burst_id;
static uint8_t buffer[10];


enum {
	TAG_TX_WAIT_BURST = 0,
	TAG_TX_WAIT_SEND
};

static int local_state;

void tag_init(void) {
	cph_queue_init(&event_queue, sizeof(cph_deca_event_t), TAG_EVENTS_MAX, event_queue_data);
	cph_deca_init(tag_states, &event_queue);
}

static void tag_init_run(cph_deca_state_t * curr_state) {

	// init gpio (interrupts disabled)
	cph_deca_init_gpio();

	// init spi
	cph_deca_spi_init();

	// init dw
	cph_deca_init_dw(cph_deca_txcallback, cph_deca_rxcallback);

	// config dw (rf, interrupt masks, etc.)
	cph_deca_config_dw();

	// enable dw isr gpio
	cph_deca_isr_enable();

	// done w/ init, so transition to RX
	cph_deca_state_transition(CPH_DECA_STATE_TX);
}
static void tag_tx_enter(cph_deca_state_t * prev_state) {
	buffer[0] = 'C';
	buffer[1] = 'P';
	buffer[2] = 'H';
	local_state = TAG_TX_WAIT_BURST;
	burst_id = 0;
}

static void tag_tx_run(cph_deca_state_t * curr_state) {
	uint8_t txTimeStamp[5] = { 0, 0, 0, 0, 0 };
	uint64_t timestamp = 0;
	uint32_t delay_time = 0;
	uint32_t elapsed;
	cph_deca_event_t event;

	elapsed = cph_get_millis() - curr_state->timestamp;

	switch (local_state) {
	case TAG_TX_WAIT_BURST:
		if (elapsed > TAG_BURST_DELAY) {
			port_pin_set_output_level(LED_PIN, false);
			curr_state->timestamp = cph_get_millis();
			local_state = TAG_TX_WAIT_SEND;
			burst_count = TAG_BURST_COUNT;
			burst_id++;
			buffer[3] = burst_id;
			buffer[4] = burst_count;
			dwt_writetxdata(5+2, buffer, 0);
			dwt_writetxfctrl(5+2, 0);
			dwt_starttx(DWT_START_TX_IMMEDIATE);
		}
		break;

	case TAG_TX_WAIT_SEND:

		if (elapsed > 200) {
			TRACE("TAG_TX_WAIT_SEND:  application timeout .. Restart TAG_TX_WAIT_BURST\r\n");
			curr_state->timestamp = cph_get_millis();
			local_state = TAG_TX_WAIT_BURST;
		} else {
			if (event_queue.count > 0) {

				cph_queue_pop(&event_queue, &event);

				switch (event.event_id) {
				case DWT_SIG_TX_DONE:
					port_pin_set_output_level(LED_PIN, true);

					TRACE("TAG_TX_WAIT_SEND:  burst:%d  timestamp:%2X%08X   ", burst_count,
							(uint32_t ) (event.timestamp_dw >> 32), (uint32_t ) (event.timestamp_dw & 0x00000000FFFFFFFF));
					TRACE("%02X ", buffer[0]);
					TRACE("%02X ", buffer[1]);
					TRACE("%02X ", buffer[2]);
					TRACE("%02X ", buffer[3]);
					TRACE("%02X ", buffer[4]);
					TRACE("\r\n");

					burst_count--;

					if (burst_count) {
						port_pin_set_output_level(LED_PIN, false);
						delay_time = (event.timestamp_dw >> 8);		// reduce 40-bit timestamp down to 32
						delay_time += TAG_SEND_DELAY;
						buffer[4] = burst_count;
						dwt_writetxdata(5+2, buffer, 0);
						dwt_writetxfctrl(5+2, 0);
						dwt_setdelayedtrxtime(delay_time);
						dwt_starttx(DWT_START_TX_DELAYED);
					} else {
						TRACE("\r\n");
						curr_state->timestamp = cph_get_millis();
						local_state = TAG_TX_WAIT_BURST;
					}
					break;
				default:
					curr_state->timestamp = cph_get_millis();
					local_state = TAG_TX_WAIT_BURST;
					break;
				}
			}
		}

		break;

	default:
		TRACE("tag_tx_run: unknown state %d   HALT\r\n", local_state);
		while(1){}
		break;
	}



//	uint32_t count = 0;
//	uint32_t delay_time = 0;
//	while (1) {
//		cph_millis_delay(1000);
//		print_status(count++);
//		_burst_count = TAG_BURST_COUNT;
//		dwt_writetxdata(3, buffer, 0);
//		dwt_writetxfctrl(3, 0);
//		if (count > 1) {
//			delay_time = dwt_readsystimestamphi32();
//			delay_time += +0x17CDC00;	// 100ms
//			dwt_setdelayedtrxtime(delay_time);
//			dwt_starttx(DWT_START_TX_DELAYED);
//		} else {
//			dwt_starttx(DWT_START_TX_IMMEDIATE);
//		}
//		port_pin_set_output_level(LED_PIN, false);
//	}
}
