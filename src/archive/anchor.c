/*
 * anchor.c
 *
 *  Created on: Dec 4, 2015
 *      Author: ericrudisill
 */

#include <cph.h>
#include <cph_deca.h>
#include <cph_deca_state.h>
#include <cph_deca_port.h>
#include <anchor.h>

static void anchor_init_run(cph_deca_state_t * curr_state);
static void anchor_rx_enter(cph_deca_state_t * prev_state);
static void anchor_rx_run(cph_deca_state_t * curr_state);

static cph_deca_state_t anchor_states[] =
{
		{	CPH_DECA_STATE_NONE,			0,	0,								0,								0 },
		{	CPH_DECA_STATE_INIT,			0,	0,								anchor_init_run,				0 },
		{	CPH_DECA_STATE_RX,				0,	anchor_rx_enter,				anchor_rx_run,					0 },
		{	CPH_DECA_STATE_TX,				0,	0,								0,								0 }
};

static cph_queue_info_t event_queue;
static cph_deca_event_t event_queue_data[ANCHOR_EVENTS_MAX];

static volatile uint64_t _last_rx_timestamp = 0;


void anchor_init(void) {
	cph_queue_init(&event_queue, sizeof(cph_deca_event_t), ANCHOR_EVENTS_MAX, event_queue_data);
	cph_deca_init(anchor_states, &event_queue);
}


static void anchor_init_run(cph_deca_state_t * curr_state) {

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
	cph_deca_state_transition(CPH_DECA_STATE_RX);
}

static void anchor_rx_enter(cph_deca_state_t * prev_state) {
	TRACE("dwt_rxenable ...");
	dwt_rxenable(0);
	TRACE("done\r\n");
}

static void anchor_rx_run(cph_deca_state_t * curr_state) {
	uint64_t delta_timestamp = 0;
	int32_t delta_int = 0;
	double tof = 0;
	double dist = 0;
	cph_deca_event_t event;

	// check for error conditions
	if (_timeout || _ovr || _err) {
		TRACE("ERROR! Resetting\r\n");
		_timeout = false;
		_ovr = false;
		_err = false;
		dwt_forcetrxoff();
		//dwt_rxreset();
		cph_millis_delay(1);
		dwt_rxenable(0);
	}

	if (event_queue.count > 0) {

		cph_queue_pop(&event_queue, &event);

		switch (event.event_id) {

		case DWT_SIG_RX_OKAY:
			delta_int = (int32_t)(event.timestamp_dw - _last_rx_timestamp - 0x17CDC0000);
			_last_rx_timestamp = event.timestamp_dw;
			if (delta_int < 0) {
				delta_int *= -1;
			}

			tof = (double)delta_int;
			tof *= (double)DWT_TIME_UNITS;
			dist = tof * SPEED_OF_LIGHT;

//			TRACE("[RCV] %2X%08X  delta:%08X  tof:%f  dist:%f  ovr:%d   ", (uint32_t) (event.timestamp_dw >> 32), (uint32_t) (event.timestamp_dw & 0x00000000FFFFFFFF), delta_int, tof, dist, _ovr_count);
			TRACE("[RCV] %2X%08X  delta:%08X  depth:%d  ovr:%d   ", (uint32_t) (event.timestamp_dw >> 32), (uint32_t) (event.timestamp_dw & 0x00000000FFFFFFFF), delta_int, _ovr_count, event_queue.count);
			for (int i = 0; i < event.data_len; i++)
				TRACE("%02X ", event.data[i]);
			;
			TRACE("\r\n");

			dwt_rxenable(0);
			break;

		case DWT_SIG_RX_TIMEOUT:
			TRACE("cph_deca_state_rx_run  DWT_SIG_RX_TIMEOUT  timeouts:%d\r\n", _timeoutcount);
			dwt_rxenable(0);
			break;

		default:
			TRACE("cph_deca_state_rx_run  DEFAULT event:%02X  timeouts:%d\r\n", event.event_id, 	_timeoutcount);
			dwt_rxenable(0);
			break;

		}
	}
}
