/*
 * cph_deca_state.c
 *
 *  Created on: Dec 3, 2015
 *      Author: ericrudisill
 */

#include <cph.h>
#include <cph_deca.h>
#include <cph_deca_state.h>
#include <cph_deca_port.h>
#include <deca_device_api.h>

static cph_deca_state_t cph_deca_states[] =
{
		{	CPH_DECA_STATE_NONE,			0,	0,								0,								0 },
		{	CPH_DECA_STATE_INIT,			0,	0,								cph_deca_state_init_run,		0 },
		{	CPH_DECA_STATE_RX,				0,	cph_deca_state_rx_enter,		cph_deca_state_rx_run,			0 },
		{	CPH_DECA_STATE_TX,				0,	0,								0,								0 }
};

static cph_deca_state_t * cph_deca_state_current;



volatile int _timeoutcount = 0;
volatile bool _timeout = false;
volatile int _ovr_count = 0;
volatile bool _ovr = false;
volatile bool _err = false;

#define BURST_COUNT			3
static volatile uint8_t _burst_count = BURST_COUNT;
static volatile uint64_t _last_rx_timestamp = 0;

//static uint8_t buffer[128];

static cph_queue_info_t event_queue;
static cph_deca_event_t event_queue_data[CPH_DECA_EVENTS_MAX];


void cph_deca_state_init(void) {
	// initialize current state
	cph_deca_state_current = &cph_deca_states[CPH_DECA_STATE_NONE];
}

void cph_deca_state_transition(int new_state) {
	int old_state = cph_deca_state_current->state_id;

	// call exit of current state
	if (cph_deca_state_current->exit_f) {
		cph_deca_state_current->exit_f(&cph_deca_states[new_state]);
	}

	// set new state
	cph_deca_state_current = &cph_deca_states[new_state];

	// call enter of new state
	if (cph_deca_state_current->enter_f) {
		cph_deca_state_current->timestamp = cph_get_millis();
		cph_deca_state_current->enter_f(&cph_deca_states[old_state]);
	}
}


void cph_deca_state_tick(void) {
	// Tick the machine
	if (cph_deca_state_current->run_f) {
		cph_deca_state_current->run_f(cph_deca_state_current);
	}
}

static void cph_deca_txcallback(const dwt_callback_data_t *txd) {
	uint8_t txTimeStamp[5] = { 0, 0, 0, 0, 0 };
	uint64_t timestamp = 0;
	uint32_t delay_time = 0;

	if (cph_deca_state_current->state_id != CPH_DECA_STATE_TX) {
		TRACE("cph_deca_txcallback: INVALID STATE!");
		_err = true;
		return;
	}


	port_pin_set_output_level(LED_PIN, true);

	if (_burst_count) {
		dwt_readtxtimestamp(txTimeStamp);
		timestamp = txTimeStamp[4];
		timestamp <<= 32;
		timestamp += (uint32) txTimeStamp[0] + ((uint32) txTimeStamp[1] << 8) + ((uint32) txTimeStamp[2] << 16)
				+ ((uint32) txTimeStamp[3] << 24);

		delay_time = dwt_readtxtimestamphi32();
		delay_time += 0x17CDC00;	// 100ms (32-bit)

		TRACE("cph_deca_txcallback  burst:%d  event:%02X  fctrl:%02X%02X  timeouts:%d  timestamp:%2X%08X\r\n", _burst_count,
				txd->event, txd->fctrl[0], txd->fctrl[1], _timeoutcount, (uint32_t) (timestamp >> 32), (uint32_t) (timestamp & 0x00000000FFFFFFFF));

		_burst_count--;
		dwt_setdelayedtrxtime(delay_time);
		dwt_starttx(DWT_START_TX_DELAYED);
	} else {
		TRACE("cph_deca_txcallback BURST FINISHED\r\n");
	}
}

static void cph_deca_rxcallback(const dwt_callback_data_t *rxd) {

	uint8_t rxTimeStamp[5] = { 0, 0, 0, 0, 0 };
	uint64_t timestamp = 0;
	cph_deca_event_t event;

	if (cph_deca_state_current->state_id != CPH_DECA_STATE_RX) {
		TRACE("cph_deca_rxcallback: INVALID STATE!");
		_err = true;
		return;
	}


	switch (rxd->event) {

	case DWT_SIG_RX_OKAY:
		dwt_readrxtimestamp(rxTimeStamp);
		timestamp = rxTimeStamp[4];
		timestamp <<= 32;
		timestamp += (uint32) rxTimeStamp[0] + ((uint32) rxTimeStamp[1] << 8) + ((uint32) rxTimeStamp[2] << 16)
				+ ((uint32) rxTimeStamp[3] << 24);


		event.event_id = DWT_SIG_RX_OKAY;
		event.data_len = rxd->datalength;
		event.timestamp_sys = cph_get_millis();
		event.timestamp_dw = timestamp;
		dwt_readrxdata(event.data, rxd->datalength, 0);

		cph_queue_push(&event_queue, &event);

		if(dwt_checkoverrun()) //the overrun has occured while we were reading the data - dump the frame/data
		{
			_ovr_count++;
			_ovr = true;
		}
		break;

	case DWT_SIG_RX_TIMEOUT:
		event.event_id = DWT_SIG_RX_TIMEOUT;
		event.timestamp_sys = cph_get_millis();
		cph_queue_push(&event_queue, &event);

		_timeoutcount++;
		_timeout = true;
		break;

	default:
		event.event_id = rxd->event;
		event.timestamp_sys = cph_get_millis();
		cph_queue_push(&event_queue, &event);
		break;
	}

}

static void cph_deca_state_init_run(cph_deca_state_t * curr_state) {

	cph_queue_init(&event_queue, sizeof(cph_deca_event_t), CPH_DECA_EVENTS_MAX, event_queue_data);

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



static void cph_deca_state_rx_enter(cph_deca_state_t * prev_state) {
	TRACE("dwt_rxenable ...");
	dwt_rxenable(0);
	TRACE("done\r\n");
}

static void cph_deca_state_rx_run(cph_deca_state_t * curr_state) {
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
		dwt_rxreset();
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
			break;

		default:
			TRACE("cph_deca_state_rx_run  DEFAULT event:%02X  imeouts:%d\r\n", event.event_id, 	_timeoutcount);
			break;

		}
	}
}



