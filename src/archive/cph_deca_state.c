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

static cph_deca_state_t * cph_deca_states;

static cph_deca_state_t * cph_deca_state_current;



volatile int _timeoutcount = 0;
volatile bool _timeout = false;
volatile int _ovr_count = 0;
volatile bool _ovr = false;
volatile bool _err = false;
volatile int _err_count = 0;

static cph_queue_info_t * _event_queue;


void cph_deca_state_init(cph_deca_state_t states[], cph_queue_info_t * q) {
	// point to the state machine
	cph_deca_states = states;

	// assign the queue
	_event_queue = q;

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

void cph_deca_txcallback(const dwt_callback_data_t *txd) {
	uint8_t txTimeStamp[5] = { 0, 0, 0, 0, 0 };
	uint64_t timestamp = 0;
	uint32_t delay_time = 0;
	cph_deca_event_t event;

	if (cph_deca_state_current->state_id != CPH_DECA_STATE_TX) {
		TRACE("cph_deca_txcallback: INVALID STATE!\r\n");
		_err = true;
		_err_count++;
		dwt_forcetrxoff();
		//dwt_rxreset();
		event.event_id = txd->event;
		event.timestamp_sys = cph_get_millis();
		cph_queue_push(_event_queue, &event);
		return;
	}

	switch (txd->event) {
	case DWT_SIG_TX_DONE:
		dwt_readtxtimestamp(txTimeStamp);
		timestamp = txTimeStamp[4];
		timestamp <<= 32;
		timestamp += (uint32) txTimeStamp[0] + ((uint32) txTimeStamp[1] << 8) + ((uint32) txTimeStamp[2] << 16)
				+ ((uint32) txTimeStamp[3] << 24);

		event.event_id = DWT_SIG_TX_DONE;
		event.data_len = txd->datalength;
		event.timestamp_sys = cph_get_millis();
		event.timestamp_dw = timestamp;

		cph_queue_push(_event_queue, &event);

		break;
	default:
		event.event_id = txd->event;
		event.timestamp_sys = cph_get_millis();
		cph_queue_push(_event_queue, &event);
		break;
	}
}

void cph_deca_rxcallback(const dwt_callback_data_t *rxd) {

	uint8_t rxTimeStamp[5] = { 0, 0, 0, 0, 0 };
	uint64_t timestamp = 0;
	cph_deca_event_t event;

	if (cph_deca_state_current->state_id != CPH_DECA_STATE_RX) {
		TRACE("cph_deca_rxcallback: INVALID STATE!");
		_err = true;
		_err_count++;
		dwt_forcetrxoff();
		//dwt_rxreset();
		event.event_id = rxd->event;
		event.timestamp_sys = cph_get_millis();
		cph_queue_push(_event_queue, &event);
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

		cph_queue_push(_event_queue, &event);

		if(dwt_checkoverrun()) //the overrun has occured while we were reading the data - dump the frame/data
		{
			_ovr_count++;
			_ovr = true;
		}
		break;

	case DWT_SIG_RX_TIMEOUT:
		event.event_id = DWT_SIG_RX_TIMEOUT;
		event.timestamp_sys = cph_get_millis();
		cph_queue_push(_event_queue, &event);

		_timeoutcount++;
		_timeout = true;
		break;

	default:
		event.event_id = rxd->event;
		event.timestamp_sys = cph_get_millis();
		cph_queue_push(_event_queue, &event);
		break;
	}

}






