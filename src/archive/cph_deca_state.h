/*
 * cph_deca_state.h
 *
 *  Created on: Dec 3, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_DECA_CPH_DECA_STATE_H_
#define SRC_CPH_DECA_CPH_DECA_STATE_H_

//////////////////////////////////////////////////////////////////////////////////////////////
//
// Private header file to be included ONLY by cph_deca.c, due to definitions of state
// structure arrays.
//
//////////////////////////////////////////////////////////////////////////////////////////////

#include <deca_device_api.h>

#define CPH_DECA_DATA_LEN_MAX	128

enum {
	CPH_DECA_STATE_NONE = 0,
	CPH_DECA_STATE_INIT,
	CPH_DECA_STATE_RX,
	CPH_DECA_STATE_TX
};

enum {
	CPH_DECA_STATE_HOOK_NONE = 0,
	CPH_DECA_STATE_HOOK_ENTER,
	CPH_DECA_STATE_HOOK_RUN,
	CPH_DECA_STATE_HOOK_EXIT
};

typedef struct {
	int			event_id;
	int 		hook_id;
	uint32_t	timestamp_sys;
	uint64_t	timestamp_dw;
	int			data_len;
	uint8_t		data[CPH_DECA_DATA_LEN_MAX];
} cph_deca_event_t;

// cph_deca_state_f - function pointer to call a state handler
//  void *		- pointer to previous state or upcoming state or null
typedef void (*cph_deca_state_f)(void *);


typedef struct {
	int					state_id;			// state enum
	uint32_t			timestamp;			// timestamp (millis) of enter_f start
	cph_deca_state_f	enter_f;			// called during state transition from previous state
	cph_deca_state_f	run_f;				// called each tick
	cph_deca_state_f	exit_f;				// called before state transition to next state
} cph_deca_state_t;




void cph_deca_state_init(cph_deca_state_t states[], cph_queue_info_t * q);
void cph_deca_state_transition(int new_state);
void cph_deca_state_tick(void);

void cph_deca_txcallback(const dwt_callback_data_t *txd);
void cph_deca_rxcallback(const dwt_callback_data_t *txd);


extern volatile int _timeoutcount;
extern volatile bool _timeout;
extern volatile int _ovr_count;
extern volatile bool _ovr;
extern volatile bool _err;
extern volatile int _err_count;

#endif /* SRC_CPH_DECA_CPH_DECA_STATE_H_ */
