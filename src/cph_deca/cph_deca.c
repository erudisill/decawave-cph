/*
 * cph_deca.c
 *
 *  Created on: Dec 2, 2015
 *      Author: ericrudisill
 */

#include <cph.h>
#include <cph_deca.h>
#include <cph_deca_state.h>
#include <cph_deca_port.h>
#include <deca_device_api.h>
#include <deca_regs.h>


void cph_deca_print_status(uint32_t count) {
	uint32_t id = 0;
	uint32 status = 0;
	uint32 status1 = 0;

	id = dwt_readdevid();

	status = dwt_read32bitreg(SYS_STATUS_ID);
	status1 = dwt_read32bitoffsetreg(SYS_STATUS_ID, 1);            // read status register
	TRACE("\r\nID:%08X  SYS_STATUS %02X %08X  count:%lu    timeouts:%d\r\n\r\n", id, status1 >> 24, status, count,
			_timeoutcount);
}

void cph_deca_init_dw(void (*txcallback)(const dwt_callback_data_t *), void (*rxcallback)(const dwt_callback_data_t *)) {
	int result;
	uint16_t panid = PAN_ID;
	uint64_t eui64 = MAC_ADDRESS;

	cph_deca_reset();

	TRACE("dwt_initialise ... ");
	result = dwt_initialise(DWT_LOADUCODE | DWT_LOADLDO | DWT_LOADTXCONFIG | DWT_LOADANTDLY | DWT_LOADXTALTRIM);
	TRACE("done  %8X\r\n", result);

//	TRACE("dwt_enableframefilter ...");
//	dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);
//	TRACE("done\r\n");

	TRACE("dwt_setautorxreenable ...");
	dwt_setautorxreenable(1);
	TRACE("done\r\n");

//	TRACE("dwt_setrxtimeout ...");
//	dwt_setrxtimeout(0);
//	TRACE("done\r\n");

	TRACE("dwt_setpanid ... ");
	dwt_setpanid(panid);
	TRACE("done\r\n");

	TRACE("dwt_seteui ...");
	dwt_seteui((uint8_t*) &eui64);
	dwt_geteui((uint8_t*) &eui64);
	TRACE("%8X%08X\r\n", (uint32_t) (eui64 >> 32), (uint32_t) (eui64 & 0x00000000FFFFFFFF));

	TRACE("dwt_setinterrupt ...");
	dwt_setinterrupt(
			DWT_INT_TFRS | DWT_INT_RFCG
					| (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/),
			1);
	TRACE("done\r\n");

	TRACE("dwt_setcallbacks ...");
	dwt_setcallbacks(txcallback, rxcallback);
	TRACE("done\r\n");

}

void cph_deca_config_dw(void) {
	/*
	 { 2,              // channel
	 DWT_PRF_64M,    // prf
	 DWT_BR_110K,    // datarate
	 9,             // preambleCode
	 DWT_PLEN_1024,  // preambleLength
	 DWT_PAC32,      // pacSize
	 1,       // non-standard SFD
	 (1025 + 64 - 32) //SFD timeout
	 },
	 */
	dwt_config_t config;

	config.chan = 2;
	config.rxCode = 9;
	config.txCode = 9;
	config.prf = DWT_PRF_64M;
	config.dataRate = DWT_BR_110K;
	config.txPreambLength = DWT_PLEN_1024;
	config.rxPAC = DWT_PAC32;
	config.nsSFD = 1;
	config.phrMode = DWT_PHRMODE_STD;
	config.sfdTO = (1025 + 64 - 32);
	config.smartPowerEn = 0;

	TRACE("dwt_configure ...");
	dwt_configure(&config, DWT_LOADANTDLY | DWT_LOADXTALTRIM);
	TRACE("done\r\n");

	// Refer to instance_common.c function instance_config()  (line 473)
	// dwt_setsmarttxpower()
	// dwt_configuretxrf()
	// dwt_setrxantennadelay()
	// dwt_settxantennadelay()

}


void cph_deca_init(void) {
	// init the state machine
	cph_deca_state_init();

	// now transition to the init state
	cph_deca_state_transition(CPH_DECA_STATE_INIT);
}

void cph_deca_run(void) {

	cph_deca_state_tick();

	// If not in active state, pull an event and launch it

	// If not in activate state and no events, what do we do?  Sleep?
}
