/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <cph.h>
#include <cph_deca_port.h>
#include <deca_regs.h>

static uint8_t buffer[128];

void cph_deca_txcallback(const dwt_callback_data_t *txd);
void cph_deca_rxcallback(const dwt_callback_data_t *txd);
void config_dw(void);
void init_dw(void);

static void print_status(uint32_t count);

static volatile int _timeoutcount = 0;
static volatile bool _timeout = false;

static volatile uint8_t _burst_count = 3;

void cph_deca_txcallback(const dwt_callback_data_t *txd) {
	uint8_t txTimeStamp[5] = { 0, 0, 0, 0, 0 };
	uint64_t timestamp = 0;
	uint32_t delay_time = 0;
	port_pin_set_output_level(LED_PIN, true);

	if (_burst_count) {
		_burst_count--;

		dwt_readtxtimestamp(txTimeStamp);
		timestamp = txTimeStamp[4];
		timestamp <<= 32;
		timestamp += (uint32) txTimeStamp[0] + ((uint32) txTimeStamp[1] << 8) + ((uint32) txTimeStamp[2] << 16)
				+ ((uint32) txTimeStamp[3] << 24);

		delay_time = dwt_readtxtimestamphi32();
		delay_time += 0x17CDC00;	// 100ms (32-bit)

		printf("cph_deca_txcallback  burst:%d  event:%02X  fctrl:%02X%02X  timeouts:%d  timestamp:%2X%08X\r\n", _burst_count,
				txd->event, txd->fctrl[0], txd->fctrl[1], _timeoutcount, (uint32_t) (timestamp >> 32), (uint32_t) (timestamp & 0x00000000FFFFFFFF));
		dwt_setdelayedtrxtime(delay_time);
		dwt_starttx(DWT_START_TX_DELAYED);
	} else {
		printf("cph_deca_txcallback BURST FINISHED\r\n");
	}
}

void cph_deca_rxcallback(const dwt_callback_data_t *rxd) {
	uint8_t rxTimeStamp[5] = { 0, 0, 0, 0, 0 };
	uint64_t timestamp = 0;

	switch (rxd->event) {

	case DWT_SIG_RX_OKAY:
		//printf("cph_deca_rxcallback  DWT_SIG_RX_OKAY  fctrl:%02X%02X  timeouts:%d\r\n", rxd->fctrl[0], rxd->fctrl[1], _timeoutcount);
		dwt_readrxtimestamp(rxTimeStamp);
		timestamp = rxTimeStamp[4];
		timestamp <<= 32;
		timestamp += (uint32) rxTimeStamp[0] + ((uint32) rxTimeStamp[1] << 8) + ((uint32) rxTimeStamp[2] << 16)
				+ ((uint32) rxTimeStamp[3] << 24);

		dwt_readrxdata(buffer, rxd->datalength, 0);
		printf("[RCV] %2X%08X %08X  ", (uint32_t) (timestamp >> 32), (uint32_t) (timestamp & 0x00000000FFFFFFFF),
		cph_get_millis());
		for (int i = 0; i < rxd->datalength; i++)
			printf("%02X ", buffer[i]);
		;
		printf("\r\n");
		dwt_rxenable(0);
		break;

	case DWT_SIG_RX_TIMEOUT:
		_timeoutcount++;
		printf("cph_deca_rxcallback  DWT_SIG_RX_TIMEOUT  fctrl:%02X%02X  timeouts:%d\r\n", rxd->fctrl[0], rxd->fctrl[1],
				_timeoutcount);
		_timeout = true;
		break;

	default:
		printf("cph_deca_rxcallback  event:%02X  fctrl:%02X%02X  timeouts:%d\r\n", rxd->event, rxd->fctrl[0], rxd->fctrl[1],
				_timeoutcount);
		break;
	}

	// check for overruns during read .. if so, dump the buffer

}

void init_dw(void) {
	int result;
	uint16_t panid = PAN_ID;
	uint64_t eui64 = MAC_ADDRESS;

	cph_deca_reset();

	printf("dwt_initialise ... ");
	result = dwt_initialise(DWT_LOADUCODE | DWT_LOADLDO | DWT_LOADTXCONFIG | DWT_LOADANTDLY | DWT_LOADXTALTRIM);
	printf("done  %8X\r\n", result);

//	printf("dwt_enableframefilter ...");
//	dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);
//	printf("done\r\n");

	printf("dwt_setautorxreenable ...");
	dwt_setautorxreenable(1);
	printf("done\r\n");

//	printf("dwt_setrxtimeout ...");
//	dwt_setrxtimeout(0);
//	printf("done\r\n");

	printf("dwt_setpanid ... ");
	dwt_setpanid(panid);
	printf("done\r\n");

	printf("dwt_seteui ...");
	dwt_seteui((uint8_t*) &eui64);
	dwt_geteui((uint8_t*) &eui64);
	printf("%8X%08X\r\n", (uint32_t) (eui64 >> 32), (uint32_t) (eui64 & 0x00000000FFFFFFFF));

	printf("dwt_setinterrupt ...");
	dwt_setinterrupt(
			DWT_INT_TFRS | DWT_INT_RFCG
					| (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/),
			1);
	printf("done\r\n");

	printf("dwt_setcallbacks ...");
	dwt_setcallbacks(cph_deca_txcallback, cph_deca_rxcallback);
	printf("done\r\n");

}

void config_dw(void) {
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

	printf("dwt_configure ...");
	dwt_configure(&config, DWT_LOADANTDLY | DWT_LOADXTALTRIM);
	printf("done\r\n");

	// Refer to instance_common.c function instance_config()  (line 473)
	// dwt_setsmarttxpower()
	// dwt_configuretxrf()
	// dwt_setrxantennadelay()
	// dwt_settxantennadelay()

}

static void print_status(uint32_t count) {
	uint32_t id = 0;
	uint32 status = 0;
	uint32 status1 = 0;

	id = dwt_readdevid();

	status = dwt_read32bitreg(SYS_STATUS_ID);
	status1 = dwt_read32bitoffsetreg(SYS_STATUS_ID, 1);            // read status register
	printf("\r\nID:%08X  SYS_STATUS %02X %08X  count:%lu    timeouts:%d\r\n\r\n", id, status1 >> 24, status, count,
			_timeoutcount);
}

int main(void) {
	system_init();

	cph_millis_init();
	cph_stdio_init();

	printf("DECAWAVE CPH       \r\n");
	printf(SOFTWARE_VER_STRING);
	printf("\r\n");

	uint32_t f = system_gclk_gen_get_hz(0);
	printf("CPU FREQ: %lu\r\n", f);

	// Blink LED for 5 seconds
	for (int i = 0; i < (5 * 4); i++) {
		port_pin_set_output_level(LED_PIN, false);
		cph_millis_delay(125);
		port_pin_set_output_level(LED_PIN, true);
		cph_millis_delay(125);
	}

	// init gpio (interrupts disabled)
	cph_deca_init_gpio();

	// init spi
	printf("cph_deca_spi_init\r\n");
	cph_deca_spi_init();

	// init dw
	init_dw();

	// config dw (rf, interrupt masks, etc.)
	config_dw();

	// enable dw isr gpio
	cph_deca_isr_enable();

	system_interrupt_enable_global();

#if 0
	printf("dwt_rxenable ...");
	dwt_rxenable(0);
	printf("done\r\n");

	// process events
	uint32_t count = 0;
	while (1) {
		for (int i=0;i<5;i++) {
			if (_timeout) {
				_timeout = false;
				dwt_forcetrxoff();
				dwt_rxreset();
				cph_millis_delay(1);
				dwt_rxenable(0);
			}
			cph_millis_delay(1000);
		}
		print_status(count++);
	}
#else
	uint32_t count = 0;
	uint32_t delay_time = 0;
	buffer[0] = 'C';
	buffer[1] = 'P';
	buffer[2] = 'H';
	while (1) {
		cph_millis_delay(1000);
		print_status(count++);
		_burst_count = 3;
		dwt_writetxdata(3, buffer, 0);
		dwt_writetxfctrl(3, 0);
		if (count > 1) {
			delay_time = dwt_readsystimestamphi32();
			delay_time += +0x17CDC00;	// 100ms
			dwt_setdelayedtrxtime(delay_time);
			dwt_starttx(DWT_START_TX_DELAYED);
		} else {
			dwt_starttx(DWT_START_TX_IMMEDIATE);
		}
		port_pin_set_output_level(LED_PIN, false);
	}
#endif

}
