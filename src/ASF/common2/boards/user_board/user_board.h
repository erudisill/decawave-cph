/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup group_common_boards
 * \defgroup user_board_group User board
 *
 * @{
 */

void system_board_init(void);

/** Name string macro */
#define BOARD_NAME                "DW_ALPHA"

//#define BOARD_REV_A02
#define BOARD_REV_A03

#ifdef BOARD_REV_A02
#define BOARD_REV_MAJOR		0x0A
#define BOARD_REV_MINOR		0x02
#endif
#ifdef BOARD_REV_A03
#define BOARD_REV_MAJOR		0x0A
#define BOARD_REV_MINOR		0x03
#endif



/************************************************************	STDIO	*/
#ifdef CONF_BOARD_LED

#define LED_PIN			PIN_PA27
#define LED_PIN_CONFIG	{ PORT_PIN_DIR_OUTPUT, PORT_PIN_PULL_NONE, false }

#endif




/************************************************************	STDIO	*/
#ifdef CONF_BOARD_STDIO

#define STDIO_HW		SERCOM0
#define STDIO_BAUD 		115200
#define STDIO_MUX 		USART_RX_1_TX_0_XCK_1
#define STDIO_PAD0		PINMUX_PA04D_SERCOM0_PAD0
#define STDIO_PAD1		PINMUX_PA05D_SERCOM0_PAD1
#define STDIO_PAD2		PINMUX_UNUSED
#define STDIO_PAD3		PINMUX_UNUSED

#endif



/************************************************************	STDIO	*/
#ifdef CONF_BOARD_VSENSE

#define VSENSE_PIN			PIN_PA03
#define VSENSE_PIN_CONFIG	{ PORT_PIN_DIR_INPUT, PORT_PIN_PULL_NONE, false }

#endif




/************************************************************	DECAWAVE	*/
#ifdef CONF_BOARD_DW

#define DW_SPI_HW				SERCOM1
#define DW_SPI_SS_PIN			PIN_PA18
#define DW_SPI_MUX				SPI_SIGNAL_MUX_SETTING_D
#define DW_SPI_PAD0				PINMUX_PA16C_SERCOM1_PAD0
#define DW_SPI_PAD1				PINMUX_PA17C_SERCOM1_PAD1
#define DW_SPI_PAD2				PINMUX_UNUSED
#define DW_SPI_PAD3				PINMUX_PA19C_SERCOM1_PAD3
#define DW_SPI_TRANSFER_MODE	SPI_TRANSFER_MODE_0

#ifdef BOARD_REV_A02
#define DW_IRQ_LINE				0
#define DW_IRQ_PIN				PIN_PA00
#define DW_IRQ_PIN_MUX			PIN_PA00A_EIC_EXTINT0
#endif
#ifdef BOARD_REV_A03
#define DW_IRQ_LINE				2
#define DW_IRQ_PIN				PIN_PA02
#define DW_IRQ_PIN_MUX			PIN_PA02A_EIC_EXTINT2
#endif
#define DW_IRQ_PIN_PULL			EXTINT_PULL_NONE
#define DW_IRQ_PIN_DETECT		EXTINT_DETECT_RISING

#ifdef BOARD_REV_A02
#define DW_RST_PIN				PIN_PA01
#define DW_RST_LINE				1
#define DW_RST_PIN_MUX			PIN_PA01A_EIC_EXTINT1
#endif
#ifdef BOARD_REV_A03
#define DW_RST_PIN				PIN_PA22
#define DW_RST_LINE				6
#define DW_RST_PIN_MUX			PIN_PA22A_EIC_EXTINT6
#endif

#define DW_RST_PIN_PULL			EXTINT_PULL_NONE
#define DW_RST_PIN_DETECT		EXTINT_DETECT_RISING
#define DW_RST_PIN_CONFIG		{ PORT_PIN_DIR_INPUT, PORT_PIN_PULL_NONE, false }

#endif





/** @} */

#ifdef __cplusplus
}
#endif

#endif // USER_BOARD_H
