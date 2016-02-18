/*
 * cph.h
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_H_
#define SRC_CPH_H_

#define FW_MAJOR				0x01
#define FW_MINOR				0x01

//#define	ANCHOR
#define TAG

#define TRACE(...)				printf(__VA_ARGS__)

#define RANGE_METHOD_DS_TWR

//#define DEBUG_PRINT_RXFRAME


#include <asf.h>

#include <cph_millis.h>
#include <cph_stdio.h>
#include <cph_queue.h>
#include <cph_config.h>
#include <cph_utils.h>
#include <cph_deca.h>

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

extern int cph_mode;

extern uint16_t cph_coordid;


#endif /* SRC_CPH_H_ */
