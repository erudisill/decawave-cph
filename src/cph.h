/*
 * cph.h
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_H_
#define SRC_CPH_H_

#include <asf.h>

//#define MAIN_TEST


#include <cph_millis.h>
#include <cph_stdio.h>
#include <cph_queue.h>
#include <cph_deca.h>

#define	DEBUG_RECEIVE

#define SOFTWARE_VER_STRING  	"Version 0.01    "


#define TRACE(...)				printf(__VA_ARGS__)

#ifdef DEBUG_RECEIVE
#define MAC_ADDRESS				0xDECA010000000077
#else
#define MAC_ADDRESS				0xDECA010000000078
#endif

#define PAN_ID					0xDECA

#endif /* SRC_CPH_H_ */
